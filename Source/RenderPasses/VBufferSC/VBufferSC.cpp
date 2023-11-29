/***************************************************************************
 # Copyright (c) 2015-23, NVIDIA CORPORATION. All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions
 # are met:
 #  * Redistributions of source code must retain the above copyright
 #    notice, this list of conditions and the following disclaimer.
 #  * Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 #  * Neither the name of NVIDIA CORPORATION nor the names of its
 #    contributors may be used to endorse or promote products derived
 #    from this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
 # EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 # PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 # CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 # PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 # PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 # OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 # OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **************************************************************************/
#include "VBufferSC.h"
#include "RenderGraph/RenderPassStandardFlags.h"
#include "Utils/SampleGenerators/DxSamplePattern.h"
#include "Utils/SampleGenerators/HaltonSamplePattern.h"
#include "Utils/SampleGenerators/StratifiedSamplePattern.h"
#include "Scene/HitInfo.h"
#include "RenderGraph/RenderPassHelpers.h"

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, VBufferSC>();
}

// From GBufferBase
namespace
{
// Scripting options.
const char kOutputSize[] = "outputSize";
const char kFixedOutputSize[] = "fixedOutputSize";
const char kSamplePattern[] = "samplePattern";
const char kSampleCount[] = "sampleCount";
const char kUseAlphaTest[] = "useAlphaTest";
const char kDisableAlphaTest[] = "disableAlphaTest"; ///< Deprecated for "useAlphaTest".
const char kAdjustShadingNormals[] = "adjustShadingNormals";
const char kForceCullMode[] = "forceCullMode";
const char kCullMode[] = "cull";

} // namespace
// From VBufferRT
namespace
{
const std::string kProgramRaytraceFile = "RenderPasses/VBufferSC/VBufferSC.rt.slang";
const std::string kProgramComputeFile = "RenderPasses/VBufferSC/VBufferSC.cs.slang";

// Scripting options.
const char kUseTraceRayInline[] = "useTraceRayInline";
const char kUseDOF[] = "useDOF";

// Ray tracing settings that affect the traversal stack size. Set as small as possible.
// TODO: The shader doesn't need a payload, set this to zero if it's possible to pass a null payload to TraceRay()
const uint32_t kMaxPayloadSizeBytes = 64u;
const uint32_t kMaxRecursionDepth = 2u;

const std::string kVBufferName = "vbuffer";
const std::string kVBufferDesc = "V-buffer in packed format (indices + barycentrics)";

// Additional output channels.
const ChannelList kVBufferExtraChannels = {
    // clang-format off
    { "depth",          "gDepth",           "Depth buffer (NDC)",               true /* optional */, ResourceFormat::R32Float    },
    { "mvec",           "gMotionVector",    "Motion vector",                    true /* optional */, ResourceFormat::RG32Float   },
    { "viewW",          "gViewW",           "View direction in world space",    true /* optional */, ResourceFormat::RGBA32Float }, // TODO: Switch to packed 2x16-bit snorm format.
    { "time",           "gTime",            "Per-pixel execution time",         true /* optional */, ResourceFormat::R32Uint     },
    { "mask",           "gMask",            "Mask",                             true /* optional */, ResourceFormat::R32Float    },
    { "throughput",     "gThp",             "Throughput for transparent materials", true /* optional */, ResourceFormat::R32Float    },
    { "emissive",       "gEmissive",        "Emissive color",                   true /* optional */, ResourceFormat::R32Float    },
    // clang-format on
};
}; // namespace

// My defines
namespace
{
const char kSpecRoughCutoff[] = "specRoughCutoff";
const char kRecursionDepth[] = "recursionDepth";
}; // namespace

VBufferSC::VBufferSC(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    if (!mpDevice->isShaderModelSupported(ShaderModel::SM6_5))
        FALCOR_THROW("VBufferSC requires Shader Model 6.5 support.");
    if (!mpDevice->isFeatureSupported(Device::SupportedFeatures::RaytracingTier1_1))
        FALCOR_THROW("VBufferSC requires Raytracing Tier 1.1 support.");

    parseProperties(props);

    // Create sample generator
    mpSampleGenerator = SampleGenerator::create(mpDevice, SAMPLE_GENERATOR_DEFAULT);
    FALCOR_ASSERT(mpSampleGenerator);
}

Properties VBufferSC::getProperties() const
{
    Properties props;
    props[kOutputSize] = mOutputSizeSelection;
    if (mOutputSizeSelection == RenderPassHelpers::IOSize::Fixed)
        props[kFixedOutputSize] = mFixedOutputSize;
    props[kSamplePattern] = mSamplePattern;
    props[kSampleCount] = mSampleCount;
    props[kUseAlphaTest] = mUseAlphaTest;
    props[kAdjustShadingNormals] = mAdjustShadingNormals;
    props[kForceCullMode] = mForceCullMode;
    props[kCullMode] = mCullMode;
    props[kUseTraceRayInline] = mUseTraceRayInline;
    props[kUseDOF] = mUseDOF;
    props[kSpecRoughCutoff] = mSpecRoughCutoff;
    props[kRecursionDepth] = mRecursionDepth;

    return props;
}

RenderPassReflection VBufferSC::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    const uint2 sz = RenderPassHelpers::calculateIOSize(mOutputSizeSelection, mFixedOutputSize, compileData.defaultTexDims);

    // Add the required output. This always exists.
    reflector.addOutput(kVBufferName, kVBufferDesc)
        .bindFlags(ResourceBindFlags::UnorderedAccess)
        .format(mVBufferFormat)
        .texture2D(sz.x, sz.y);

    // Add all the other outputs.
    addRenderPassOutputs(reflector, kVBufferExtraChannels, ResourceBindFlags::UnorderedAccess, sz);

    return reflector;
}

void VBufferSC::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Update refresh flag if options that affect the output have changed.
    auto& dict = renderData.getDictionary();
    if (mOptionsChanged)
    {
        auto flags = dict.getValue(kRenderPassRefreshFlags, RenderPassRefreshFlags::None);
        dict[Falcor::kRenderPassRefreshFlags] = flags | Falcor::RenderPassRefreshFlags::RenderOptionsChanged;
        mOptionsChanged = false;
    }

    // Pass flag for adjust shading normals to subsequent passes via the dictionary.
    // Adjusted shading normals cannot be passed via the VBuffer, so this flag allows consuming passes to compute them when enabled.
    dict[Falcor::kRenderPassGBufferAdjustShadingNormals] = mAdjustShadingNormals;

    // Update frame dimension based on render pass output.
    auto pOutput = renderData.getTexture(kVBufferName);
    FALCOR_ASSERT(pOutput);
    updateFrameDim(uint2(pOutput->getWidth(), pOutput->getHeight()));

    // If there is no scene, clear the output and return.
    if (mpScene == nullptr)
    {
        pRenderContext->clearUAV(pOutput->getUAV().get(), uint4(0));
        clearRenderPassChannels(pRenderContext, kVBufferExtraChannels, renderData);
        return;
    }

    // Check for scene changes.
    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RecompileNeeded) ||
        is_set(mpScene->getUpdates(), Scene::UpdateFlags::GeometryChanged) ||
        is_set(mpScene->getUpdates(), Scene::UpdateFlags::SDFGridConfigChanged))
    {
        recreatePrograms();
    }

    // Configure depth-of-field.
    // When DOF is enabled, two PRNG dimensions are used. Pass this info to subsequent passes via the dictionary.
    mComputeDOF = mUseDOF && mpScene->getCamera()->getApertureRadius() > 0.f;
    if (mUseDOF)
    {
        renderData.getDictionary()[Falcor::kRenderPassPRNGDimension] = mComputeDOF ? 2u : 0u;
    }

    mUseTraceRayInline ? executeCompute(pRenderContext, renderData) : executeRaytrace(pRenderContext, renderData);

    mFrameCount++;
}

void VBufferSC::renderUI(Gui::Widgets& widget)
{
    // Controls for output size.
    // When output size requirements change, we'll trigger a graph recompile to update the render pass I/O sizes.
    if (widget.dropdown("Output size", mOutputSizeSelection))
        requestRecompile();
    if (mOutputSizeSelection == RenderPassHelpers::IOSize::Fixed)
    {
        if (widget.var("Size in pixels", mFixedOutputSize, 32u, 16384u))
            requestRecompile();
    }

    // Sample pattern controls.
    bool updatePattern = widget.dropdown("Sample pattern", mSamplePattern);
    widget.tooltip(
        "Selects sample pattern for anti-aliasing over multiple frames.\n\n"
        "The camera jitter is set at the start of each frame based on the chosen pattern.\n"
        "All render passes should see the same jitter.\n"
        "'Center' disables anti-aliasing by always sampling at the center of the pixel.",
        true
    );
    if (mSamplePattern != SamplePattern::Center)
    {
        updatePattern |= widget.var("Sample count", mSampleCount, 1u);
        widget.tooltip("Number of samples in the anti-aliasing sample pattern.", true);
    }
    if (updatePattern)
    {
        updateSamplePattern();
        mOptionsChanged = true;
    }

    // Misc controls.
    mOptionsChanged |= widget.checkbox("Alpha Test", mUseAlphaTest);
    widget.tooltip("Use alpha testing on non-opaque triangles.");

    mOptionsChanged |= widget.checkbox("Adjust shading normals", mAdjustShadingNormals);
    widget.tooltip("Enables adjustment of the shading normals to reduce the risk of black pixels due to back-facing vectors.", true);

    // Cull mode controls.
    mOptionsChanged |= widget.checkbox("Force cull mode", mForceCullMode);
    widget.tooltip(
        "Enable this option to override the default cull mode.\n\n"
        "Otherwise the default for rasterization is to cull backfacing geometry, "
        "and for ray tracing to disable culling.",
        true
    );

    if (mForceCullMode)
    {
        if (auto cullMode = mCullMode; widget.dropdown("Cull mode", cullMode))
        {
            setCullMode(cullMode);
            mOptionsChanged = true;
        }
    }

    if (widget.checkbox("Use TraceRayInline", mUseTraceRayInline))
    {
        mOptionsChanged = true;
    }

    if (widget.checkbox("Use depth-of-field", mUseDOF))
    {
        mOptionsChanged = true;
    }
    widget.tooltip(
        "This option enables stochastic depth-of-field when the camera's aperture radius is nonzero. "
        "Disable it to force the use of a pinhole camera.",
        true
    );
    if (widget.var("Specular Roughness Cutoff", mSpecRoughCutoff, 0.0f, 1.0f))
        requestRecompile();
    if (widget.var("path trace max depth", mRecursionDepth, 1u, 30u))
        requestRecompile();
}

void VBufferSC::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
{
    mpScene = pScene;
    mFrameCount = 0;
    updateSamplePattern();

    if (pScene)
    {
        // Trigger graph recompilation if we need to change the V-buffer format.
        ResourceFormat format = pScene->getHitInfo().getFormat();
        if (format != mVBufferFormat)
        {
            mVBufferFormat = format;
            requestRecompile();
        }
    }
    recreatePrograms();
}

// private
void VBufferSC::parseProperties(const Properties& props)
{
    for (const auto& [key, value] : props)
    {
        if (key == kOutputSize)
            mOutputSizeSelection = value;
        else if (key == kFixedOutputSize)
            mFixedOutputSize = value;
        else if (key == kSamplePattern)
            mSamplePattern = value;
        else if (key == kSampleCount)
            mSampleCount = value;
        else if (key == kUseAlphaTest)
            mUseAlphaTest = value;
        else if (key == kAdjustShadingNormals)
            mAdjustShadingNormals = value;
        else if (key == kForceCullMode)
            mForceCullMode = value;
        else if (key == kCullMode)
            mCullMode = value;
        else if (key == kSpecRoughCutoff)
            mSpecRoughCutoff = value;
        else if (key == kRecursionDepth)
            mRecursionDepth = value;
        // TODO: Check for unparsed fields, including those parsed in derived classes.
    }

    // Handle deprecated "disableAlphaTest" value.
    if (props.has(kDisableAlphaTest) && !props.has(kUseAlphaTest))
        mUseAlphaTest = !props[kDisableAlphaTest];

    for (const auto& [key, value] : props)
    {
        if (key == kUseTraceRayInline)
            mUseTraceRayInline = value;
        else if (key == kUseDOF)
            mUseDOF = value;
        // TODO: Check for unparsed fields, including those parsed in base classes.
    }
}

void VBufferSC::executeRaytrace(RenderContext* pRenderContext, const RenderData& renderData)
{
    if (!mRaytrace.pProgram || !mRaytrace.pVars)
    {
        DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add(mpSampleGenerator->getDefines());
        defines.add(getShaderDefines(renderData));

        // Create ray tracing program.
        ProgramDesc desc;
        desc.addShaderModules(mpScene->getShaderModules());
        desc.addShaderLibrary(kProgramRaytraceFile);
        desc.addTypeConformances(mpScene->getTypeConformances());
        desc.setMaxPayloadSize(kMaxPayloadSizeBytes);
        desc.setMaxAttributeSize(mpScene->getRaytracingMaxAttributeSize());
        desc.setMaxTraceRecursionDepth(kMaxRecursionDepth);

        ref<RtBindingTable> sbt = RtBindingTable::create(1, 1, mpScene->getGeometryCount());
        sbt->setRayGen(desc.addRayGen("rayGen"));
        sbt->setMiss(0, desc.addMiss("miss"));
        sbt->setHitGroup(0, mpScene->getGeometryIDs(Scene::GeometryType::TriangleMesh), desc.addHitGroup("closestHit", "anyHit"));

        // Add hit group with intersection shader for triangle meshes with displacement maps.
        if (mpScene->hasGeometryType(Scene::GeometryType::DisplacedTriangleMesh))
        {
            sbt->setHitGroup(
                0,
                mpScene->getGeometryIDs(Scene::GeometryType::DisplacedTriangleMesh),
                desc.addHitGroup("displacedTriangleMeshClosestHit", "", "displacedTriangleMeshIntersection")
            );
        }

        // Add hit group with intersection shader for curves (represented as linear swept spheres).
        if (mpScene->hasGeometryType(Scene::GeometryType::Curve))
        {
            sbt->setHitGroup(
                0, mpScene->getGeometryIDs(Scene::GeometryType::Curve), desc.addHitGroup("curveClosestHit", "", "curveIntersection")
            );
        }

        // Add hit group with intersection shader for SDF grids.
        if (mpScene->hasGeometryType(Scene::GeometryType::SDFGrid))
        {
            sbt->setHitGroup(
                0, mpScene->getGeometryIDs(Scene::GeometryType::SDFGrid), desc.addHitGroup("sdfGridClosestHit", "", "sdfGridIntersection")
            );
        }

        mRaytrace.pProgram = Program::create(mpDevice, desc, defines);
        mRaytrace.pVars = RtProgramVars::create(mpDevice, mRaytrace.pProgram, sbt);

        // Bind static resources.
        ShaderVar var = mRaytrace.pVars->getRootVar();
        mpSampleGenerator->bindShaderData(var);
    }

    mRaytrace.pProgram->addDefines(getShaderDefines(renderData));

    ShaderVar var = mRaytrace.pVars->getRootVar();
    bindShaderData(var, renderData);

    // Dispatch the rays.
    mpScene->raytrace(pRenderContext, mRaytrace.pProgram.get(), mRaytrace.pVars, uint3(mFrameDim, 1));
}

void VBufferSC::executeCompute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // Create compute pass.
    if (!mpComputePass)
    {
        ProgramDesc desc;
        desc.addShaderModules(mpScene->getShaderModules());
        desc.addShaderLibrary(kProgramComputeFile).csEntry("main");
        desc.addTypeConformances(mpScene->getTypeConformances());

        DefineList defines;
        defines.add(mpScene->getSceneDefines());
        defines.add(mpSampleGenerator->getDefines());
        defines.add(getShaderDefines(renderData));

        mpComputePass = ComputePass::create(mpDevice, desc, defines, true);

        // Bind static resources
        ShaderVar var = mpComputePass->getRootVar();
        mpScene->setRaytracingShaderData(pRenderContext, var);
        mpSampleGenerator->bindShaderData(var);
    }

    mpComputePass->getProgram()->addDefines(getShaderDefines(renderData));

    ShaderVar var = mpComputePass->getRootVar();
    bindShaderData(var, renderData);

    mpComputePass->execute(pRenderContext, uint3(mFrameDim, 1));
}

void VBufferSC::recreatePrograms()
{
    mRaytrace.pProgram = nullptr;
    mRaytrace.pVars = nullptr;
    mpComputePass = nullptr;
}

DefineList VBufferSC::getShaderDefines(const RenderData& renderData) const
{
    DefineList defines;
    defines.add("COMPUTE_DEPTH_OF_FIELD", mComputeDOF ? "1" : "0");
    defines.add("USE_ALPHA_TEST", mUseAlphaTest ? "1" : "0");

    // Setup ray flags.
    RayFlags rayFlags = RayFlags::None;
    if (mForceCullMode && mCullMode == RasterizerState::CullMode::Front)
        rayFlags = RayFlags::CullFrontFacingTriangles;
    else if (mForceCullMode && mCullMode == RasterizerState::CullMode::Back)
        rayFlags = RayFlags::CullBackFacingTriangles;
    defines.add("RAY_FLAGS", std::to_string((uint32_t)rayFlags));

    // For optional I/O resources, set 'is_valid_<name>' defines to inform the program of which ones it can access.
    // TODO: This should be moved to a more general mechanism using Slang.
    defines.add(getValidResourceDefines(kVBufferExtraChannels, renderData));
    return defines;
}

void VBufferSC::bindShaderData(const ShaderVar& var, const RenderData& renderData)
{
    var["gVBufferSC"]["frameDim"] = mFrameDim;
    var["gVBufferSC"]["frameCount"] = mFrameCount;
    var["CB"]["gSpecularRougnessCutoff"] = mSpecRoughCutoff;
    var["CB"]["gMaxRecursion"] = mRecursionDepth;

    // Bind resources.
    var["gVBuffer"] = getOutput(renderData, kVBufferName);

    // Bind output channels as UAV buffers.
    auto bind = [&](const ChannelDesc& channel)
    {
        ref<Texture> pTex = getOutput(renderData, channel.name);
        var[channel.texname] = pTex;
    };
    for (const auto& channel : kVBufferExtraChannels)
        bind(channel);
}

// From GBuffer
void VBufferSC::updateFrameDim(const uint2 frameDim)
{
    FALCOR_ASSERT(frameDim.x > 0 && frameDim.y > 0);
    mFrameDim = frameDim;
    mInvFrameDim = 1.f / float2(frameDim);

    // Update sample generator for camera jitter.
    if (mpScene)
        mpScene->getCamera()->setPatternGenerator(mpSampleGenerator_base, mInvFrameDim);
}

static ref<CPUSampleGenerator> createSamplePattern(VBufferSC::SamplePattern type, uint32_t sampleCount)
{
    switch (type)
    {
    case VBufferSC::SamplePattern::Center:
        return nullptr;
    case VBufferSC::SamplePattern::DirectX:
        return DxSamplePattern::create(sampleCount);
    case VBufferSC::SamplePattern::Halton:
        return HaltonSamplePattern::create(sampleCount);
    case VBufferSC::SamplePattern::Stratified:
        return StratifiedSamplePattern::create(sampleCount);
    default:
        FALCOR_UNREACHABLE();
        return nullptr;
    }
}

void VBufferSC::updateSamplePattern()
{
    mpSampleGenerator_base = createSamplePattern(mSamplePattern, mSampleCount);
    if (mpSampleGenerator_base)
        mSampleCount = mpSampleGenerator_base->getSampleCount();
}

ref<Texture> VBufferSC::getOutput(const RenderData& renderData, const std::string& name) const
{
    // This helper fetches the render pass output with the given name and verifies it has the correct size.
    FALCOR_ASSERT(mFrameDim.x > 0 && mFrameDim.y > 0);
    auto pTex = renderData.getTexture(name);
    if (pTex && (pTex->getWidth() != mFrameDim.x || pTex->getHeight() != mFrameDim.y))
    {
        FALCOR_THROW("GBufferBase: Pass output '{}' has mismatching size. All outputs must be of the same size.", name);
    }
    return pTex;
}
