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
#include "UVMeshPass.h"
#include "Scene/HitInfo.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"
#include "RenderGraph/RenderPassStandardFlags.h"
#include "Utils/SampleGenerators/DxSamplePattern.h"
#include "Utils/SampleGenerators/HaltonSamplePattern.h"
#include "Utils/SampleGenerators/StratifiedSamplePattern.h"

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, UVMeshPass>();
}

namespace
{
// from gbufferbase
const char kOutputSize[] = "outputSize";
const char kFixedOutputSize[] = "fixedOutputSize";
const char kSamplePattern[] = "samplePattern";
const char kSampleCount[] = "sampleCount";
const char kUseAlphaTest[] = "useAlphaTest";
const char kDisableAlphaTest[] = "disableAlphaTest"; ///< Deprecated for "useAlphaTest".
const char kAdjustShadingNormals[] = "adjustShadingNormals";
const char kForceCullMode[] = "forceCullMode";
const char kCullMode[] = "cull";
// from vbufferraster
const std::string kProgramFile = "RenderPasses/UVMeshPass/UVMeshPass.3d.slang";
const RasterizerState::CullMode kDefaultCullMode = RasterizerState::CullMode::Back;

const std::string kVBufferName = "vbuffer";
const std::string kVBufferDesc = "V-buffer in packed format (indices + barycentrics)";

const ChannelList kVBufferExtraChannels = {
    // clang-format off
    { "mvec",           "gMotionVector",    "Motion vector",                true /* optional */, ResourceFormat::RG32Float   },
    { "mask",           "gMask",            "Mask",                         true /* optional */, ResourceFormat::R32Float    },
    { "uv",           "gUV",            "UV map",                         true /* optional */, ResourceFormat::RG32Float    },
    { "meshid",           "gMeshID",            "Mesh ID",                         true /* optional */, ResourceFormat::R32Int    },
    // clang-format on
};

const std::string kDepthName = "depth";
} // namespace

UVMeshPass::UVMeshPass(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    // Check for required features.
    if (!mpDevice->isShaderModelSupported(ShaderModel::SM6_2))
        FALCOR_THROW("VBufferRaster requires Shader Model 6.2 support.");
    if (!mpDevice->isFeatureSupported(Device::SupportedFeatures::Barycentrics))
        FALCOR_THROW("VBufferRaster requires pixel shader barycentrics support.");
    if (!mpDevice->isFeatureSupported(Device::SupportedFeatures::RasterizerOrderedViews))
        FALCOR_THROW("VBufferRaster requires rasterizer ordered views (ROVs) support.");

    parseProperties(props);

    // Initialize graphics state
    mRaster.pState = GraphicsState::create(mpDevice);

    // Set depth function
    DepthStencilState::Desc dsDesc;
    dsDesc.setDepthFunc(ComparisonFunc::LessEqual).setDepthWriteMask(true);
    mRaster.pState->setDepthStencilState(DepthStencilState::create(dsDesc));

    mpFbo = Fbo::create(mpDevice);
}

Properties UVMeshPass::getProperties() const
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
    return props;
}

void UVMeshPass::renderUI(Gui::Widgets& widget)
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
}

RenderPassReflection UVMeshPass::reflect(const CompileData& compileData)
{
    RenderPassReflection reflector;
    const uint2 sz = RenderPassHelpers::calculateIOSize(mOutputSizeSelection, mFixedOutputSize, compileData.defaultTexDims);

    // Add the required outputs. These always exist.
    reflector.addOutput(kDepthName, "Depth buffer")
        .format(ResourceFormat::D32Float)
        .bindFlags(ResourceBindFlags::DepthStencil)
        .texture2D(sz.x, sz.y);
    reflector.addOutput(kVBufferName, kVBufferDesc)
        .bindFlags(ResourceBindFlags::RenderTarget | ResourceBindFlags::UnorderedAccess)
        .format(mVBufferFormat)
        .texture2D(sz.x, sz.y);

    // Add all the other outputs.
    addRenderPassOutputs(reflector, kVBufferExtraChannels, ResourceBindFlags::UnorderedAccess, sz);

    return reflector;
}

void UVMeshPass::setScene(RenderContext* pRenderContext, const ref<Scene>& pScene)
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

    if (pScene)
    {
        if (pScene->getMeshVao() && pScene->getMeshVao()->getPrimitiveTopology() != Vao::Topology::TriangleList)
        {
            FALCOR_THROW("VBufferRaster: Requires triangle list geometry due to usage of SV_Barycentrics.");
        }
    }
}

void UVMeshPass::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
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

    // Clear depth and output buffer.
    auto pDepth = getOutput(renderData, kDepthName);
    pRenderContext->clearUAV(pOutput->getUAV().get(), uint4(0)); // Clear as UAV for integer clear value
    pRenderContext->clearDsv(pDepth->getDSV().get(), 1.f, 0);

    // Clear extra output buffers.
    clearRenderPassChannels(pRenderContext, kVBufferExtraChannels, renderData);

    // If there is no scene, we're done.
    if (mpScene == nullptr)
    {
        return;
    }

    // Check for scene changes.
    if (is_set(mpScene->getUpdates(), Scene::UpdateFlags::RecompileNeeded))
    {
        recreatePrograms();
    }

    // Create raster program.
    if (!mRaster.pProgram)
    {
        ProgramDesc desc;
        desc.addShaderModules(mpScene->getShaderModules());
        desc.addShaderLibrary(kProgramFile).vsEntry("vsMain").psEntry("psMain");
        desc.addTypeConformances(mpScene->getTypeConformances());

        mRaster.pProgram = Program::create(mpDevice, desc, mpScene->getSceneDefines());
        mRaster.pState->setProgram(mRaster.pProgram);
    }

    // Set program defines.
    mRaster.pProgram->addDefine("USE_ALPHA_TEST", mUseAlphaTest ? "1" : "0");

    // For optional I/O resources, set 'is_valid_<name>' defines to inform the program of which ones it can access.
    // TODO: This should be moved to a more general mechanism using Slang.
    mRaster.pProgram->addDefines(getValidResourceDefines(kVBufferExtraChannels, renderData));

    // Create program vars.
    if (!mRaster.pVars)
    {
        mRaster.pVars = ProgramVars::create(mpDevice, mRaster.pProgram.get());
    }

    mpFbo->attachColorTarget(pOutput, 0);
    mpFbo->attachDepthStencilTarget(pDepth);
    mRaster.pState->setFbo(mpFbo); // Sets the viewport

    auto var = mRaster.pVars->getRootVar();
    var["PerFrameCB"]["gFrameDim"] = mFrameDim;

    // Bind extra channels as UAV buffers.
    for (const auto& channel : kVBufferExtraChannels)
    {
        ref<Texture> pTex = getOutput(renderData, channel.name);
        var[channel.texname] = pTex;
    }

    // Rasterize the scene.
    RasterizerState::CullMode cullMode = mForceCullMode ? mCullMode : kDefaultCullMode;
    mpScene->rasterize(pRenderContext, mRaster.pState.get(), mRaster.pVars.get(), cullMode);
}

void UVMeshPass::recreatePrograms()
{
    mRaster.pProgram = nullptr;
    mRaster.pVars = nullptr;
}

void UVMeshPass::parseProperties(const Properties& props)
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
        // TODO: Check for unparsed fields, including those parsed in derived classes.
    }

    // Handle deprecated "disableAlphaTest" value.
    if (props.has(kDisableAlphaTest) && !props.has(kUseAlphaTest))
        mUseAlphaTest = !props[kDisableAlphaTest];
}

void UVMeshPass::updateFrameDim(const uint2 frameDim)
{
    FALCOR_ASSERT(frameDim.x > 0 && frameDim.y > 0);
    mFrameDim = frameDim;
    mInvFrameDim = 1.f / float2(frameDim);

    // Update sample generator for camera jitter.
    if (mpScene)
        mpScene->getCamera()->setPatternGenerator(mpSampleGenerator, mInvFrameDim);
}

static ref<CPUSampleGenerator> createSamplePattern(UVMeshPass::SamplePattern type, uint32_t sampleCount)
{
    switch (type)
    {
    case UVMeshPass::SamplePattern::Center:
        return nullptr;
    case UVMeshPass::SamplePattern::DirectX:
        return DxSamplePattern::create(sampleCount);
    case UVMeshPass::SamplePattern::Halton:
        return HaltonSamplePattern::create(sampleCount);
    case UVMeshPass::SamplePattern::Stratified:
        return StratifiedSamplePattern::create(sampleCount);
    default:
        FALCOR_UNREACHABLE();
        return nullptr;
    }
}

void UVMeshPass::updateSamplePattern()
{
    mpSampleGenerator = createSamplePattern(mSamplePattern, mSampleCount);
    if (mpSampleGenerator)
        mSampleCount = mpSampleGenerator->getSampleCount();
}

ref<Texture> UVMeshPass::getOutput(const RenderData& renderData, const std::string& name) const
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
