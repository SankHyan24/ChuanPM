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
#pragma once
#include "Falcor.h"
#include "RenderGraph/RenderPass.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "Utils/Sampling/SampleGenerator.h"

using namespace Falcor;
/***
 * Create a new widget item (variables) u need to do these things:
 * 1. first, write the variable in shader (gVar)
 * Then finish the C++ program part
 * 2. create an anonymous namespace variable (about the string description, kVar)
 * 3. Create the variable itself in the Class (mVar)
 * 4. getprops and parseprops, bind then
 * 5. Add the widget item in RenderUI Function
 * Then bind the shader and C++ program
 * 6. bindshaderdata, bind C++ variable with shader bufname and varname
 */

class VBufferSC : public RenderPass
{
public:
    FALCOR_PLUGIN_CLASS(VBufferSC, "VBufferSC", "VBuffer for SC Photon Mapping");
    enum class SamplePattern : uint32_t
    {
        Center,
        DirectX,
        Halton,
        Stratified,
    };
    FALCOR_ENUM_INFO(
        SamplePattern,
        {
            {SamplePattern::Center, "Center"},
            {SamplePattern::DirectX, "DirectX"},
            {SamplePattern::Halton, "Halton"},
            {SamplePattern::Stratified, "Stratified"},
        }
    );

    static ref<VBufferSC> create(ref<Device> pDevice, const Properties& props) { return make_ref<VBufferSC>(pDevice, props); }

    VBufferSC(ref<Device> pDevice, const Properties& props);

    virtual Properties getProperties() const override;
    virtual RenderPassReflection reflect(const CompileData& compileData) override;
    virtual void compile(RenderContext* pRenderContext, const CompileData& compileData) override {}
    virtual void execute(RenderContext* pRenderContext, const RenderData& renderData) override;
    virtual void renderUI(Gui::Widgets& widget) override;
    virtual void setScene(RenderContext* pRenderContext, const ref<Scene>& pScene);
    virtual bool onMouseEvent(const MouseEvent& mouseEvent) override { return false; }
    virtual bool onKeyEvent(const KeyboardEvent& keyEvent) override { return false; }

private:
    void parseProperties(const Properties& props);

    void executeRaytrace(RenderContext* pRenderContext, const RenderData& renderData);
    void executeCompute(RenderContext* pRenderContext, const RenderData& renderData);

    DefineList getShaderDefines(const RenderData& renderData) const;
    void bindShaderData(const ShaderVar& var, const RenderData& renderData);
    void recreatePrograms();

    // From GBuffer
    virtual void setCullMode(RasterizerState::CullMode mode) { mCullMode = mode; }
    void updateFrameDim(const uint2 frameDim);
    void updateSamplePattern();
    ref<Texture> getOutput(const RenderData& renderData, const std::string& name) const;

    // From GBuffer
    ref<Scene> mpScene;
    /// Sample generator for camera jitter.
    ref<CPUSampleGenerator> mpSampleGenerator_base;

    /// Frames rendered since last change of scene. This is used as random seed.
    uint32_t mFrameCount = 0;
    /// Current frame dimension in pixels. Note this may be different from the window size.
    uint2 mFrameDim = {};
    float2 mInvFrameDim = {};
    ResourceFormat mVBufferFormat = HitInfo::kDefaultFormat;

    // UI variables

    /// Selected output size.
    RenderPassHelpers::IOSize mOutputSizeSelection = RenderPassHelpers::IOSize::Default;
    /// Output size in pixels when 'Fixed' size is selected.
    uint2 mFixedOutputSize = {512, 512};
    /// Which camera jitter sample pattern to use.
    SamplePattern mSamplePattern = SamplePattern::Center;
    /// Sample count for camera jitter.
    uint32_t mSampleCount = 16;
    /// Enable alpha test.
    bool mUseAlphaTest = true;
    /// Adjust shading normals.
    bool mAdjustShadingNormals = true;
    /// Force cull mode for all geometry, otherwise set it based on the scene.
    bool mForceCullMode = false;
    /// Cull mode to use for when mForceCullMode is true.
    RasterizerState::CullMode mCullMode = RasterizerState::CullMode::Back;

    /// Indicates whether any options that affect the output have changed since last frame.
    bool mOptionsChanged = false;

    // From VBufferRT

    // Internal state

    /// Flag indicating if depth-of-field is computed for the current frame.
    bool mComputeDOF = false;
    ref<SampleGenerator> mpSampleGenerator;

    // UI variables

    bool mUseTraceRayInline = false;
    /// Option for enabling depth-of-field when camera's aperture radius is nonzero.
    bool mUseDOF = true;

    struct
    {
        ref<Program> pProgram;
        ref<RtProgramVars> pVars;
    } mRaytrace;

    ref<ComputePass> mpComputePass;

    // My defines Configuration
    float mSpecRoughCutoff = 0.5f; ///< Cutoff for when all hits are counted diffuse.
    uint mRecursionDepth = 10;
};

FALCOR_ENUM_REGISTER(VBufferSC::SamplePattern);
