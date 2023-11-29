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
#include "RTPM.h"
#include "RenderGraph/RenderPassHelpers.h"
#include "RenderGraph/RenderPassStandardFlags.h"

#include <random>
#include <ctime>
#include <limits>
#include <fstream>

extern "C" FALCOR_API_EXPORT void registerPlugin(Falcor::PluginRegistry& registry)
{
    registry.registerClass<RenderPass, RTPM>();
}

namespace
{
const char kShaderGeneratePhoton[] = "RenderPasses/HashPPM/PMGenerate.rt.slang";
const char kShaderCollectPhoton[] = "RenderPasses/HashPPM/PMCollect.cs.slang";

// Ray tracing settings that affect the traversal stack size.
// These should be set as small as possible.
const uint32_t kMaxPayloadSizeBytes = 64u;
const uint32_t kMaxPayloadSizeBytesCollect = 32u;
const uint32_t kMaxAttributeSizeBytes = 8u;
const uint32_t kMaxRecursionDepth = 2u;

const ChannelList kInputChannels = {
    {"vbuffer", "gVBuffer", "V Buffer to get the intersected triangle", false},
    {"viewW", "gViewWorld", "World View Direction", false},
    {"thp", "gThp", "Throughput", false},
    {"emissive", "gEmissive", "Emissive", false},
};

const ChannelList kOutputChannels = {
    {"PhotonImage",
     "gPhotonImage",
     "An image that shows the caustics and indirect light from global photons",
     false,
     ResourceFormat::RGBA32Float}};

const Gui::DropdownList kInfoTexDropdownList{//{(uint)PhotonMapperHash::TextureFormat::_8Bit , "8Bits"},
                                             {(uint)RTPM::TextureFormat::_16Bit, "16Bits"},
                                             {(uint)RTPM::TextureFormat::_32Bit, "32Bits"}};

const Gui::DropdownList kLightTexModeList{{RTPM::LightTexMode::power, "Power"}, {RTPM::LightTexMode::area, "Area"}};

const Gui::DropdownList kCausticMapModes{{0, "LS+D"}, {1, "L(S|D)*SD"}};
} // namespace

RTPM::RTPM(ref<Device> pDevice, const Properties& props) : RenderPass(pDevice)
{
    parseProperties(props);
    mpSampleGenerator = SampleGenerator::create(pDevice, SAMPLE_GENERATOR_UNIFORM);
    FALCOR_ASSERT(mpSampleGenerator);
}

void RTPM::parseProperties(const Properties& props)
{
    for (const auto& [key, value] : props)
    {
        // sc: no need now
    }
}

Properties RTPM::getProperties() const
{
    Properties props;
    // sc: not have now
    return props;
}

RenderPassReflection RTPM::reflect(const CompileData& compileData)
{
    // Define the required resources here
    RenderPassReflection reflector;
    // reflector.addOutput("dst");
    // reflector.addInput("src");
    addRenderPassInputs(reflector, kInputChannels);
    addRenderPassOutputs(reflector, kOutputChannels);
    return reflector;
}

void RTPM::execute(RenderContext* pRenderContext, const RenderData& renderData)
{
    // renderData holds the requested resources
    // auto& pTexture = renderData.getTexture("src");
}

void RTPM::renderUI(Gui::Widgets& widget)
{
    float2 dummySpacing = float2(0, 10);
    bool dirty = false;

    // Info
    widget.text("Iterations: " + std::to_string(mFrameCount));
    widget.text("Caustic Photons: " + std::to_string(mPhotonCount[0]) + " / " + std::to_string(mCausticBuffers.maxSize));
    widget.tooltip("Photons for current Iteration / Buffer Size");
    widget.text("Global Photons: " + std::to_string(mPhotonCount[1]) + " / " + std::to_string(mGlobalBuffers.maxSize));
    widget.tooltip("Photons for current Iteration / Buffer Size");

    widget.text("Current Global Radius: " + std::to_string(mGlobalRadius));
    widget.text("Current Caustic Radius: " + std::to_string(mCausticRadius));

    widget.dummy("", dummySpacing);
    widget.var("Number Photons", mNumPhotonsUI, 1000u, UINT_MAX, 1000u);
    widget.tooltip("The number of photons that are shot per iteration. Press \"Apply\" to apply the change");
    widget.var("Size Caustic Buffer", mCausticBufferSizeUI, 1000u, UINT_MAX, 1000u);
    widget.var("Size Global Buffer", mGlobalBufferSizeUI, 1000u, UINT_MAX, 1000u);
    mNumPhotonsChanged |= widget.button("Apply");
    widget.dummy("", float2(15, 0), true);
    mFitBuffersToPhotonShot |= widget.button("Fit Buffers", true);
    widget.tooltip("Fitts the Caustic and Global Buffer to current number of photons shot + 10 %");
    widget.dummy("", dummySpacing);

    // If fit buffers is triggered, also trigger the photon change routine
    mNumPhotonsChanged |= mFitBuffersToPhotonShot;

    // Progressive PM
    dirty |= widget.checkbox("Use SPPM", mUseStatisticProgressivePM);
    widget.tooltip("Activate Statistically Progressive Photon Mapping");

    if (mUseStatisticProgressivePM)
    {
        dirty |= widget.var("Global Alpha", mSPPMAlphaGlobal, 0.1f, 1.0f, 0.001f);
        widget.tooltip("Sets the Alpha in SPPM for the Global Photons");
        dirty |= widget.var("Caustic Alpha", mSPPMAlphaCaustic, 0.1f, 1.0f, 0.001f);
        widget.tooltip("Sets the Alpha in SPPM for the Caustic Photons");
    }

    widget.dummy("", dummySpacing);
    // miscellaneous
    dirty |= widget.slider("Max Recursion Depth", mMaxBounces, 1u, 32u);
    widget.tooltip("Maximum path length for Photon Bounces");
    mResetCS |= widget.checkbox("Use Photon Face Normal Rejection", mEnableFaceNormalRejection);
    widget.tooltip("Uses encoded Face Normal to reject photon hits on different surfaces (corners / other side of wall).");
    dirty |= mResetCS;
    dirty |= widget.dropdown("Caustic Map Definition", kCausticMapModes, mCausticMapMultipleDiffuseHits);
    widget.tooltip(
        "Changes definition of the caustic photons map. L(S|D)SD path will store way more stray caustic photons, but allows caustics from "
        "indirect illuminated surfaces"
    );

    widget.dummy("", dummySpacing);

    // Timer
    if (auto group = widget.group("Timer"))
    {
        bool resetTimer = false;
        resetTimer |= widget.checkbox("Enable Timer", mUseTimer);
        widget.tooltip("Enables the timer");
        if (mUseTimer)
        {
            uint sec = static_cast<uint>(mTimerDurationSec);
            if (sec != 0)
                widget.text("Elapsed seconds: " + std::to_string(mCurrentElapsedTime) + " / " + std::to_string(sec));
            if (mTimerMaxIterations != 0)
                widget.text("Iterations: " + std::to_string(mFrameCount) + " / " + std::to_string(mTimerMaxIterations));
            resetTimer |= widget.var("Timer Seconds", sec, 0u, UINT_MAX, 1u);
            widget.tooltip("Time in seconds needed to stop rendering. When 0 time is not used");
            resetTimer |= widget.var("Max Iterations", mTimerMaxIterations, 0u, UINT_MAX, 1u);
            widget.tooltip("Max iterations until stop. When 0 iterations are not used");
            mTimerDurationSec = static_cast<double>(sec);
            resetTimer |= widget.checkbox("Record Times", mTimerRecordTimes);
            resetTimer |= widget.button("Reset Timer");
            if (mTimerRecordTimes)
            {
                if (widget.button("Store Times", true))
                {
                    FileDialogFilterVec filters;
                    filters.push_back({"csv", "CSV Files"});
                    std::filesystem::path path;
                    if (saveFileDialog(filters, path))
                    {
                        mTimesOutputFilePath = path.string();
                        outputTimes();
                    }
                }
            }
        }
        mResetTimer |= resetTimer;
        dirty |= resetTimer;
    }

    // Radius settings
    if (auto group = widget.group("Radius Options"))
    {
        dirty |= widget.var("Caustic Radius Start", mCausticRadiusStart, kMinPhotonRadius, FLT_MAX, 0.001f);
        widget.tooltip("The start value for the radius of caustic Photons");
        dirty |= widget.var("Global Radius Start", mGlobalRadiusStart, kMinPhotonRadius, FLT_MAX, 0.001f);
        widget.tooltip("The start value for the radius of global Photons");
        dirty |= widget.var("Russian Roulette", mRussianRoulette, 0.001f, 1.f, 0.001f);
        widget.tooltip("Probabilty that a Global Photon is saved");
    }
    // Material Settings
    if (auto group = widget.group("Material Options"))
    {
        dirty |= widget.var("Emissive Scalar", mIntensityScalar, 0.0f, FLT_MAX, 0.001f);
        widget.tooltip("Scales the intensity of all emissive Light Sources");
        dirty |= widget.var("SpecRoughCutoff", mSpecRoughCutoff, 0.0f, 1.0f, 0.01f);
        widget.tooltip("The cutoff for Specular Materials. All Reflections above this threshold are considered Diffuse");
        dirty |= widget.checkbox("Alpha Test", mUseAlphaTest);
        widget.tooltip("Enables Alpha Test for Photon Generation");
        dirty |= widget.checkbox("Adjust Shading Normals", mAdjustShadingNormals);
        widget.tooltip("Adjusts the shading normals in the Photon Generation");
    }
    // Hash Settings
    if (auto group = widget.group("Hash Options"))
    {
        dirty |= widget.var("Quadradic Probe Iterations", mQuadraticProbeIterations, 0u, 100u, 1u);
        widget.tooltip("Max iterations that are used for quadratic probe");
        mResetCS |= widget.slider("Num Photons per bucket", mNumPhotonsPerBucket, 2u, 32u);
        widget.tooltip("Max number of photons that can be saved in a hash grid");
        mResetCS |= widget.slider("Bucket size (bits)", mNumBucketBits, 2u, 32u);
        widget.tooltip("Bucket size in 2^x. One bucket takes 16Byte + Num photons per bucket * 4 Byte");

        dirty |= mResetCS;
    }

    if (auto group = widget.group("Light Sample Tex"))
    {
        mRebuildLightTex |= widget.dropdown("Sample mode", kLightTexModeList, (uint32_t&)mLightTexMode);
        widget.tooltip("Changes photon distribution for the light sampling texture. Also rebuilds the texture.");
        mRebuildLightTex |= widget.button("Rebuild Light Tex");
        dirty |= mRebuildLightTex;
    }

    mPhotonInfoFormatChanged |= widget.dropdown("Photon Info size", kInfoTexDropdownList, mInfoTexFormat);
    widget.tooltip("Determines the resolution of each element of the photon info struct.");

    dirty |= mPhotonInfoFormatChanged; // Reset iterations if format is changed

    // Disable Photon Collecion
    if (auto group = widget.group("Collect Options"))
    {
        dirty |= widget.checkbox("Disable Global Photons", mDisableGlobalCollection);
        widget.tooltip("Disables the collection of Global Photons. However they will still be generated");
        dirty |= widget.checkbox("Disable Caustic Photons", mDisableCausticCollection);
        widget.tooltip("Disables the collection of Caustic Photons. However they will still be generated");
        dirty |= widget.checkbox("Stochastic Collection", mEnableStochasticCollection);
        widget.tooltip("Enables stochastic collection. A geometrically distributed random step is used for that");
        if (mEnableStochasticCollection)
        {
            dirty |= widget.slider("Stochastic Collection Probability", mStochasticCollectProbability, 0.0001f, 1.0f);
            widget.tooltip("Probability for the geometrically distributed random step");
        }
    }
    widget.dummy("", dummySpacing);
    // Reset Iterations
    widget.checkbox("Always Reset Iterations", mAlwaysResetIterations);
    widget.tooltip("Always Resets the Iterations, currently good for moving the camera");
    mResetIterations |= widget.button("Reset Iterations");
    widget.tooltip("Resets the iterations");
    dirty |= mResetIterations;

    // set flag to indicate that settings have changed and the pass has to be rebuild
    if (dirty)
        mOptionsChanged = true;
}

void RTPM::checkTimer()
{
    if (!mUseTimer)
        return;

    // reset timer
    if (mResetTimer)
    {
        mCurrentElapsedTime = 0.0;
        mTimerStartTime = std::chrono::steady_clock::now();
        mTimerStopRenderer = false;
        mResetTimer = false;
        if (mTimerRecordTimes)
        {
            mTimesList.clear();
            mTimesList.reserve(10000);
        }
        return;
    }

    if (mTimerStopRenderer)
        return;

    // check time
    if (mTimerDurationSec != 0)
    {
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedSec = currentTime - mTimerStartTime;
        mCurrentElapsedTime = elapsedSec.count();

        if (mTimerDurationSec <= mCurrentElapsedTime)
        {
            mTimerStopRenderer = true;
        }
    }

    // check iterations
    if (mTimerMaxIterations != 0)
    {
        if (mTimerMaxIterations <= mFrameCount)
        {
            mTimerStopRenderer = true;
        }
    }

    // Add to times list
    if (mTimerRecordTimes)
    {
        mTimesList.push_back(mCurrentElapsedTime);
    }
}

void RTPM::outputTimes()
{
    if (mTimesOutputFilePath.empty() || mTimesList.empty())
        return;

    std::ofstream file = std::ofstream(mTimesOutputFilePath, std::ios::trunc);

    if (!file)
    {
        FALCOR_THROW(fmt::format("Failed to open file '{}'.", mTimesOutputFilePath));
        mTimesOutputFilePath.clear();
        return;
    }

    // Write into file
    file << "Hash_Times" << std::endl;
    file << std::fixed << std::setprecision(16);
    for (size_t i = 0; i < mTimesList.size(); i++)
    {
        file << mTimesList[i];
        file << std::endl;
    }
    file.close();
}
