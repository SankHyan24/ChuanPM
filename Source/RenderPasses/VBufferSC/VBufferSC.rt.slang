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
#include "Scene/SceneDefines.slangh"
#include "Utils/Math/MathConstants.slangh"

import Scene.Raytracing;
import Scene.Intersection;
import Scene.SDFs.SDFGridHitData;
import Utils.Math.MathHelpers;
import Utils.Geometry.GeometryHelpers;
import Utils.Sampling.SampleGenerator;
import Rendering.Lights.LightHelpers;
import VBufferSC;

ConstantBuffer<VBufferSC> gVBufferSC;

cbuffer CB
{
    bool gAdjustShadingNormals;    // Adjusts the shading normals if activated
    bool gUseAlphaTest;            // uses alpha test if activated
    bool gUseRandomPixelPosCamera; // Uses a randomly generated pixel offset for camera direction sampeling instead of fixed jitter
    //
    float gSpecularRougnessCutoff; // cutoff for when a hit counts as diffuse
    uint gMaxRecursion;            // Max Recursion of ray trace path
}

struct RayData
{
    // TODO: Passing in an empty payload struct doesn't work. Declare a dummy variable so that the compiler doesn't remove the declaration.
    float3 thp;       ///< Current path throughput. This is updated at each path vertex.
    bool terminated;  ///< Set to true when path is terminated.
    float3 origin;    ///< Next path segment origin.
    uint pathLength;  ///< Length of the path
    float3 direction; ///< Next path segment direction.
    float distance;

    SampleGenerator sg; ///< Per-ray state for the sample generator (up to 16B).

    /** Create ray payload with default parameters.
     */
    __init(SampleGenerator sg)
    {
        this.thp = float3(1.0, 1.0, 1.0);
        this.terminated = false;
        this.origin = float3(0);
        this.distance = 0.f;
        this.direction = float3(0);
        this.pathLength = 0;
        this.sg = sg;
    }
};

/**
 * Setup ShadingData based on loaded vertex/material attributes for a hit point.
 * @param[in] hit Hit information.
 * @param[in] rayOrigin Ray origin.
 * @param[in] rayDir Normalized ray direction.
 * @param[in] lod Method for computing texture level-of-detail.
 * @return ShadingData struct.
 */
ShadingData loadShadingData(const HitInfo hit, const float3 rayOrigin, const float3 rayDir, const ITextureSampler lod)
{
    VertexData v = {};
    uint materialID = {};

#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_TRIANGLE_MESH)
    if (hit.getType() == HitType::Triangle)
    {
        const TriangleHit triangleHit = hit.getTriangleHit();
        v = gScene.getVertexData(triangleHit);
        materialID = gScene.getMaterialID(triangleHit.instanceID);
    }
#endif
#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_DISPLACED_TRIANGLE_MESH)
    if (hit.getType() == HitType::DisplacedTriangle)
    {
        const DisplacedTriangleHit displacedTriangleHit = hit.getDisplacedTriangleHit();
        v = gScene.getVertexData(displacedTriangleHit, -rayDir);
        materialID = gScene.getMaterialID(displacedTriangleHit.instanceID);
    }
#endif
#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_CURVE)
    if (hit.getType() == HitType::Curve)
    {
        const CurveHit curveHit = hit.getCurveHit();
        v = gScene.getVertexDataFromCurve(curveHit);
        materialID = gScene.getMaterialID(curveHit.instanceID);
    }
#endif
#if SCENE_HAS_GEOMETRY_TYPE(GEOMETRY_TYPE_SDF_GRID)
    if (hit.getType() == HitType::SDFGrid)
    {
        const SDFGridHit sdfGridHit = hit.getSDFGridHit();
        v = gScene.getVertexDataFromSDFGrid(sdfGridHit, rayOrigin, rayDir);
        materialID = gScene.getMaterialID(sdfGridHit.instanceID);
    }
#endif

    ShadingData sd = gScene.materials.prepareShadingData(v, materialID, -rayDir, lod);

    return sd;
}

bool generateNewRay(
    const ShadingData sd,
    const IMaterialInstance mi,
    bool isCurveHit,
    float3 rayOrigin,
    inout BSDFSample bsdfSample,
    inout RayData rayData
)
{
    if (mi.sample(sd, rayData.sg, bsdfSample, true)) // sc: deflaut use IS
    {
        rayData.origin = rayOrigin;
        if (!isCurveHit && bsdfSample.isLobe(LobeType::Transmission))
        {
            rayData.origin = sd.computeRayOrigin(false);
        }
        rayData.direction = bsdfSample.wo;
        rayData.thp *= bsdfSample.weight;
        return any(rayData.thp > 0.f);
    }
    return false;
}

void handleHit(const HitInfo hit, inout RayData rayData)
{
    uint2 launchIndex = DispatchRaysIndex().xy;
    const bool isCurveHit = hit.getType() == HitType::Curve;
    let lod = ExplicitLodTextureSampler(0.f);

    // load shading data
    ShadingData sd = loadShadingData(hit, rayData.origin, normalize(rayData.direction), lod);
    // not implement adjust shading normal yet

    // create material instance
    bool isPrimaryHit = rayData.pathLength == 0;
    let hints = getMaterialInstanceHints(hit, isPrimaryHit);
    const IMaterialInstance mi = gScene.materials.getMaterialInstance(sd, lod, hints);
    // const IMaterialInstance mi = gScene.materials.getMaterialInstance(sd, lod);
    // emission
    float3 throughPut = rayData.thp; // 当前的throughput，还没经过BSDF的weight（还没射出去）
    float3 emissiveRadiance;         // 这个emissive只乘上眼睛到hit的距离（bsdfweight），不乘上throughput

    // Compute ray origin for new rays spawned from the hit.
    float3 rayOrigin;
    if (isCurveHit)
    {
        // For curves, we set the new origin at the sphere center.
        rayOrigin = sd.posW - sd.curveRadius * sd.frame.N;
    }
    else
    {
        rayOrigin = sd.computeRayOrigin();
    }

    rayData.distance = distance(ObjectRayOrigin(), sd.posW); // accumulate ray distance // SC:Refine
    // rayData.distance += distance(ObjectRayOrigin(), sd.posW); // accumulate ray distance

    BSDFSample bsdfSample;
    bool canGenerateNewRay = generateNewRay(sd, mi, isCurveHit, rayOrigin, bsdfSample, rayData);

    // if diffuse, if so, write hit and terminate
    BSDFProperties bsdfProperties = mi.getProperties(sd);
    // bool isDiffuse = bsdfProperties.roughness > gSpecularRougnessCutoff;
    bool isDiffuse = bsdfProperties.roughness > gSpecularRougnessCutoff;
    // if ((bsdfSample.isLobe(LobeType::DiffuseReflection) || (bsdfSample.isLobe(LobeType::Reflection) && isDiffuse)) &&
    //     (!rayData.terminated || isDiffuse))
    if ((bsdfSample.isLobe(LobeType::DiffuseReflection) || (bsdfSample.isLobe(LobeType::Reflection) && isDiffuse)))
    // if (true)
    {
        gVBufferSC.writeHit(
            launchIndex,
            WorldRayOrigin(),
            WorldRayDirection(),
            throughPut,
            mi.getProperties(sd).emission * bsdfSample.weight,
            hit,
            rayData.distance
        );
        rayData.terminated = true;
    }
    if (!canGenerateNewRay)
    {
        rayData.terminated = true;
        // rayData.distance = -1.f;
        gVBufferSC.writeMiss(launchIndex);
        return;
    }

    rayData.pathLength++;
}

//
// Shader entry point for miss shader.
//

[shader("miss")]
void miss(inout RayData rayData)
{
    // Write invalid hit to output buffer.
    uint2 launchIndex = DispatchRaysIndex().xy;
    rayData.terminated = true;
    gVBufferSC.writeMiss(launchIndex);
}

//
// Shader entry points for TriangleMesh hit groups.
//

[shader("anyhit")]
void anyHit(inout RayData rayData, BuiltInTriangleIntersectionAttributes attribs)
{
    if (VBufferSC::kUseAlphaTest)
    {
        // Alpha test for non-opaque geometry.
        GeometryInstanceID instanceID = getGeometryInstanceID();
        VertexData v = getVertexData(instanceID, PrimitiveIndex(), attribs);
        uint materialID = gScene.getMaterialID(instanceID);
        if (gScene.materials.alphaTest(v, materialID, 0.f))
            IgnoreHit();
    }
}

[shader("closesthit")]
void closestHit(inout RayData rayData, BuiltInTriangleIntersectionAttributes attribs)
{
    // Store hit information. Note we don't access the materials here.
    TriangleHit triangleHit;
    triangleHit.instanceID = getGeometryInstanceID();
    triangleHit.primitiveIndex = PrimitiveIndex();
    triangleHit.barycentrics = attribs.barycentrics;

    handleHit(HitInfo(triangleHit), rayData);
    // preserved
}

//
// Shader entry points for DisplacedTriangleMesh hit groups.
//

[shader("intersection")]
void displacedTriangleMeshIntersection()
{
    const Ray ray = Ray(WorldRayOrigin(), WorldRayDirection(), RayTMin(), RayTCurrent());
    DisplacedTriangleMeshIntersector::Attribs attribs;
    float t;
    if (DisplacedTriangleMeshIntersector::intersect(ray, getGeometryInstanceID(), PrimitiveIndex(), attribs, t))
    {
        ReportHit(t, 0, attribs);
    }
}

[shader("closesthit")]
void displacedTriangleMeshClosestHit(inout RayData rayData, DisplacedTriangleMeshIntersector::Attribs attribs)
{
    // Store hit information. Note we don't access the materials here.
    DisplacedTriangleHit displacedTriangleHit;
    displacedTriangleHit.instanceID = getGeometryInstanceID();
    displacedTriangleHit.primitiveIndex = PrimitiveIndex();
    displacedTriangleHit.barycentrics = attribs.barycentrics;
    displacedTriangleHit.displacement = attribs.displacement;

    // Write hit info to output buffer.
    uint2 launchIndex = DispatchRaysIndex().xy;
    handleHit(HitInfo(displacedTriangleHit), rayData);
}

//
// Shader entry points for Curve hit groups.
//

[shader("intersection")]
void curveIntersection()
{
    const Ray ray = Ray(WorldRayOrigin(), WorldRayDirection(), RayTMin(), RayTCurrent());
    CurveIntersector::Attribs attribs;
    float t;
    if (CurveIntersector::intersect(ray, getGeometryInstanceID(), PrimitiveIndex(), attribs, t))
    {
        ReportHit(t, 0, attribs);
    }
}

[shader("closesthit")]
void curveClosestHit(inout RayData rayData, CurveIntersector::Attribs attribs)
{
    // Store hit information. Note we don't access the materials here.
    CurveHit curveHit;
    curveHit.instanceID = getGeometryInstanceID();
    curveHit.primitiveIndex = PrimitiveIndex();
    curveHit.barycentrics = attribs.barycentrics;

    // Write hit info to output buffer.
    uint2 launchIndex = DispatchRaysIndex().xy;
    handleHit(HitInfo(curveHit), rayData);
}

//
// Shader entry points for SDFGrid hit groups.
//

[shader("intersection")]
void sdfGridIntersection()
{
    const Ray ray = Ray(WorldRayOrigin(), WorldRayDirection(), RayTMin(), RayTCurrent());
    SDFGridHitData sdfGridHitData;
    float t;
    if (SDFGridIntersector::intersect(ray, getGeometryInstanceID(), PrimitiveIndex(), sdfGridHitData, t))
    {
        ReportHit(t, 0, sdfGridHitData);
    }
}

[shader("closesthit")]
void sdfGridClosestHit(inout RayData rayData, SDFGridHitData sdfGridHitData)
{
    // Store hit information. Note we don't access the materials here.
    SDFGridHit sdfGridHit;
    sdfGridHit.instanceID = getGeometryInstanceID();
    sdfGridHit.hitData = sdfGridHitData;

    // Write hit info to output buffer.
    uint2 launchIndex = DispatchRaysIndex().xy;
    handleHit(HitInfo(sdfGridHit), rayData);
}

//
// Shader entry point for ray generation shader.
//

[shader("raygeneration")]
void rayGen()
{
    uint2 launchIndex = DispatchRaysIndex().xy;

    GpuTimer timer;
    gVBufferSC.beginTime(timer);

    // Trace ray
    // RayData rayData;
    RayData rayData = RayData(SampleGenerator(launchIndex, gVBufferSC.frameCount));
    const Ray camRay = gVBufferSC.generateRay(launchIndex, rayData.sg);
    Ray ray = camRay;
    rayData.origin = ray.origin;
    rayData.direction = ray.dir;
    RayDesc raydesc = ray.toRayDesc();
    raydesc.TMax = camRay.tMax;
    raydesc.TMin = camRay.tMin;
    for (uint i = 0; i < gMaxRecursion; i++)
    // for (uint i = 0; i < 1; i++)
    {
        TraceRay(
            gScene.rtAccel,
            VBufferSC::kRayFlags,
            0xff /* instanceInclusionMask */,
            0 /* hitIdx */,
            rayTypeCount,
            0 /* missIdx */,
            raydesc,
            rayData
        );
        if (rayData.terminated)
            break;
        // update ray for next iteration
        raydesc.Direction = rayData.direction;
        raydesc.Origin = rayData.origin;
        if (i == 0)
        {
            raydesc.TMin = 0.0; // Change tMin for transparent hits
        }
    }

    gVBufferSC.endTime(launchIndex, timer);
}
