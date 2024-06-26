#include "Scene/SceneDefines.slangh"
#include "Utils/Math/MathConstants.slangh"

import Scene.Raytracing;
import Utils.Math.MathHelpers;
import Utils.Sampling.SampleGenerator;
import Rendering.Materials.StandardMaterial;
import Scene.Material.ShadingUtils;
import Rendering.Lights.LightHelpers;
import Utils.Color.ColorHelpers;

import PMHashFunction;

cbuffer PerFrame
{
    uint gFrameCount;              // Frame count since scene was loaded.
    float gCausticRadius;          // Radius for the caustic photons
    float gGlobalRadius;           // Radius for the global photons
    float gCausticHashScaleFactor; // Hash scale factor for caustic hash cells
    float gGlobalHashScaleFactor;
}

cbuffer CB
{
    uint gPRNGDimension;    // First available PRNG dimension.
    float gGlobalRejection; // Probabilty that an global photon is saved
    float gEmissiveScale;   // Scale for emissive ligth sources
    float gSpecRoughCutoff; // Cutoff for specular reflections

    uint gMaxRecursion;         // Max Iterations per path
    bool gUseAlphaTest;         // Enable Alpha Test
    bool gAdjustShadingNormals; // Adjust shading Normals
    uint gQuadProbeIt;          // Max number of quadratic probe iterations
};

// Inputs
Texture2D<int> gLightSample;
StructuredBuffer<uint> gNumPhotonsPerEmissive;
// Internal Buffer Structs

struct PhotonBucket
{
    uint size;
    int cell;
    uint2 pad;
    uint photonIdx[NUM_PHOTONS_PER_BUCKET];
};

struct PhotonInfo
{
    float3 dir;
    float faceNTheta;
    float3 flux;
    float faceNPhi;
};

// Internal Buffer Structs
RWStructuredBuffer<PhotonBucket> gGlobalHashBucket;
RWStructuredBuffer<PhotonBucket> gCausticHashBucket;

RWTexture2D<float4> gCausticPos;
RWTexture2D<float4> gCausticFlux;
RWTexture2D<float4> gCausticDir;
RWTexture2D<float4> gGlobalPos;
RWTexture2D<float4> gGlobalFlux;
RWTexture2D<float4> gGlobalDir;

Texture2D<uint> gRndSeedBuffer;

struct PhotonCounter
{
    uint caustic;
    uint global;
};
RWStructuredBuffer<PhotonCounter> gPhotonCounter;

// Static configuration based on defines set from the host
static const bool kUseAnalyticLights = USE_ANALYTIC_LIGHTS;
static const bool kUseEmissiveLights = USE_EMISSIVE_LIGHTS;
static const bool kUseEnvLight = USE_ENV_LIGHT;
static const bool kUseEnvBackground = USE_ENV_BACKGROUND;
static const float3 kDefaultBackgroundColor = float3(0, 0, 0);
static const float kRayTMax = FLT_MAX;
static const uint kMaxPhotonIndexGLB = MAX_PHOTON_INDEX_GLOBAL;
static const uint kMaxPhotonIndexCAU = MAX_PHOTON_INDEX_CAUSTIC;
static const float kAnalyticInvPdf = ANALYTIC_INV_PDF; // Inverse analytic pdf
static const uint kInfoTexHeight = INFO_TEXTURE_HEIGHT;
static const uint kNumBuckets = NUM_BUCKETS; // Total number of buckets in 2^x
static const bool kUsePhotonFaceNormal = PHOTON_FACE_NORMAL;
static const bool kUseMultiDiffuseHitCausticMap = MULTI_DIFFHIT_CAUSTIC_MAP;

static const float k_2Pi = 6.28318530717958647692;
static const float k_4Pi = 12.5663706143591729538;
static const float Inv2Pi = 0.15915494309189533577;
static const float Inv4Pi = 0.07957747154594766788;

/** Payload for scatter ray (64B).
 */
struct RayData
{
    float3 thp;             ///< Current path throughput. This is updated at each path vertex.
    uint encodedFaceNormal; ///< Face normal encoded in 16 bit polar coordinates
    float3 origin;          ///< Next path segment origin.
    bool terminated;        ///< Set to true when path is terminated.
    float3 direction;       ///< Next path segment direction.
    bool diffuseHit;        ///< saves if the his is diffuse

    SampleGenerator sg; ///< Per-ray state for the sample generator (up to 16B).

    /** Create ray payload with default parameters.
     */
    static RayData create()
    {
        RayData d;
        d.terminated = false;
        d.encodedFaceNormal = 0;
        d.thp = float3(1, 1, 1);
        d.origin = float3(0, 0, 0);
        d.direction = float3(0, 0, 0);
        return d;
    }
};

// cartesian to polar coodinate. Assumes p is normalized
float2 toSphericalCoordinate(float3 p)
{
    return float2(acos(p.y), atan2(p.z, p.x));
}

// transforms direction from local to world space
void fromLocalToWorld(in float3 lightDirW, inout float3 dir)
{
    // transform from local space -> random dir vector in world space
    float3 tangent, bitangent;
    if (abs(lightDirW.x) < 0.99)
        tangent = cross(lightDirW, float3(1., 0., 0.));
    else
        tangent = cross(lightDirW, float3(0., 1., 0.));
    // tangent = normalize(tangent);
    bitangent = cross(tangent, lightDirW);

    dir = dir.x * tangent + dir.y * bitangent + dir.z * lightDirW;
}

/** *Samples a random direction in a hemisphere or nornmal sphere
 * type for the different light types. 0 =Point ; 1 = Area ; 2 = Spot
 */
void calcLightDirection(in float3 lightDirW, float2 rnd, inout float3 newDir, inout float lightPDF, uint type, float cosThetaMax = M_PI)
{
    // theta and pdf for the different light types
    switch (type)
    {
    // Point
    case 0:
        newDir = sample_sphere(rnd);
        lightPDF = k_4Pi;
        break;
    // Area
    case 1:
        newDir = sample_cosine_hemisphere_polar(rnd, lightPDF);
        break;
    // Spot
    case 2:
        newDir = sample_cone(rnd, cosThetaMax);
        lightPDF = 1 / (k_2Pi * (1.0 - cosThetaMax));
    }

    fromLocalToWorld(lightDirW, newDir);
}

AABB calcPhotonAABB(in float3 center, in float radius)
{
    AABB aabb = AABB(center - radius, center + radius);
    return aabb;
}

[shader("miss")]
void miss(inout RayData rayData: SV_RayPayload)
{
    rayData.terminated = true;
}

[shader("closesthit")]
void closestHit(inout RayData rayData: SV_RayPayload, BuiltInTriangleIntersectionAttributes attribs: SV_IntersectionAttributes)
{
    const float3 incomingRayDir = -WorldRayDirection();
    // Evaluate Falcor's material parameters at the hit point.
    GeometryInstanceID instanceID = getGeometryInstanceID();
    VertexData v = getVertexData(instanceID, PrimitiveIndex(), attribs);
    const uint materialID = gScene.getMaterialID(instanceID);
    let lod = ExplicitLodTextureSampler(0.f);
    ShadingData sd = gScene.materials.prepareShadingData(v, materialID, incomingRayDir, lod);

    // Adjust shading normals if desired
    if (gAdjustShadingNormals)
    {
        adjustShadingNormal(sd, sd.frame);
    }

    BSDFSample bsdfSample;
    let mi = gScene.materials.getMaterialInstance(sd, lod);
    let bsdfProperties = mi.getProperties(sd);

    bool valid = mi.sample(sd, rayData.sg, bsdfSample, true /*ImportanceSampling*/);

    float3 thpStart = rayData.thp;

    bool transmission = bsdfSample.isLobe(LobeType::Transmission);
    float3 rayOrigin = sd.computeRayOrigin(!transmission); // false is for transmissions
    rayData.origin = rayOrigin;
    rayData.thp *= valid ? bsdfSample.weight : float3(0);
    rayData.direction = bsdfSample.wo;
    rayData.diffuseHit =
        bsdfSample.isLobe(LobeType::Diffuse) || (bsdfSample.isLobe(LobeType::Reflection) && (bsdfProperties.roughness > gSpecRoughCutoff));

    if (kUsePhotonFaceNormal)
    {
        // Flip face normal to point in photon hit direction
        float faceSignFactor = dot(incomingRayDir, sd.faceN) > 0 ? 1. : -1.;
        sd.faceN *= faceSignFactor;
        float2 sph = toSphericalCoordinate(sd.frame.N);
        // convert to two float16 and store in payload
        rayData.encodedFaceNormal = (f32tof16(sph.x) << 16) | f32tof16(sph.y);
    }

    // if throughput is 0, return
    if (!valid || any(rayData.thp < 0))
        rayData.terminated = true;
}

// Use anyhit for alpha test if activated
[shader("anyhit")]
void anyHit(inout RayData rayData: SV_RayPayload, BuiltInTriangleIntersectionAttributes attribs: SV_IntersectionAttributes)
{
    if (gUseAlphaTest)
    {
        // Alpha test for non-opaque geometry.
        GeometryInstanceID instanceID = getGeometryInstanceID();
        VertexData v = getVertexData(instanceID, PrimitiveIndex(), attribs);
        uint materialID = gScene.getMaterialID(instanceID);
        if (gScene.materials.alphaTest(v, materialID, 0.f))
            IgnoreHit();
    }
}

[shader("raygeneration")]
void rayGen()
{
    uint2 launchIndex = DispatchRaysIndex().xy;
    uint2 launchDim = DispatchRaysDimensions().xy;

    // random seed
    uint seed = gRndSeedBuffer[launchIndex];

    // Prepare ray payload.
    RayData rayData = RayData.create();
    rayData.sg = SampleGenerator(launchIndex, seed + gFrameCount);

    // Advance the generator to the first available dimension.
    // TODO: This is potentially expensive. We may want to store/restore the state from memory if it becomes a problem.
    for (uint i = 0; i < gPRNGDimension; i++)
        sampleNext1D(rayData.sg);

    LightCollection lc = gScene.lightCollection;

    // Get current light index and type. For emissive triangles only active ones where sampled
    int lightIndex = gLightSample[launchIndex];
    // 0 means invalid light index
    if (lightIndex == 0)
        return;
    bool analytic = lightIndex < 0; // Negative values are analytic lights
    if (analytic)
        lightIndex *= -1; // Swap sign if analytic
    lightIndex -= 1;      // Change index from 1->N to 0->(N-1)

    float invPdf = kAnalyticInvPdf; // Set to analytic pdf by default. If Emissive it is set later
    float3 lightPos = float3(0);
    float3 lightDir = float3(0, 1, 0);
    float3 lightIntensity = float3(0);
    float maxSpotAngle = 0;
    float penumbra = 0.0;
    uint type = 0; // 0 == Point, 1 == Area, 2 == Spot
    float lightArea = 1.f;

    if (analytic)
    {
        LightData currentLight = gScene.getLight(lightIndex);

        // we only support point lights
        if (currentLight.type != uint(LightType::Point))
            return;

        if (currentLight.openingAngle < (M_PI / 2.0))
            type = 2;

        lightPos = currentLight.posW;
        lightDir = currentLight.dirW;
        lightIntensity = currentLight.intensity;
        maxSpotAngle = currentLight.openingAngle;
        penumbra = currentLight.penumbraAngle;
    }
    // Emissive
    else
    {
        // invPdf = gEmissiveInvPdf[lightIndex]; //Assume all emissive triangles as pdf
        invPdf = 1.f / gNumPhotonsPerEmissive[lightIndex];
        const uint triIndex = lc.activeTriangles[lightIndex];
        EmissiveTriangle emiTri = lc.getTriangle(triIndex); // get the random triangle
        // random baycentric coordinates
        float2 RS = sampleNext2D(rayData.sg);
        const float3 barycentrics = sample_triangle(RS);
        lightPos = emiTri.getPosition(barycentrics);
        lightDir = emiTri.normal;
        lightArea = emiTri.area;

        // Evaluate Emitted radiance
        float2 uv = emiTri.getTexCoord(barycentrics);
        lightIntensity = gScene.materials.evalEmissive(emiTri.materialID, uv);
        lightIntensity *= gEmissiveScale;
        type = 1;
    }

    RayDesc ray;
    float lightDirPDF = 0.0;
    float3 lightRnd = sampleNext3D(rayData.sg);
    float spotAngle = maxSpotAngle - penumbra * lightRnd.z;
    calcLightDirection(lightDir, lightRnd.xy, ray.Direction, lightDirPDF, type, cos(spotAngle));

    // create photon
    float3 photonPos = float3(0, 0, 0);
    PhotonInfo photon;
    photon.dir = float3(0, 0, 0);
    photon.faceNTheta = 1.f;
    photon.flux = float3(0, 0, 0);
    photon.faceNPhi = 1.f;

    // light flux
    float3 lightFlux = lightIntensity * invPdf;
    if (!analytic)
        lightFlux *= abs(dot(lightDir, ray.Direction)) * lightArea;                       // Convert L to flux
    lightFlux /= analytic ? float(launchDim.x * launchDim.y) * lightDirPDF : lightDirPDF; // Total number of photons is in the invPDF for
                                                                                          // emissive
    // ray tracing vars
    ray.Origin = lightPos + 0.01 * ray.Direction;
    ray.TMin = 0.01f;
    ray.TMax = 1000.f;
    uint rayFlags = 0;

    bool wasReflectedSpecular = false;
    bool reflectedDiffuse = false;

    // recursion where the photons are shot
    for (uint i = 0; i < gMaxRecursion && !rayData.terminated; i++)
    {
        photon.flux = lightFlux * rayData.thp;
        TraceRay(gScene.rtAccel, rayFlags, 0xff /* instanceInclusionMask */, 0 /* hitIdx */, rayTypeCount, 0 /* missIdx */, ray, rayData);

        if (rayData.terminated)
            break;

        photonPos = rayData.origin;
        photon.dir = ray.Direction;
        reflectedDiffuse = rayData.diffuseHit;

        // save photon
        if (reflectedDiffuse)
        {
            // Get face normal and store
            if (kUsePhotonFaceNormal)
            {
                uint encTheta = (rayData.encodedFaceNormal >> 16) & 0xFFFF;
                uint encPhi = rayData.encodedFaceNormal & 0xFFFF;
                photon.faceNTheta = f16tof32(encTheta);
                photon.faceNPhi = f16tof32(encPhi);
            }

            uint photonIndex = 0;
            uint photonBucketIndex = 0;
            // rejection
            float rndRoulette = sampleNext1D(rayData.sg);
            bool roulette = rndRoulette <= gGlobalRejection;

            // hash scale
            float cellScale = wasReflectedSpecular ? gCausticHashScaleFactor : gGlobalHashScaleFactor;
            int3 cell = int3(floor(photonPos * cellScale));
            uint bucketIdx = hash(cell) & (kNumBuckets - 1);
            uint d = 0;

            int cellXY = (cell.x << 16) | (cell.y & 0xFFFF);
            if (cellXY == 0)
                cellXY = 0xFFFFFFFF; // avoid zero cell index
            // caustic photon
            if (wasReflectedSpecular)
            {
                // probe for free bucket
                bool probeSuccess = false;
                for (uint i = 0; i <= gQuadProbeIt; i++)
                {
                    int origValue;
                    InterlockedCompareExchange(gCausticHashBucket[bucketIdx].cell, 0, cellXY, origValue);
                    if (origValue == 0 || origValue == cellXY)
                    {
                        probeSuccess = true;
                        break;
                    }
                    ++d;
                    bucketIdx = (bucketIdx + ((d + d * d) >> 1)) & (kNumBuckets - 1); // quadradic probe
                }
                // insert caustic photon
                if (probeSuccess)
                {
                    InterlockedAdd(gCausticHashBucket[bucketIdx].size, 1u, photonBucketIndex);
                    // if bucket is full of photons replace a photon stochastically
                    if (photonBucketIndex >= NUM_PHOTONS_PER_BUCKET)
                    {
                        photonBucketIndex = min(sampleNext1D(rayData.sg) * photonBucketIndex + 1, photonBucketIndex);
                    }
                    if (photonBucketIndex < NUM_PHOTONS_PER_BUCKET)
                    {
                        InterlockedAdd(gPhotonCounter[0].caustic, 1u, photonIndex);
                        photonIndex = min(photonIndex, kMaxPhotonIndexCAU);
                        gCausticHashBucket[bucketIdx].photonIdx[photonBucketIndex] = photonIndex;
                        if (bucketIdx == 0)
                            gCausticHashBucket[bucketIdx].pad.x = photonIndex;
                        uint2 photonIndex2D = uint2(photonIndex / kInfoTexHeight, photonIndex % kInfoTexHeight);
                        gCausticPos[photonIndex2D] = float4(photonPos, cellXY);
                        gCausticFlux[photonIndex2D] = float4(photon.flux, photon.faceNTheta);
                        gCausticDir[photonIndex2D] = float4(photon.dir, photon.faceNPhi);
                    }
                }
            }
            // Global photon
            else if (roulette)
            {
                // probe for free bucket
                bool probeSuccess = false;
                for (uint i = 0; i <= gQuadProbeIt; i++)
                {
                    int origValue = 1;
                    InterlockedCompareExchange(gGlobalHashBucket[bucketIdx].cell, 0, cellXY, origValue);
                    if (origValue == 0 || origValue == cellXY)
                    {
                        probeSuccess = true;
                        break;
                    }
                    ++d;
                    bucketIdx = (bucketIdx + ((d + d * d) >> 1)) & (kNumBuckets - 1); // quadradic probe
                }
                // insert global photon
                if (probeSuccess)
                {
                    InterlockedAdd(gGlobalHashBucket[bucketIdx].size, 1u, photonBucketIndex);
                    // if bucket is full of photons replace a photon stochastically
                    if (photonBucketIndex >= NUM_PHOTONS_PER_BUCKET)
                    {
                        photonBucketIndex = min(sampleNext1D(rayData.sg) * photonBucketIndex + 1, photonBucketIndex);
                    }
                    if (photonBucketIndex < NUM_PHOTONS_PER_BUCKET)
                    {
                        photon.flux /= gGlobalRejection; //
                        InterlockedAdd(gPhotonCounter[0].global, 1u, photonIndex);
                        photonIndex = min(photonIndex, kMaxPhotonIndexGLB);
                        gGlobalHashBucket[bucketIdx].photonIdx[photonBucketIndex] = photonIndex;
                        if (bucketIdx == 0)
                            gGlobalHashBucket[bucketIdx].pad.x = photonIndex;
                        uint2 photonIndex2D = uint2(photonIndex / kInfoTexHeight, photonIndex % kInfoTexHeight);
                        gGlobalPos[photonIndex2D] = float4(photonPos, cellXY);
                        gGlobalFlux[photonIndex2D] = float4(photon.flux, photon.faceNTheta);
                        gGlobalDir[photonIndex2D] = float4(photon.dir, photon.faceNPhi);
                    }
                }
            }
        }

        // Russian Roulette
        const float rrVal = luminance(rayData.thp);
        const float prob = max(0.f, 1.f - rrVal);
        float rnd = sampleNext1D(rayData.sg);
        if (rnd < prob)
        {
            break; // Photon is absorbed
        }
        else
        {
            rayData.thp /= (1.f - prob);
        }

        // Definition for the caustic map. If MultiDiffHit is active all L(S|D)*SD paths are stored in the caustic map. Else only LS+D paths
        // are stored
        if (kUseMultiDiffuseHitCausticMap)
            wasReflectedSpecular = !reflectedDiffuse;
        else
        {
            // Are we on a LS+D Path ?
            if (i == 0)
                wasReflectedSpecular = !reflectedDiffuse;

            // Set to false on the first diffuse reflection
            wasReflectedSpecular = wasReflectedSpecular && !reflectedDiffuse;
        }

        // update ray for next iteration
        ray.Origin = rayData.origin;
        ray.Direction = rayData.direction;
    }
}
