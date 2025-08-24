#pragma once

#include <cstdint>
#include <utility>
#include <vector>
#include <immintrin.h>
#include <mmintrin.h>
#include <xmmintrin.h>
#include <cstdint>
#include <vector>
#include <xmmintrin.h>

#include "Camera.hh"
#include "Mat4x4.hh"
#include "Object3D.hh"
#include "BS_thread_pool.hh"
#include "Camera.hh"
#include "Mat4x4.hh"
#include "Object3D.hh"
#include "Vector4.hh"

const std::array<std::pair<float, bool>, 4> boundaries = {
    std::make_pair(-1.0f, true),   // Left (X)
    std::make_pair(1.0f, true),    // Right (X)
    std::make_pair(-1.0f, false),  // Bottom (Y)
    std::make_pair(1.0f, false)    // Top (Y)
};

class Pipeline
{
public:
    Pipeline();
    Pipeline(Camera &cam, std::vector<Object3D> &SceneObjects, std::vector<uint8_t> &frameBuffer, int fHeight, int fWidth);
    Pipeline(Camera &cam, std::vector<Object3D> &SceneObjects, std::vector<uint8_t> &frameBuffer, std::vector<float> &depthBuffer, int fHeight, int fWidth);

    void ObjectClipSpace(Object3D &localObject);
    void ObjectNDCCliping(Object3D &localObject);

    void ObjectClipSpaceThreaded(Object3D &localObject);

    // Cohen-Sutherland implem -> See Liang-Barksy algo seems to be faster
    // inline fun is already fast, will see
    inline bool isOutsideNearFar(const Vector4& clipPos);


    bool shouldClipTriangle(const Vector4& v0, const Vector4& v1, const Vector4& v2);
    // Clip a single edge against a plane (near or far)
    inline Vector4 clipEdge(const Vector4& v1, const Vector4& v2, float plane, bool isNear);
    // Clip triangle against near/far planes
    std::vector<Vector4> clipTriangle(const Vector4& v0, const Vector4& v1, const Vector4& v2);

    bool isInsideNDC(const Vector3& ndc);
    std::vector<Vector3> clipTriangleXY(const Vector3& v0, const Vector3& v1, const Vector3& v2);

    // member function of the naive rasterization implementation
    void rasterizeObject(Object3D &obj);

    // naive rasterization
    void rasterizeScene();

    // first implementation of the rasterization implementation that is threaded 
    // now have the clip space threaded function to be faster
    // is using the SIMD triangle drawing
    void rasterizeSceneThreaded();

    // best implementation so far, use the thread pool the most
    // split the work in batches and called modified function that haves mutexes to be use with threads
    void rasterizeSceneThreadedFull();

    // Basic triangle rasterization using barycentric coordinates
    void drawTriangle(Vector3& v0, Vector3& v1, Vector3& v2, const Vector4& c0, const Vector4& c1, const Vector4& c2);


    // SIMD translated function are only translation from regular function to SIMD ones

    // first use of the SIMD function and operation to be faster, faster when couple with -O2 optimisation in addition
    void drawTriangleSIMD(Vector3& v0, Vector3& v1, Vector3& v2, const Vector4& c0, const Vector4& c1, const Vector4& c2);

    // Best drawing function, it can be used in the thread pool batch implementation of the rasterizeSceneThreadedFull method
    void drawTriangleSIMDOptimized(Vector3& v0, Vector3& v1, Vector3& v2, const Vector4& c0, const Vector4& c1, const Vector4& c2);

    // first and naive edge function calculation
    inline float edgeFunction(const Vector3& a, const Vector3& b, const Vector3& c);

    // SIMD implementation of the edge function, it calculate 4 pixels at once
    inline __m128 edgeFunctionSIMD(__m128 ax, __m128 ay,
                                   __m128 bx, __m128 by,
                                   __m128 cx, __m128 cy);

    // Naive setpixel function that apply the color at the x y coordinate in the frameBuffer
    void setPixel(uint32_t x, uint32_t y, const Vector4& color);

    std::vector<Object3D> SceneObjects;
    std::vector<uint8_t> frameBuffer;
    std::vector<float> depthBuffer;
    int fHeight;
    int fWidth;
    float ffHeight;
    float ffWidth;
    Camera Cam;
    uint64_t counterCalled;
    BS::thread_pool<> pool;
};
