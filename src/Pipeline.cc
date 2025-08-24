#include "Pipeline.hh"

Pipeline::Pipeline()
: Cam(), SceneObjects(), frameBuffer(std::vector<uint8_t>(800*800*4)), fHeight(800), fWidth(800),depthBuffer(std::vector<float>(800*800, 255.0f)), counterCalled(0), pool(std::thread::hardware_concurrency), ffHeight(800.0f), ffWidth(800.0f) {}

Pipeline::Pipeline(Camera &cam, std::vector<Object3D> &SceneObjects, std::vector<uint8_t> &frameBuffer, int fHeight, int fWidth)
: Cam(cam), SceneObjects(SceneObjects), frameBuffer(frameBuffer), fHeight(fHeight), fWidth(fWidth), depthBuffer(std::vector<float>(fHeight*fWidth, 255.0f)), counterCalled(0), pool(std::thread::hardware_concurrency), ffHeight(static_cast<float>(fHeight)), ffWidth(static_cast<float>(fWidth)) {}

Pipeline::Pipeline(Camera &cam, std::vector<Object3D> &SceneObjects, std::vector<uint8_t> &frameBuffer, std::vector<float> &depthBuffer, int fHeight, int fWidth)
: Cam(cam), SceneObjects(SceneObjects), frameBuffer(frameBuffer), fHeight(fHeight), fWidth(fWidth), depthBuffer(depthBuffer), counterCalled(0), pool(std::thread::hardware_concurrency), ffHeight(static_cast<float>(fHeight)), ffWidth(static_cast<float>(fWidth)) {}

void Pipeline::ObjectClipSpace(Object3D &localObject)
{
    Mat4x4 MVP = Cam.getPositionMatrix(localObject.modelMatrix);
    std::vector<Triangle> clippedTriangles;
    clippedTriangles.reserve(localObject.triangles.size());


    for (const Triangle &tri : localObject.triangles)
    {
        Vector4 v0(tri.v0.position);
        Vector4 v1(tri.v1.position);
        Vector4 v2(tri.v2.position);
        v0.w = 1.0f;
        v1.w = 1.0f;
        v2.w = 1.0f;
        v0 = MVP * v0;
        v1 = MVP * v1;
        v2 = MVP * v2;

        std::vector<Vector4> clipped = clipTriangle(v0, v1, v2);
        if (!clipped.empty())
        {
            for (int i = 2; i < clipped.size(); i++)
            {
                // Converting to NDC SPACE when emplacing
                clippedTriangles.emplace_back(
                    Vertex(clipped[0].xyz() / clipped[0].w, tri.v0.normal, tri.v0.uvCoord, tri.v0.color),
                    Vertex(clipped[i - 1].xyz() / clipped[i - 1].w, tri.v1.normal, tri.v1.uvCoord, tri.v1.color),
                    Vertex(clipped[i].xyz() / clipped[i].w, tri.v2.normal, tri.v2.uvCoord, tri.v2.color)
                );
            }
        }
    }
    localObject.triangles = std::move(clippedTriangles);
}

void Pipeline::ObjectClipSpaceThreaded(Object3D &localObject) {
    constexpr size_t BATCH_SIZE = 512;
    Mat4x4 MVP = Cam.getPositionMatrix(localObject.modelMatrix);
    std::vector<Triangle> clippedTriangles;
    clippedTriangles.reserve(localObject.triangles.size());

    std::vector<std::future<std::vector<Triangle>>> futures;

    for (size_t start = 0; start < localObject.triangles.size(); start += BATCH_SIZE) {
        size_t end = std::min(start + BATCH_SIZE, localObject.triangles.size());

        futures.emplace_back(
            pool.submit_task([start, end, &localObject, &MVP, this]() {
                std::vector<Triangle> batchResult;
                batchResult.reserve((end - start) * 2);

                for (size_t i = start; i < end; i++) {
                    const Triangle& tri = localObject.triangles[i];
                    Vector4 v0(tri.v0.position, 1.0f);
                    Vector4 v1(tri.v1.position, 1.0f);
                    Vector4 v2(tri.v2.position, 1.0f);

                    v0 = MVP * v0;
                    v1 = MVP * v1;
                    v2 = MVP * v2;

                    std::vector<Vector4> clipped = clipTriangle(v0, v1, v2);

                    if (!clipped.empty()) {
                        for (int j = 2; j < clipped.size(); j++) {
                            batchResult.emplace_back(
                                Vertex(clipped[0].xyz() / clipped[0].w, tri.v0.normal, tri.v0.uvCoord, tri.v0.color),
                                Vertex(clipped[j-1].xyz() / clipped[j-1].w, tri.v1.normal, tri.v1.uvCoord, tri.v1.color),
                                Vertex(clipped[j].xyz() / clipped[j].w, tri.v2.normal, tri.v2.uvCoord, tri.v2.color)
                            );
                        }
                    }
                }
                return batchResult;
            })
        );
    }

    for (auto& future : futures) {
        auto triangles = future.get();
        clippedTriangles.insert(clippedTriangles.end(),
                                std::make_move_iterator(triangles.begin()),
                                std::make_move_iterator(triangles.end()));
    }

    localObject.triangles = std::move(clippedTriangles);
}

void Pipeline::ObjectNDCCliping(Object3D &localObject) {
    std::vector<Triangle> visibleTriangles;
    visibleTriangles.reserve(localObject.triangles.size());

    for (const Triangle &tri : localObject.triangles) {
        // Get NDC coordinates
        Vector3 ndc0 = tri.v0.position;
        Vector3 ndc1 = tri.v1.position;
        Vector3 ndc2 = tri.v2.position;

        if (isInsideNDC(ndc0) && isInsideNDC(ndc1) && isInsideNDC(ndc2)) {
            visibleTriangles.emplace_back(tri);
            continue;
        }

        // Clip against XY bounds
        std::vector<Vector3> clipped = clipTriangleXY(ndc0, ndc1, ndc2);
        if (!clipped.empty()) {
            for (size_t i = 2; i < clipped.size(); i++) {
                visibleTriangles.emplace_back(
                    Vertex(clipped[0], tri.v0.normal, tri.v0.uvCoord, tri.v0.color),
                    Vertex(clipped[i-1], tri.v1.normal, tri.v1.uvCoord, tri.v1.color),
                    Vertex(clipped[i], tri.v2.normal, tri.v2.uvCoord, tri.v2.color)
                );
            }
        }
    }

    localObject.triangles = std::move(visibleTriangles);
}

bool Pipeline::isInsideNDC(const Vector3& ndc) {
    return (ndc.x >= -1.0f) && (ndc.x <= 1.0f) && 
    (ndc.y >= -1.0f) && (ndc.y <= 1.0f);
}

std::vector<Vector3> Pipeline::clipTriangleXY(const Vector3& v0, const Vector3& v1, const Vector3& v2) {
    std::array<Vector3, 6> output = {v0, v1, v2};
    int count = 3;

    // Clip against each screen edge (left, right, bottom, top)
    for (const auto& [boundary, isXAxis] : boundaries) {
        std::array<Vector3, 6> input = output;
        int newCount = 0;
        for (int i = 0; i < count; i++) {
            const Vector3& current = input[i];
            const Vector3& prev = input[(i + count - 1) % count];

            // Check if inside current boundary
            bool currentInside = isXAxis ? 
                (current.x >= boundary) : (current.y >= boundary);
            bool prevInside = isXAxis ? 
                (prev.x >= boundary) : (prev.y >= boundary);

            if (currentInside != prevInside) {
                float t = isXAxis ? 
                    (boundary - prev.x) / (current.x - prev.x) :
                    (boundary - prev.y) / (current.y - prev.y);

                output[newCount++] = Vector3{
                    isXAxis ? boundary : prev.x + t * (current.x - prev.x),
                    isXAxis ? prev.y + t * (current.y - prev.y) : boundary,
                    prev.z + t * (current.z - prev.z)  // Interpolate Z
                };
            }
            if (currentInside) {
                output[newCount++] = current;
            }
        }
        count = newCount;
        if (count < 3) return {};
    }

    return std::vector<Vector3>(output.begin(), output.begin() + count);
}

inline bool Pipeline::isOutsideNearFar(const Vector4& clipPos) {
    const float z = clipPos.z;
    const float w = clipPos.w;
    return (z < -w) || (z > w);
}

bool Pipeline::shouldClipTriangle(const Vector4& v0, const Vector4& v1, const Vector4& v2) {
    return isOutsideNearFar(v0) || isOutsideNearFar(v1) || isOutsideNearFar(v2);
}

inline Vector4 Pipeline::clipEdge(const Vector4& v1, const Vector4& v2, float plane, bool isNear)
{
    float t;
    if (isNear) {
        t = (v1.z - (-v1.w)) / ((v1.z - (-v1.w)) - (v2.z - (-v2.w))); // Near: z = -w
    } else {
        t = (v1.z - v1.w) / ((v1.z - v1.w) - (v2.z - v2.w));          // Far: z = w
    }

    return Vector4{
        v1.x + t * (v2.x - v1.x),
        v1.y + t * (v2.y - v1.y),
        isNear ? -v1.w : v1.w,
        v1.w + t * (v2.w - v1.w)
    };
}

std::vector<Vector4> Pipeline::clipTriangle(const Vector4& v0, const Vector4& v1, const Vector4& v2)
{
    std::array<Vector4, 6> output = {v0, v1, v2};
    int count = 3;

    // Clip against near plane (z = -w)
    {
        std::array<Vector4, 6> input = output;
        int newCount = 0;
        for (int i = 0; i < count; i++) {
            const Vector4& current = input[i];
            const Vector4& prev = input[(i + count - 1) % count];

            bool currentInside = (current.z >= -current.w);
            bool prevInside = (prev.z >= -prev.w);

            if (currentInside != prevInside) {
                output[newCount++] = clipEdge(prev, current, -1.0f, true);
            }
            if (currentInside) {
                output[newCount++] = current;
            }
        }
        count = newCount;
        if (count < 3)
            return {};
    }

    // Clip against far plane (z = w)
    {
        std::array<Vector4, 6> input = output;
        int newCount = 0;
        for (int i = 0; i < count; i++) {
            const Vector4& current = input[i];
            const Vector4& prev = input[(i + count - 1) % count];

            bool currentInside = (current.z <= current.w);
            bool prevInside = (prev.z <= prev.w);

            if (currentInside != prevInside) {
                output[newCount++] = clipEdge(prev, current, 1.0f, false);
            }
            if (currentInside) {
                output[newCount++] = current;
            }
        }
        count = newCount;
        if (count < 3) return {};
    }

    return std::vector<Vector4>(output.begin(), output.begin() + count);
}

void Pipeline::rasterizeObject(Object3D &obj)
{
    // Rasterize each triangle
    for (const auto& tri : obj.triangles) {
        // Get NDC coordinates
        Vector3 v0 = tri.v0.position;
        Vector3 v1 = tri.v1.position;
        Vector3 v2 = tri.v2.position;

        // Convert from NDC to screen coordinates
        Vector3 screenV0 = {
            (v0.x + 1.0f) * 0.5f * fWidth,
            (1.0f - v0.y) * 0.5f * fHeight,  // Flip Y axis
            v0.z
        };
        Vector3 screenV1 = {
            (v1.x + 1.0f) * 0.5f * fWidth,
            (1.0f - v1.y) * 0.5f * fHeight,
            v1.z
        };
        Vector3 screenV2 = {
            (v2.x + 1.0f) * 0.5f * fWidth,
            (1.0f - v2.y) * 0.5f * fHeight,
            v2.z
        };

        drawTriangle(screenV0, screenV1, screenV2, tri.v0.color, tri.v1.color, tri.v2.color);
    }
}
void Pipeline::rasterizeSceneThreaded()
{
    std::vector<std::future<void>> futures;
    futures.reserve(SceneObjects.size());

    for (auto& obj : SceneObjects) {
        futures.emplace_back(
            pool.submit_task([&obj, this]() {
                ObjectClipSpaceThreaded(obj);
                ObjectNDCCliping(obj);
                for (const auto& tri : obj.triangles) {
                    Vector3 v0 = tri.v0.position;
                    Vector3 v1 = tri.v1.position;
                    Vector3 v2 = tri.v2.position;
                    Vector3 s0 = { (v0.x+1)*0.5f*fWidth,  (1.0f-v0.y)*0.5f*fHeight, v0.z };
                    Vector3 s1 = { (v1.x+1)*0.5f*fWidth,  (1.0f-v1.y)*0.5f*fHeight, v1.z };
                    Vector3 s2 = { (v2.x+1)*0.5f*fWidth,  (1.0f-v2.y)*0.5f*fHeight, v2.z };
                    drawTriangleSIMD(s0, s1, s2,
                                     tri.v0.color, tri.v1.color, tri.v2.color);
                }
            })
        );
    }

    for (auto& fut : futures) {
        fut.get();
    }
}

void Pipeline::rasterizeSceneThreadedFull()
{
    std::vector<std::future<void>> futures;

    // First process all objects through clip space and NDC clipping
    for (size_t objIdx = 0; objIdx < SceneObjects.size(); ++objIdx) {
        futures.emplace_back(
            pool.submit_task([objIdx, this]() {
                auto& obj = SceneObjects[objIdx];
                ObjectClipSpaceThreaded(obj);
                ObjectNDCCliping(obj);
            })
        );
    }

    // Wait for preprocessing to complete
    for (auto& future : futures) {
        future.get();
    }
    futures.clear();

    // Now process triangles in parallel batches
    for (size_t objIdx = 0; objIdx < SceneObjects.size(); ++objIdx) {
        auto& obj = SceneObjects[objIdx];
        const size_t numTriangles = obj.triangles.size();
        constexpr size_t BATCH_SIZE = 16;

        for (size_t start = 0; start < numTriangles; start += BATCH_SIZE) {
            size_t end = std::min(start + BATCH_SIZE, numTriangles);

            futures.emplace_back(
                pool.submit_task([objIdx, start, end, this]() {
                    Object3D& obj = SceneObjects[objIdx];
                    for (size_t i = start; i < end; ++i) {
                        const Triangle& tri = obj.triangles[i];
                        Vector3 v0 = tri.v0.position;
                        Vector3 v1 = tri.v1.position;
                        Vector3 v2 = tri.v2.position;

                        // Convert from NDC to screen coordinates
                        Vector3 s0 = { (v0.x+1)*0.5f*fWidth,  (1.0f-v0.y)*0.5f*fHeight, v0.z };
                        Vector3 s1 = { (v1.x+1)*0.5f*fWidth,  (1.0f-v1.y)*0.5f*fHeight, v1.z };
                        Vector3 s2 = { (v2.x+1)*0.5f*fWidth,  (1.0f-v2.y)*0.5f*fHeight, v2.z };

                        drawTriangleSIMD(s0, s1, s2,
                                         tri.v0.color, tri.v1.color, tri.v2.color);
                    }
                })
            );
        }
    }

    // Wait for all rasterization to complete
    for (auto& future : futures) {
        future.get();
    }
}

void Pipeline::rasterizeScene()
{
    for (auto& obj : SceneObjects)
    {
        ObjectClipSpaceThreaded(obj);
        ObjectNDCCliping(obj);
        rasterizeObject(obj);
    }
}

void Pipeline::drawTriangle(Vector3& v0, Vector3& v1, Vector3& v2, const Vector4& c0, const Vector4& c1, const Vector4& c2) {
    float min_x = std::fmin(std::fmin(v0.x, v1.x), v2.x);
    float max_x = std::fmax(std::fmax(v0.x, v1.x), v2.x);
    float min_y = std::fmin(std::fmin(v0.y, v1.y), v2.y);
    float max_y = std::fmax(std::fmax(v0.y, v1.y), v2.y);

    uint32_t minX = static_cast<uint32_t>(std::max(0.0f, min_x));
    uint32_t maxX = static_cast<uint32_t>(std::min(ffWidth, std::max(0.0f, max_x)));
    uint32_t minY = static_cast<uint32_t>(std::max(0.0f, min_y));
    uint32_t maxY = static_cast<uint32_t>(std::min(ffHeight, std::max(0.0f, max_y)));

    float area = edgeFunction(v0, v1, v2);

    // Very important line, 7 fps won
    if (area == 0)
        return;
    if (area < 0)
    {
        std::swap(v1, v2);
        area = -area;
    }

    size_t areacounter = 0;
    for (uint32_t y = minY; y <= maxY; ++y)
    {
        for (uint32_t x = minX; x <= maxX; ++x)
        {
            Vector3 pixelCenter(static_cast<float>(x) + 0.5f, static_cast<float>(y) + 0.5f, 0);

            // Calculate barycentric coordinates
            float w0 = edgeFunction(v1, v2, pixelCenter);
            float w1 = edgeFunction(v2, v0, pixelCenter);
            float w2 = edgeFunction(v0, v1, pixelCenter);

            // If pixel is inside the triangle or on its edge
            if (w0 >= 0 && w1 >= 0 && w2 >= 0)
            {
                // Normalize weights
                w0 /= area;
                w1 /= area;
                w2 /= area;

                // Calculate depth (Z-buffer)
                float z = v0.z * w0 + v1.z * w1 + v2.z * w2;

                // Check depth buffer
                size_t depthIndex = y * fWidth + x;
                areacounter++;
                if (z < depthBuffer[depthIndex])
                {
                    depthBuffer[depthIndex] = z;
                    Vector4 color = c0 * w0 + c1 * w1 + c2 * w2;
                    setPixel(x, y, color);
                }
            }
        }
    }
}

void Pipeline::drawTriangleSIMD(Vector3& v0, Vector3& v1, Vector3& v2, const Vector4& c0, const Vector4& c1, const Vector4& c2) {
    // Find bounding box of the triangle    
    float min_x = std::fmin(std::fmin(v0.x, v1.x), v2.x);
    float max_x = std::fmax(std::fmax(v0.x, v1.x), v2.x);
    float min_y = std::fmin(std::fmin(v0.y, v1.y), v2.y);
    float max_y = std::fmax(std::fmax(v0.y, v1.y), v2.y);

    uint32_t minX = static_cast<uint32_t>(std::max(0.0f, min_x));
    uint32_t maxX = static_cast<uint32_t>(std::min(ffWidth, std::max(0.0f, max_x)));
    uint32_t minY = static_cast<uint32_t>(std::max(0.0f, min_y));
    uint32_t maxY = static_cast<uint32_t>(std::min(ffHeight, std::max(0.0f, max_y)));

    float area = edgeFunction(v0, v1, v2);

    // Handle back-facing triangles
    if (area == 0)
        return;
    if (area < 0) {
        std::swap(v1, v2);
        area = -area;
    }

    // Convert triangle vertices to SIMD registers
    __m128 v0x = _mm_set1_ps(v0.x);
    __m128 v0y = _mm_set1_ps(v0.y);
    __m128 v0z = _mm_set1_ps(v0.z);
    __m128 v1x = _mm_set1_ps(v1.x);
    __m128 v1y = _mm_set1_ps(v1.y);
    __m128 v1z = _mm_set1_ps(v1.z);
    __m128 v2x = _mm_set1_ps(v2.x);
    __m128 v2y = _mm_set1_ps(v2.y);
    __m128 v2z = _mm_set1_ps(v2.z);    

    __m128 invArea = _mm_set1_ps(1.0f / area);

    // Iterate through each pixel in the bounding box
    for (uint32_t y = minY; y <= maxY; ++y) {
        __m128 py = _mm_set1_ps(static_cast<float>(y) + 0.5f);

        for (uint32_t x = minX; x <= maxX; x += 4) {
            // Prepare 4 x-coordinates and handle remainder at end
            uint32_t remaining = std::min(4u, maxX - x + 1);
            float px_vals[4] = {
                static_cast<float>(x) + 0.5f,
                static_cast<float>(x+1) + 0.5f,
                static_cast<float>(x+2) + 0.5f,
                static_cast<float>(x+3) + 0.5f
            };
            __m128 px = _mm_loadu_ps(px_vals);

            // Calculate barycentric coordinates for 4 pixels at once
            __m128 w0 = edgeFunctionSIMD(v1x, v1y, v2x, v2y, px, py);
            __m128 w1 = edgeFunctionSIMD(v2x, v2y, v0x, v0y, px, py);
            __m128 w2 = edgeFunctionSIMD(v0x, v0y, v1x, v1y, px, py);

            // Create mask for pixels inside triangle (>= 0 for all weights)
            // recreating (w0 >= 0 && w1 >= 0 && w2 > 0)
            __m128 insideMask = 
                _mm_and_ps(_mm_cmpge_ps(w0, _mm_setzero_ps()),
                           _mm_and_ps(_mm_cmpge_ps(w1, _mm_setzero_ps()),
                                      _mm_cmpge_ps(w2, _mm_setzero_ps())));

            // Normalize weights
            w0 = _mm_mul_ps(w0, invArea);
            w1 = _mm_mul_ps(w1, invArea);
            w2 = _mm_mul_ps(w2, invArea);

            // Calculate depth for 4 pixels
            __m128 z = _mm_add_ps(
                _mm_mul_ps(v0z, w0),
                _mm_add_ps(_mm_mul_ps(v1z, w1),
                           _mm_mul_ps(v2z, w2))
            );

            // Extract mask bits
            int mask = _mm_movemask_ps(insideMask);

            // Process each of the 4 pixels
            for (int i = 0; i < remaining; i++) {
                if (mask & (1 << i)) {
                    uint32_t currX = x + i;
                    size_t index = y * fWidth + currX;

                    // Extract values for this pixel
                    float z_array[4], w0_array[4], w1_array[4], w2_array[4];
                    _mm_storeu_ps(z_array, z);
                    _mm_storeu_ps(w0_array, w0);
                    _mm_storeu_ps(w1_array, w1);
                    _mm_storeu_ps(w2_array, w2);

                    float pixelZ = z_array[i];
                    if (pixelZ < depthBuffer[index]) {
                        depthBuffer[index] = pixelZ;

                        Vector4 color = c0 * w0_array[i] + 
                            c1 * w1_array[i] + 
                            c2 * w2_array[i];
                        setPixel(currX, y, color);
                    }
                }
            }
        }
    }
}

// Helper function to calculate edge function
inline float Pipeline::edgeFunction(const Vector3& a, const Vector3& b, const Vector3& c) {
    return (c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x);
}

inline __m128 Pipeline::edgeFunctionSIMD(__m128 ax, __m128 ay, 
                                         __m128 bx, __m128 by,
                                         __m128 cx, __m128 cy) {
    // (cx - ax) * (by - ay) - (cy - ay) * (bx - ax)
    __m128 term1 = _mm_mul_ps(_mm_sub_ps(cx, ax), _mm_sub_ps(by, ay));
    __m128 term2 = _mm_mul_ps(_mm_sub_ps(cy, ay), _mm_sub_ps(bx, ax));
    return _mm_sub_ps(term1, term2);
}

void Pipeline::setPixel(uint32_t x, uint32_t y, const Vector4& color) {
    if (x >= fWidth || y >= fHeight) return;

    size_t index = (y * fWidth + x) * 4;
    frameBuffer[index+1] = static_cast<uint8_t>(color.x * 255);
    frameBuffer[index+2] = static_cast<uint8_t>(color.y * 255);
    frameBuffer[index+3] = static_cast<uint8_t>(color.z * 255);
}

