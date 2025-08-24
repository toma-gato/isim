#pragma once

#include <vector>

#include "Triangle.hh"
#include "Mat4x4.hh"

class Object3D {
public:

    Object3D();
    void addTriangle(const Triangle& triangle);

    std::vector<Triangle> triangles;
    Mat4x4 modelMatrix;
};
