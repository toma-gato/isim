#pragma once

#include <array>
#include <cmath>

#include "Vector3.hh"
#include "Vector4.hh"
#include "Utils.hh"

class Vector3;
class Vector4;

class Mat4x4 {
public:

    Mat4x4();
    Mat4x4(const float values);
    
    std::array<float, 16> mat;

    static Mat4x4 identity();
    Mat4x4 translation(float x, float y, float z);
    Mat4x4 rotationX(float angle);
    Mat4x4 rotationY(float angle);
    Mat4x4 rotationZ(float angle);
    static Mat4x4 perspective(float fov, float aspect, float near, float far);

    float& operator()(int i, int j);
    const float& operator()(int i, int j) const;

    Mat4x4 operator*(const Mat4x4& other) const;
    Vector4 operator*(const Vector4& v) const;
};