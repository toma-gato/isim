#pragma once

#include "Vector3.hh"
#include "Mat4x4.hh"
#include "Utils.hh"

#include <cmath>

class Vector3;
class Mat4x4;

class Vector4 {
public:

    Vector4();
    Vector4(float x, float y, float z, float w);
    Vector4(const Vector3& v, float w = 1.0f);

    ~Vector4();

    float x;
    float y;
    float z;
    float w;

    Vector4 operator+(const Vector4& other) const;
    Vector4 operator-(const Vector4& other) const;
    Vector4 operator*(float scalar) const;
    Vector4 operator/(float scalar) const;

    Vector4 operator*(const Mat4x4& mat) const;

    float& operator[](int i);
    const float& operator[](int i) const;

    Vector3 xyz();
};
