#pragma once

#include "Vector4.hh"

#include <cmath>

class Vector4;

class Vector3 
{
public:

    Vector3();
    Vector3(float x, float y, float z);

    ~Vector3();

    float x;
    float y;
    float z;

    float length() const;

    Vector3 normalize() const;

    Vector3 operator+(const Vector3& other) const;
    Vector3 operator-(const Vector3& other) const;
    Vector3 operator*(float scalar) const;
    Vector3 operator/(float scalar) const;

    float dot(const Vector3& other) const;

    Vector3 cross(const Vector3& other) const;

    Vector3 Vec4toVec3(const Vector4& v);
};