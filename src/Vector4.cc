#include "Vector4.hh"
#include "Vector3.hh"

Vector4::Vector4() : x(0), y(0), z(0), w(0) {}
Vector4::Vector4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
Vector4::Vector4(const Vector3& v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
Vector4::~Vector4() {}

Vector4 Vector4::operator+(const Vector4& v) const {
    return {x + v.x, y + v.y, z + v.z, w + v.w};
}

Vector4 Vector4::operator-(const Vector4& v) const {
    return {x - v.x, y - v.y, z - v.z, w - v.w};
}

Vector4 Vector4::operator*(float s) const {
    return {x * s, y * s, z * s, w * s};
}

Vector4 Vector4::operator/(float s) const {
    return {x / s, y / s, z / s, w / s};
}

Vector4 Vector4::operator*(const Mat4x4& mat) const {
    return {
        cleanFloat(x * mat(0, 0) + y * mat(1, 0) + z * mat(2, 0) + w * mat(3, 0)),
        cleanFloat(x * mat(0, 1) + y * mat(1, 1) + z * mat(2, 1) + w * mat(3, 1)),
        cleanFloat(x * mat(0, 2) + y * mat(1, 2) + z * mat(2, 2) + w * mat(3, 2)),
        cleanFloat(x * mat(0, 3) + y * mat(1, 3) + z * mat(2, 3) + w * mat(3, 3))
    };
}

float& Vector4::operator[](int i) { 
    return *(&x + i);
}

const float& Vector4::operator[](int i) const {
    return *(&x + i);
}

Vector3 Vector4::xyz() {
    return Vector3(x, y, z);
}
