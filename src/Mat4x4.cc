#include "Mat4x4.hh"

Mat4x4::Mat4x4() : mat{} {};

Mat4x4::Mat4x4(const float values) {
    for (int i = 0; i < 16; ++i) {
        mat[i] = values;
    }
}

Mat4x4 Mat4x4::identity() {
    Mat4x4 result;
    result(0, 0) = 1.0f;
    result(1, 1) = 1.0f;
    result(2, 2) = 1.0f;
    result(3, 3) = 1.0f;
    return result;
}

Mat4x4 Mat4x4::translation(float tx, float ty, float tz) {
    Mat4x4 result = identity();
    result(0, 3) = tx;
    result(1, 3) = ty;
    result(2, 3) = tz;
    return result;
}

Mat4x4 Mat4x4::rotationX(float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);

    Mat4x4 result;

    result.mat = {
        1, 0, 0, 0,
        0, c, -s, 0,
        0, s, c, 0,
        0, 0, 0, 1
    };

    return result;
}

Mat4x4 Mat4x4::rotationY(float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);

    Mat4x4 result;

    result.mat = {
        c, 0, s, 0,
        0, 1, 0, 0,
        -s, 0, c, 0,
        0, 0, 0, 1
    };

    return result;
}

Mat4x4 Mat4x4::rotationZ(float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);

    Mat4x4 result;

    result.mat = {
        c, -s, 0, 0,
        s, c, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };

    return result;
}

Mat4x4 Mat4x4::perspective(float fov, float aspect, float near, float far) {
    Mat4x4 result = {};
    float fovRad = fov * (M_PI / 180.0f);
    float f = 1.0f / std::tan(fovRad / 2.0f);
    result(0, 0) = f / aspect;
    result(1, 1) = f;
    result(2, 2) = (far + near) / (near - far);
    result(2, 3) = (2 * far * near) / (near - far);
    result(3, 2) = -1.0f;
    return result;
}

float& Mat4x4::operator()(int i, int j) {
    return mat[i * 4 + j];
}

const float& Mat4x4::operator()(int i, int j) const {
    return mat[i * 4 + j];
}

Mat4x4 Mat4x4::operator*(const Mat4x4& other) const {
    Mat4x4 result;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                result(i, j) += (*this)(i, k) * other(k, j);
            }
        }
    }
    return result;
}

Vector4 Mat4x4::operator*(const Vector4& v) const {
    return {
        cleanFloat(v.x * mat[0] + v.y * mat[1] + v.z * mat[2] + v.w * mat[3]),
        cleanFloat(v.x * mat[4] + v.y * mat[5] + v.z * mat[6] + v.w * mat[7]),
        cleanFloat(v.x * mat[8] + v.y * mat[9] + v.z * mat[10] + v.w * mat[11]),
        cleanFloat(v.x * mat[12] + v.y * mat[13] + v.z * mat[14] + v.w * mat[15])
    };
}