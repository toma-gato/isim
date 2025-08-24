#include "Vector2.hh"

Vector2::Vector2()
    : x(0.0f), y(0.0f) {}
Vector2::Vector2(float x, float y)
    : x(x), y(y) {}

Vector2 Vector2::operator+(const Vector2 &other) const
{
    return Vector2(x + other.x, y + other.y);
}

Vector2 Vector2::operator-(const Vector2 &other) const
{
    return Vector2(x - other.x, y - other.y);
}

Vector2 Vector2::operator*(float scalar) const
{
    return Vector2(x * scalar, y * scalar);
}

Vector2 Vector2::operator/(float scalar) const
{
    return Vector2(x / scalar, y / scalar);
}

float Vector2::dot(const Vector2 &other) const
{
    return x * other.x + y * other.y;
}

float Vector2::length() const
{
    return std::sqrt(x * x + y * y);
}

Vector2 Vector2::normalize() const
{
    float len = length();
    if (len == 0)
        return Vector2(0, 0);
    return Vector2(x / len, y / len);
}

Vector2 Vector2::rotate(float angle) const
{
    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);
    return Vector2(x * cosAngle - y * sinAngle, x * sinAngle + y * cosAngle);
}
