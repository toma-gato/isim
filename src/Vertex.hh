#pragma once

#include "Vector2.hh"
#include "Vector3.hh"
#include "Vector4.hh"

class Vertex {
    public:
        Vertex();
        Vertex(const Vector3& pos, const Vector3& normal, const Vector2& uvCoord, const Vector4& color);

        ~Vertex();

        Vector3 position;
        Vector3 normal;
        Vector2 uvCoord; // Textures
        Vector4 color;
};
