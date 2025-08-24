#include "Vertex.hh"

Vertex::Vertex()
    : position(Vector3()), normal(Vector3()), uvCoord(Vector2()), color(Vector4()) {}
Vertex::Vertex(const Vector3& pos, const Vector3& normal, const Vector2& uvCoord, const Vector4& color)
    : position(pos), normal(normal), uvCoord(uvCoord), color(color) {}
Vertex::~Vertex() {}
