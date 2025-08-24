#include "Object3D.hh"

Object3D::Object3D() {
    modelMatrix = Mat4x4::identity();
}

void Object3D::addTriangle(const Triangle& triangle) {
    triangles.emplace_back(triangle);
}
