#include "Triangle.hh"

Triangle::Triangle()
    : v0(Vertex()), v1(Vertex()), v2(Vertex()) {}
Triangle::Triangle(const Vertex& v0, const Vertex& v1, const Vertex& v2)
    : v0(v0), v1(v1), v2(v2) {}
Triangle::~Triangle() {}
