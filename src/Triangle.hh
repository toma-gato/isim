#pragma once

#include "Vertex.hh"

class Triangle {
public:
    Triangle();
    Triangle(const Vertex& v0, const Vertex& v1, const Vertex& v2);

    ~Triangle();

    Vertex v0;
    Vertex v1;
    Vertex v2;
};
