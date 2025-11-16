#ifndef ROD_H
#define ROD_H

#include <iostream>
#include <cmath>
#define PI 3.14159265358979323846

// Function to generate rod (cylinder) vertices and indices
void generateRod(float radius, float height, unsigned int slices,
                 float*& vertices, unsigned int*& indices,
                 unsigned int& vertexCount, unsigned int& indexCount) {
    unsigned int rings = 2; // Top and bottom circle
    vertexCount = rings * (slices + 1); // two rings of (slices + 1) verts each (repeat first to close)
    indexCount = slices * 6; // two triangles per quad * slices

    vertices = new float[vertexCount * 3];
    indices = new unsigned int[indexCount];

    unsigned int vIndex = 0;
    float halfHeight = height / 2.0f;

    // Generate vertices for bottom and top rings
    for (unsigned int ring = 0; ring < rings; ++ring) {
        float y = (ring == 0) ? -halfHeight : halfHeight;
        for (unsigned int i = 0; i <= slices; ++i) {
            float theta = 2.0f * PI * i / slices;
            float x = radius * cos(theta);
            float z = radius * sin(theta);
            vertices[vIndex++] = x;
            vertices[vIndex++] = y;
            vertices[vIndex++] = z;
        }
    }

    // Generate indices for side quads (as two triangles)
    unsigned int iIndex = 0;
    for (unsigned int i = 0; i < slices; ++i) {
        unsigned int bottom = i;
        unsigned int top = i + (slices + 1);

        // First triangle
        indices[iIndex++] = bottom;
        indices[iIndex++] = top;
        indices[iIndex++] = bottom + 1;

        // Second triangle
        indices[iIndex++] = bottom + 1;
        indices[iIndex++] = top;
        indices[iIndex++] = top + 1;
    }
}

#endif // ROD_H
