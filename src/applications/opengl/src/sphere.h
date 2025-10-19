#ifndef SPHERE_H
#define SPHERE_H

#include <iostream>
#include <cmath>
#define PI 3.14159265358979323846

// Function to generate sphere vertices and indices
void generateSphere(float radius, unsigned int stacks, unsigned int slices, float*& vertices, unsigned int*& indices, unsigned int& vertexCount, unsigned int& indexCount) {
    vertexCount = (stacks + 1) * (slices + 1);
    indexCount = 6 * stacks * slices;

    // Allocate memory for vertices and indices
    vertices = new float[vertexCount * 3];
    indices = new unsigned int[indexCount];

    unsigned int vIndex = 0;
    unsigned int iIndex = 0;

    // Generate vertices (latitude and longitude lines)
    for (unsigned int i = 0; i <= stacks; i++) {
        float theta = i * PI / stacks; // Latitude angle
        float sinTheta = sin(theta);
        float cosTheta = cos(theta);

        for (unsigned int j = 0; j <= slices; j++) {
            float phi = j * 2 * PI / slices; // Longitude angle
            float sinPhi = sin(phi);
            float cosPhi = cos(phi);

            float x = radius * cosPhi * sinTheta;
            float y = radius * cosTheta;
            float z = radius * sinPhi * sinTheta;

            // Add vertex to the array
            vertices[vIndex++] = x;
            vertices[vIndex++] = y;
            vertices[vIndex++] = z;
        }
    }

    // Generate indices to form triangles
    for (unsigned int i = 0; i < stacks; i++) {
        for (unsigned int j = 0; j < slices; j++) {
            unsigned int first = (i * (slices + 1)) + j;
            unsigned int second = first + slices + 1;

            // First triangle
            indices[iIndex++] = first;
            indices[iIndex++] = second;
            indices[iIndex++] = first + 1;

            // Second triangle
            indices[iIndex++] = second;
            indices[iIndex++] = second + 1;
            indices[iIndex++] = first + 1;
        }
    }
};

#endif // SPHERE_H