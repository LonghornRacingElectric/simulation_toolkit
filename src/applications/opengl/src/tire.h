#ifndef TIRE_H
#define TIRE_H

#include <iostream>
#include <cmath>
#define PI 3.14159265358979323846

// Function to generate tire (annular cylinder) vertices and indices
// innerDiameter: inner radius of the tire
// outerDiameter: outer radius of the tire
// width: height/thickness of the tire
// segments: number of segments around the circle
void generateTire(float innerDiameter, float outerDiameter, float width,
                  unsigned int segments,
                  float*& vertices, unsigned int*& indices,
                  unsigned int& vertexCount, unsigned int& indexCount) {
    
    float innerRadius = innerDiameter / 2.0f;
    float outerRadius = outerDiameter / 2.0f;
    float halfWidth = width / 2.0f;
    
    // 4 vertices per segment (inner top, inner bottom, outer top, outer bottom)
    vertexCount = (segments + 1) * 4;
    
    // 4 quads per segment (inner wall, outer wall, top ring, bottom ring) = 8 triangles = 24 indices
    indexCount = segments * 24;
    
    vertices = new float[vertexCount * 3];
    indices = new unsigned int[indexCount];
    
    unsigned int vIndex = 0;
    
    // Generate vertices
    for (unsigned int i = 0; i <= segments; i++) {
        float angle = (float)i * 2.0f * PI / (float)segments;
        float cosA = cos(angle);
        float sinA = sin(angle);
        
        // Inner top
        vertices[vIndex++] = innerRadius * cosA;
        vertices[vIndex++] = halfWidth;
        vertices[vIndex++] = innerRadius * sinA;
        
        // Inner bottom
        vertices[vIndex++] = innerRadius * cosA;
        vertices[vIndex++] = -halfWidth;
        vertices[vIndex++] = innerRadius * sinA;
        
        // Outer top
        vertices[vIndex++] = outerRadius * cosA;
        vertices[vIndex++] = halfWidth;
        vertices[vIndex++] = outerRadius * sinA;
        
        // Outer bottom
        vertices[vIndex++] = outerRadius * cosA;
        vertices[vIndex++] = -halfWidth;
        vertices[vIndex++] = outerRadius * sinA;
    }
    
    // Generate indices
    unsigned int iIndex = 0;
    for (unsigned int i = 0; i < segments; i++) {
        unsigned int base = i * 4;
        unsigned int next = (i + 1) * 4;
        
        // Inner wall (2 triangles)
        indices[iIndex++] = base + 0;      // inner top
        indices[iIndex++] = base + 1;      // inner bottom
        indices[iIndex++] = next + 0;      // next inner top
        
        indices[iIndex++] = base + 1;      // inner bottom
        indices[iIndex++] = next + 1;      // next inner bottom
        indices[iIndex++] = next + 0;      // next inner top
        
        // Outer wall (2 triangles)
        indices[iIndex++] = base + 2;      // outer top
        indices[iIndex++] = next + 2;      // next outer top
        indices[iIndex++] = base + 3;      // outer bottom
        
        indices[iIndex++] = base + 3;      // outer bottom
        indices[iIndex++] = next + 2;      // next outer top
        indices[iIndex++] = next + 3;      // next outer bottom
        
        // Top ring (2 triangles)
        indices[iIndex++] = base + 0;      // inner top
        indices[iIndex++] = next + 0;      // next inner top
        indices[iIndex++] = base + 2;      // outer top
        
        indices[iIndex++] = next + 0;      // next inner top
        indices[iIndex++] = next + 2;      // next outer top
        indices[iIndex++] = base + 2;      // outer top
        
        // Bottom ring (2 triangles)
        indices[iIndex++] = base + 1;      // inner bottom
        indices[iIndex++] = base + 3;      // outer bottom
        indices[iIndex++] = next + 1;      // next inner bottom
        
        indices[iIndex++] = next + 1;      // next inner bottom
        indices[iIndex++] = base + 3;      // outer bottom
        indices[iIndex++] = next + 3;      // next outer bottom
    }
}

#endif // TIRE_H