#pragma once
#include "../head.h"

int segments = 100; 
const int sectorCount = 36;
const int stackCount = 18;

std::vector<float> createsphere(float radius) {
    std::vector<float> vertices;
    float x, y, z, xy;
    float sectorStep = 2 * Pi / sectorCount;
    float stackStep = Pi / stackCount;
    float sectorAngle, stackAngle;

    for (int i = 0; i <= (int)(segments/2); ++i) {
        stackAngle = Pi / 2 - i * stackStep;
        xy = radius * cos(stackAngle);
        z = radius * sin(stackAngle);

        for (int j = 0; j <= segments; ++j) {
            sectorAngle = j * sectorStep;
            x = xy * cos(sectorAngle);
            y = xy * sin(sectorAngle);

            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
        }
    }
    return vertices;
}
