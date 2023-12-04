#pragma once
#include "../head.h"

void DDA_Circle(float X, float Y, float radius) {
    glColor3f(1.0f, 0.0f, 0.0f);
    glPointSize(2);

    float x, y;
    for (int i = 0; i <= 360; i++) {
        x = (X + radius * cos(i * Pi / 180));
        y = (Y + radius * sin(i * Pi / 180));//ÕýÔò»¯
        glBegin(GL_POINTS);
        glVertex2f(x, y);
        glEnd();
    }



    glFlush();
}
