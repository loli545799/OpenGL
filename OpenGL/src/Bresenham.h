#pragma once
#include "../head.h"

void Bresenham_Circle(float X, float Y, float radius) 
{
    int x = 0;
    int y = radius;
    int d = 3 - 2 * radius;
    glColor3f(0.0f, 1.0f, 0.0f);
    glPointSize(2);
    glBegin(GL_POINTS);

    while (x <= y) 
    {
        glVertex2f((X + x)/800, (Y + y)/600);
        glVertex2f((X - x)/800, (Y + y)/600);
        glVertex2f((X + x)/800, (Y - y)/600);
        glVertex2f((X - x)/800, (Y - y)/600);
        glVertex2f((X + y)/800, (Y + x)/600);
        glVertex2f((X - y)/800, (Y + x)/600);
        glVertex2f((X + y)/800, (Y - x)/600);
        glVertex2f((X - y)/800, (Y - x)/600);

        x++;
        if (d > 0) {
            y--;
            d = d + 4 * (x - y) + 10;
        }
        else 
            d = d + 4 * x + 6;
       
        if (x > y) 
            break;
    }

    glEnd();
}