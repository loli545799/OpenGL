#pragma once
#include "../head.h"
#include "sphere.h"

float Star[9][4] = {
    // 0 speed_rotate  1 speed_revolve 2 radius_rotate 3 radius_revolve
    // sun, 
    {0.0f,0.0f,1.0f,0.0f},
    //mercury, 
    {1.0f,4.16f,0.04f,1.2f},
    //venus, 
    {1.0f,1.6f,0.09f,1.5f},
    //earth, 
    {1.0f,1.0f,0.05f,3.0f},
    //mars, 
    {1.0f,0.53f,0.025f,4.5f},
    //jupiter, 
    {1.0f,0.09f,0.5f,5.5f},
    //saturn, 
    {1.0f,0.03f,0.4f,7.0f},
    //uranus, 
    {1.0f,0.02f,0.175f,8.0f},
    //neptune
    {1.0f,0.01f,0.174f,9.5f}
};

glm::vec3 Color[9] = {
    // sun, mercury, venus, earth, mars, jupiter, saturn, uranus, neptune;
    glm::vec3(1.0f,0.0f,0.0f),
    glm::vec3(1.0f,0.84f,0.0f),
    glm::vec3(0.85f,0.84f,0.0f),
    glm::vec3(0.0f,0.0f,1.0f),
    glm::vec3(1.0f,0.0f,0.0f),
    glm::vec3(0.91f,0.55f,0.0f),
    glm::vec3(0.55f,0.27f,0.08f),
    glm::vec3(0.0f,0.75f,1.0f),
    glm::vec3(0.0f,0.0f,0.55f),

};

struct star {
    std::vector<float> vertics;  // 绘制坐标点信息
    float speed_rotate;          // 自转速度
    float speed_revolve;         // 公转速度
    float radius_rotate;         // 自转半径（球体半径）
    float radius_revolve;        // 公转半径
};

star S[9];// sun, mercury, venus, earth, mars, jupiter, saturn, uranus, neptune;

void set_radius_revolve(int loc, float r) {
    S[loc].radius_revolve = r;
};

void set_radius_rotate(int loc, float r) {
    S[loc].radius_rotate = r;
};

void set_ver(int loc) {
    S[loc].vertics = createsphere(S[loc].radius_rotate);
};

void set_speed_rotate(int loc, float s) {
    S[loc].speed_rotate = s;
};

void set_speed_revolve(int loc, float s) {
    S[loc].speed_revolve = s;
};

void init_star() { //0 speed_rotate  1 speed_revolve 2 radius_rotate 3 radius_revolve
    for (int i = 0; i < 9; i++) {
        set_speed_rotate(i, Star[i][0]);
        set_speed_revolve(i, Star[i][1]);
        set_radius_rotate(i, Star[i][2]);
        set_radius_revolve(i, Star[i][3]);
        set_ver(i);
    }
};

glm::mat4 set_model(star s) {
    glm::mat4 model;
    model = glm::translate(model, glm::vec3(s.radius_revolve * sin((float)glfwGetTime() * s.speed_revolve), s.radius_revolve * cos((float)glfwGetTime() * s.speed_revolve), 0.0f));        // revolve
    model = glm::rotate(model, (float)glfwGetTime() * s.speed_rotate, glm::vec3(0.0f, 0.0f, 1.0f));     // rotate
    model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));
    return model;
};