#ifndef _HEHEPIG_PORTAL_
#define _HEHEPIG_PORTAL_

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <learnopengl/shader_m.h>
#include <learnopengl/model.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>

#include "trans.h"


struct Portal {
    constexpr static float width=3.0f;
    constexpr static float height=6.0f;
    Portal* destPortal;
    Trans tran;
    Model door;
    bool show;
    

    Portal(): destPortal(NULL), door("resources/objs/cube_brown.glb"), show(false), tran() {}

    void Draw(const glm::mat4 &view, const glm::mat4 &proj, Shader &shader);

    glm::mat4 get_model() {return tran.get_model();}

    glm::vec3 get_normal() {return tran.rot * glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);}

    bool test_through(glm::vec3 from, glm::vec3 to);
};





#endif