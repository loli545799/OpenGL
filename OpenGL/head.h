#pragma once
#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include<GL/glut.h>
#include"src/Shader.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <cstring>
#include <Windows.h>
#include<iostream>
#define STB_IMAGE_IMPLEMENTATION    //通过定义STB_IMAGE_IMPLEMENTATION，预处理器会修改头文件，让其只包含相关的函数定义源码，等于是将这个头文件变为一个 .cpp 文件了
#include "stb_image.h"             //图片处理

using namespace std;

static const float Pi = 3.1415926f;

// 窗口大小
int screenheight = 720;
int screenwidth = 1280;

float screen_width = 1280.0f;          //窗口宽度
float screen_height = 720.0f;          //窗口高度