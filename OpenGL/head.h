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
#define STB_IMAGE_IMPLEMENTATION    //ͨ������STB_IMAGE_IMPLEMENTATION��Ԥ���������޸�ͷ�ļ�������ֻ������صĺ�������Դ�룬�����ǽ����ͷ�ļ���Ϊһ�� .cpp �ļ���
#include "stb_image.h"             //ͼƬ����

using namespace std;

static const float Pi = 3.1415926f;

// ���ڴ�С
int screenheight = 720;
int screenwidth = 1280;

float screen_width = 1280.0f;          //���ڿ��
float screen_height = 720.0f;          //���ڸ߶�