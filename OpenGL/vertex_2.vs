//vertex_2.vs
#version 330 core
layout (location = 0) in vec3 aPos;   // λ�ñ���������λ��ֵΪ 0 
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform vec3 acolor;

out vec3 ourColor; // ��Ƭ����ɫ�����һ����ɫ

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    ourColor = acolor; 
}