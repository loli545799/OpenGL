//vertex_2.vs
#version 330 core
layout (location = 0) in vec3 aPos;   // 位置变量的属性位置值为 0 
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform vec3 acolor;

out vec3 ourColor; // 向片段着色器输出一个颜色

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    ourColor = acolor; 
}