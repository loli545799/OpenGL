#version 330 core
layout (location = 0) in vec3 aPos;

out vec3 UV;

uniform mat4 view;
uniform mat4 projection;

void main()
{
   vec4 pos = projection * view * vec4(aPos.x, aPos.y, aPos.z, 1.0);
   UV = aPos;
   gl_Position = pos.xyww;
};