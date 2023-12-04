//vertex_1.vs
#version 330 core
layout (location = 0) in vec3 aPos;   // 位置变量的属性位置值为 0 
layout (location = 1) in vec3 aColor; // 颜色变量的属性位置值为 1
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform float timevalue;
//uniform vec3 camera_front; // 相机前向量
//uniform vec3 light_pos;    // 光源位置

out vec3 ourColor; // 向片段着色器输出一个颜色

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    ourColor = aColor; // 将ourColor设置为我们从顶点数据那里得到的输入颜色
    for (int i = 0; i < 3; i++) {
         if((ourColor[i] + timevalue)> 1.0f){
               ourColor[i]>timevalue?ourColor[i]-=timevalue:ourColor[i]=timevalue-ourColor[i];
         }else if((ourColor[i] + timevalue) < 0.0f){
               ourColor[i]>timevalue?ourColor[i]-=timevalue:ourColor[i]=timevalue-ourColor[i];
         }else{
               ourColor[i]+=timevalue; 
         }
    }
    
}