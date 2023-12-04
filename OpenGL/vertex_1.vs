//vertex_1.vs
#version 330 core
layout (location = 0) in vec3 aPos;   // λ�ñ���������λ��ֵΪ 0 
layout (location = 1) in vec3 aColor; // ��ɫ����������λ��ֵΪ 1
uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;
uniform float timevalue;
//uniform vec3 camera_front; // ���ǰ����
//uniform vec3 light_pos;    // ��Դλ��

out vec3 ourColor; // ��Ƭ����ɫ�����һ����ɫ

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    ourColor = aColor; // ��ourColor����Ϊ���ǴӶ�����������õ���������ɫ
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