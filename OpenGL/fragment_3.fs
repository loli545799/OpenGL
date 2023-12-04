#version 330 core
out vec4 FragColor;

//��������
struct Material {
    sampler2D diffuse;     //��������ɫ
    sampler2D specular;    //�������
    float shininess;       //�����
}; 

//ƽ�й�����
struct DirLight {
    vec3 direction;        //ƽ�йⷽ��
    vec3 ambient;          //��������ɫ
    vec3 diffuse;          //ƽ�й���������ɫ
    vec3 specular;         //ƽ�й⾵�����
};

//���Դ����
struct PointLight {
    vec3 position;        //���Դλ��
    float constant;       //���Դ˥������
    float linear;         //���Դ����˥��һ����
    float quadratic;      //���Դƽ��˥��������
    vec3 ambient;         //��������ɫ
    vec3 diffuse;         //��������ɫ
    vec3 specular;        //�������
};

//�۹������
struct SpotLight {
    vec3 position;       //�۹��λ��
    vec3 direction;      //�۹�Ƴ���
    float cutOff;        //�۹����Բ׶��
    float outerCutOff;   //�۹����Բ׶��
    vec3 ambient;        //��������ɫ  
    float constant;      //�۹��˥������
    float linear;        //�۹������˥��һ����
    float quadratic;     //�۹��ƽ��˥��������
    vec3 diffuse;        //��������ɫ
    vec3 specular;       //�������
};

#define NR_POINT_LIGHTS 4

in vec3 FragPos;         //����ռ�����
in vec3 Normal;          //�����������
in vec2 TexCoords;       //UV����

uniform vec3 viewPos;                                      //�����λ��
uniform DirLight dirLight;                                 //ƽ�й�
uniform PointLight pointLights[NR_POINT_LIGHTS];           //���Դ
uniform SpotLight spotLight;                               //�۹��
uniform Material material;                                 //����

// function prototypes
vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir);
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir);
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir);

void main()
{    
    // properties
    vec3 norm = normalize(Normal);                       //��������
    vec3 viewDir = normalize(viewPos - FragPos);         //�������
    
    // == =====================================================
    // Our lighting is set up in 3 phases: directional, point lights and an optional flashlight
    // For each phase, a calculate function is defined that calculates the corresponding color
    // per lamp. In the main() function we take all the calculated colors and sum them up for
    // this fragment's final color.
    // == =====================================================
    // ����ƽ�й�
    vec3 result = CalcDirLight(dirLight, norm, viewDir);
    // ������Դ
    for(int i = 0; i < NR_POINT_LIGHTS; i++)
        result += CalcPointLight(pointLights[i], norm, FragPos, viewDir);    
    // ����۹��
    result += CalcSpotLight(spotLight, norm, FragPos, viewDir);    
    
    FragColor = vec4(result, 1.0);
}

// ����ƽ�й�
vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir)
{
    //��Ŀ�����������
    vec3 lightDir = normalize(-light.direction);
    // ����������
    float diff = max(dot(normal, lightDir), 0.0);
    // ���㾵�淴��
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    // ���
    vec3 ambient = light.ambient * vec3(texture(material.diffuse, TexCoords));
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.diffuse, TexCoords));
    vec3 specular = light.specular * spec * vec3(texture(material.specular, TexCoords));
    return (ambient + diffuse + specular);
}

// ������Դ
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
    //��Ŀ�����Դ������
    vec3 lightDir = normalize(light.position - fragPos);
    // ���������� 
    float diff = max(dot(normal, lightDir), 0.0);
    // ���㾵�淴��
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    // ����˥��
    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));    
    // ���
    vec3 ambient = light.ambient * vec3(texture(material.diffuse, TexCoords));
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.diffuse, TexCoords));
    vec3 specular = light.specular * spec * vec3(texture(material.specular, TexCoords));
    ambient *= attenuation;
    diffuse *= attenuation;
    specular *= attenuation;
    return (ambient + diffuse + specular);
}

// ����۹��
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
    //��Ŀ����Դ(�����)������
    vec3 lightDir = normalize(light.position - fragPos);
    // ����������
    float diff = max(dot(normal, lightDir), 0.0);
    // ���㾵�淴��
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    // ����˥��
    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));    
    //  �����ǿ����
    float theta = dot(lightDir, normalize(-light.direction)); 
    float epsilon = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);
    // ���
    vec3 ambient = light.ambient * vec3(texture(material.diffuse, TexCoords));
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.diffuse, TexCoords));
    vec3 specular = light.specular * spec * vec3(texture(material.specular, TexCoords));
    ambient *= attenuation * intensity;
    diffuse *= attenuation * intensity;
    specular *= attenuation * intensity;
    return (ambient + diffuse + specular);
}