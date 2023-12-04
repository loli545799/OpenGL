#version 330 core
out vec4 FragColor;

//材质属性
struct Material {
    sampler2D diffuse;     //漫反射颜色
    sampler2D specular;    //镜面光照
    float shininess;       //反光度
}; 

//平行光属性
struct DirLight {
    vec3 direction;        //平行光方向
    vec3 ambient;          //环境光颜色
    vec3 diffuse;          //平行光漫反射颜色
    vec3 specular;         //平行光镜面光照
};

//点光源属性
struct PointLight {
    vec3 position;        //点光源位置
    float constant;       //点光源衰减常量
    float linear;         //点光源线性衰减一次项
    float quadratic;      //点光源平方衰减二次项
    vec3 ambient;         //环境光颜色
    vec3 diffuse;         //漫反射颜色
    vec3 specular;        //镜面光照
};

//聚光灯属性
struct SpotLight {
    vec3 position;       //聚光灯位置
    vec3 direction;      //聚光灯朝向
    float cutOff;        //聚光灯内圆锥角
    float outerCutOff;   //聚光灯外圆锥角
    vec3 ambient;        //环境光颜色  
    float constant;      //聚光灯衰减常量
    float linear;        //聚光灯线性衰减一次项
    float quadratic;     //聚光灯平方衰减二次项
    vec3 diffuse;        //漫反射颜色
    vec3 specular;       //镜面光照
};

#define NR_POINT_LIGHTS 4

in vec3 FragPos;         //世界空间坐标
in vec3 Normal;          //输出法线坐标
in vec2 TexCoords;       //UV坐标

uniform vec3 viewPos;                                      //摄像机位置
uniform DirLight dirLight;                                 //平行光
uniform PointLight pointLights[NR_POINT_LIGHTS];           //点光源
uniform SpotLight spotLight;                               //聚光灯
uniform Material material;                                 //材质

// function prototypes
vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir);
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir);
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir);

void main()
{    
    // properties
    vec3 norm = normalize(Normal);                       //法线向量
    vec3 viewDir = normalize(viewPos - FragPos);         //相机向量
    
    // == =====================================================
    // Our lighting is set up in 3 phases: directional, point lights and an optional flashlight
    // For each phase, a calculate function is defined that calculates the corresponding color
    // per lamp. In the main() function we take all the calculated colors and sum them up for
    // this fragment's final color.
    // == =====================================================
    // 计算平行光
    vec3 result = CalcDirLight(dirLight, norm, viewDir);
    // 计算点光源
    for(int i = 0; i < NR_POINT_LIGHTS; i++)
        result += CalcPointLight(pointLights[i], norm, FragPos, viewDir);    
    // 计算聚光灯
    result += CalcSpotLight(spotLight, norm, FragPos, viewDir);    
    
    FragColor = vec4(result, 1.0);
}

// 计算平行光
vec3 CalcDirLight(DirLight light, vec3 normal, vec3 viewDir)
{
    //项目到相机的向量
    vec3 lightDir = normalize(-light.direction);
    // 计算漫反射
    float diff = max(dot(normal, lightDir), 0.0);
    // 计算镜面反射
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    // 结果
    vec3 ambient = light.ambient * vec3(texture(material.diffuse, TexCoords));
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.diffuse, TexCoords));
    vec3 specular = light.specular * spec * vec3(texture(material.specular, TexCoords));
    return (ambient + diffuse + specular);
}

// 计算点光源
vec3 CalcPointLight(PointLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
    //项目到点光源的向量
    vec3 lightDir = normalize(light.position - fragPos);
    // 计算漫反射 
    float diff = max(dot(normal, lightDir), 0.0);
    // 计算镜面反射
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    // 计算衰减
    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));    
    // 结果
    vec3 ambient = light.ambient * vec3(texture(material.diffuse, TexCoords));
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.diffuse, TexCoords));
    vec3 specular = light.specular * spec * vec3(texture(material.specular, TexCoords));
    ambient *= attenuation;
    diffuse *= attenuation;
    specular *= attenuation;
    return (ambient + diffuse + specular);
}

// 计算聚光灯
vec3 CalcSpotLight(SpotLight light, vec3 normal, vec3 fragPos, vec3 viewDir)
{
    //项目到光源(摄像机)的向量
    vec3 lightDir = normalize(light.position - fragPos);
    // 计算漫反射
    float diff = max(dot(normal, lightDir), 0.0);
    // 计算镜面反射
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    // 计算衰减
    float distance = length(light.position - fragPos);
    float attenuation = 1.0 / (light.constant + light.linear * distance + light.quadratic * (distance * distance));    
    //  计算光强限制
    float theta = dot(lightDir, normalize(-light.direction)); 
    float epsilon = light.cutOff - light.outerCutOff;
    float intensity = clamp((theta - light.outerCutOff) / epsilon, 0.0, 1.0);
    // 结果
    vec3 ambient = light.ambient * vec3(texture(material.diffuse, TexCoords));
    vec3 diffuse = light.diffuse * diff * vec3(texture(material.diffuse, TexCoords));
    vec3 specular = light.specular * spec * vec3(texture(material.specular, TexCoords));
    ambient *= attenuation * intensity;
    diffuse *= attenuation * intensity;
    specular *= attenuation * intensity;
    return (ambient + diffuse + specular);
}