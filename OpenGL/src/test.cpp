#include "../head.h"
#include "cube.h"
#include "camera.h"
#include "texture.h"
#include "light.h"
#include "skybox.h"

GLFWwindow* window;
glm::mat4 view, projection;

void initwindow();
void draw();

int main()
{
    initwindow();
    draw();
    return 0;
}

void initwindow() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // glfw window creation
    // --------------------
    window = glfwCreateWindow(screen_width, screen_height, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return;
    }
}


void draw() {
    glEnable(GL_DEPTH_TEST); // 启用深度测试

    //编译调色器
    Shader shader("vertex_3.vs", "fragment_3.fs");
    Shader LightShader("lightvertex_1.vs", "lightfragment_1.fs");
    Shader SkyboxShader("SkyboxVertex_1.vs", "SkyboxFragment_1.fs");


    GLuint VAO; // == VAO
    GLuint VBO; // == VBO
    GLuint LightVAO; // == LightVAO
    GLuint SkyboxVAO; 
    GLuint SkyboxVBO;
    unsigned int EBO;

    // 生成并绑定VAO和VBO
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindVertexArray(VAO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    // 绑定lightVAO
    glGenVertexArrays(1, &LightVAO);
    glBindVertexArray(LightVAO);

    // 只需要绑定VBO不用再次设置VBO的数据，因为箱子的VBO数据中已经包含了正确的立方体顶点数据
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glGenVertexArrays(1, &SkyboxVAO);
    glBindVertexArray(SkyboxVAO);

    // 绑定Skybox的VAO和VBO
    glGenBuffers(1, &SkyboxVBO);
    glBindBuffer(GL_ARRAY_BUFFER, SkyboxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(SkyboxVertices), SkyboxVertices, GL_STATIC_DRAW);

    glBindVertexArray(SkyboxVAO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // 设置灯立方体的顶点属性（灯来只有位置数据）
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // 盒子纹理
    //第一个纹理对象
    unsigned int container = loadTexture("resources/textures/container.jpg");
    //第二个纹理对象
    unsigned int wall = loadTexture("resources/textures/wall.jpg");
    // 第三个纹理对象
    unsigned int paper = loadTexture("resources/textures/paper.jpg");
 
    // skybox纹理
    vector<string> SkyboxPath = {
        "resources/skybox/right.png",   //right
        "resources/skybox/left.png",   //left
        "resources/skybox/top.png",   //top
        "resources/skybox/bottom.png",   //bottom
        "resources/skybox/front.png",   //front
        "resources/skybox/back.png"    //back
    };
    unsigned int SkyboxTexture = loadSkyboxTexture(SkyboxPath);

    //设置uniform前，激活着色器
    shader.use();

    //使用内置自定义函数
    shader.setInt("material.diffuse", 0);    //0 对应的GL_TEXTURE0
    shader.setInt("material.specular", 1);   //1 对应的GL_TEXTURE1

    while (!glfwWindowShouldClose(window))
    {
        // per - frame time logic
        // --------------------
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // be sure to activate shader when setting uniforms/drawing objects
        shader.use();
        shader.setVec3("viewPos", camera.Position);
        shader.setFloat("material.shininess", 32.0f);

        /*
           Here we set all the uniforms for the 5/6 types of lights we have. We have to set them manually and index
           the proper PointLight struct in the array to set each uniform variable. This can be done more code-friendly
           by defining light types as classes and set their values in there, or by using a more efficient uniform approach
           by using 'Uniform buffer objects', but that is something we'll discuss in the 'Advanced GLSL' tutorial.
        */
        // directional light
        shader.setVec3("dirLight.direction", -0.2f, -1.0f, -0.3f);
        shader.setVec3("dirLight.ambient", 0.05f, 0.05f, 0.05f);
        shader.setVec3("dirLight.diffuse", 0.4f, 0.4f, 0.4f);
        shader.setVec3("dirLight.specular", 0.5f, 0.5f, 0.5f);
        // point light 1
        shader.setVec3("pointLights[0].position", pointLightPositions[0]);
        shader.setVec3("pointLights[0].ambient", 0.05f, 0.05f, 0.05f);
        shader.setVec3("pointLights[0].diffuse", 0.8f, 0.8f, 0.8f);
        shader.setVec3("pointLights[0].specular", 1.0f, 1.0f, 1.0f);
        shader.setFloat("pointLights[0].constant", 1.0f);
        shader.setFloat("pointLights[0].linear", 0.09f);
        shader.setFloat("pointLights[0].quadratic", 0.032f);
        // point light 2
        shader.setVec3("pointLights[1].position", pointLightPositions[1]);
        shader.setVec3("pointLights[1].ambient", 0.05f, 0.05f, 0.05f);
        shader.setVec3("pointLights[1].diffuse", 0.8f, 0.8f, 0.8f);
        shader.setVec3("pointLights[1].specular", 1.0f, 1.0f, 1.0f);
        shader.setFloat("pointLights[1].constant", 1.0f);
        shader.setFloat("pointLights[1].linear", 0.09f);
        shader.setFloat("pointLights[1].quadratic", 0.032f);
        // point light 3
        shader.setVec3("pointLights[2].position", pointLightPositions[2]);
        shader.setVec3("pointLights[2].ambient", 0.05f, 0.05f, 0.05f);
        shader.setVec3("pointLights[2].diffuse", 0.8f, 0.8f, 0.8f);
        shader.setVec3("pointLights[2].specular", 1.0f, 1.0f, 1.0f);
        shader.setFloat("pointLights[2].constant", 1.0f);
        shader.setFloat("pointLights[2].linear", 0.09f);
        shader.setFloat("pointLights[2].quadratic", 0.032f);
        // point light 4
        shader.setVec3("pointLights[3].position", pointLightPositions[3]);
        shader.setVec3("pointLights[3].ambient", 0.05f, 0.05f, 0.05f);
        shader.setVec3("pointLights[3].diffuse", 0.8f, 0.8f, 0.8f);
        shader.setVec3("pointLights[3].specular", 1.0f, 1.0f, 1.0f);
        shader.setFloat("pointLights[3].constant", 1.0f);
        shader.setFloat("pointLights[3].linear", 0.09f);
        shader.setFloat("pointLights[3].quadratic", 0.032f);
        // spotLight
        shader.setVec3("spotLight.position", camera.Position);
        shader.setVec3("spotLight.direction", camera.Front);
        shader.setVec3("spotLight.ambient", 0.0f, 0.0f, 0.0f);
        shader.setVec3("spotLight.diffuse", 1.0f, 1.0f, 1.0f);
        shader.setVec3("spotLight.specular", 1.0f, 1.0f, 1.0f);
        shader.setFloat("spotLight.constant", 1.0f);
        shader.setFloat("spotLight.linear", 0.09f);
        shader.setFloat("spotLight.quadratic", 0.032f);
        shader.setFloat("spotLight.cutOff", glm::cos(glm::radians(12.5f)));
        shader.setFloat("spotLight.outerCutOff", glm::cos(glm::radians(15.0f)));

        // view/projection transformations
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)screen_width / (float)screen_height, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        shader.setMat4("projection", projection);
        shader.setMat4("view", view);

        // world transformation
        glm::mat4 model = glm::mat4(1.0f);
        shader.setMat4("model", model);


        /*
        // bind diffuse map
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, diffuseMap);
        // bind specular map
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, specularMap);
        */

        // render containers
        glBindVertexArray(VAO);
        for (unsigned int i = 0; i < 10; i++)
        {
            // calculate the model matrix for each object and pass it to shader before drawing
            glm::mat4 model = glm::mat4(1.0f);
            model = glm::translate(model, cubePositions[i]);
            model = glm::rotate(model, (float)glfwGetTime() + i * 5.0f, glm::vec3(1.0f, 0.3f, 0.5f));
            shader.setMat4("model", model);
            if (i % 2 == 0) {
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, container);
            }
            else if (i % 3 == 1) {
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, wall);
            }
            else
            {
                glActiveTexture(GL_TEXTURE0);
                glBindTexture(GL_TEXTURE_2D, paper);
            }

            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        // also draw the lamp object(s)
        LightShader.use();
        LightShader.setMat4("projection", projection);
        LightShader.setMat4("view", view);

        // 绘制点光源
        glBindVertexArray(LightVAO);
        for (unsigned int i = 0; i < 4; i++)
        {
            model = glm::mat4(1.0f);
            model = glm::translate(model, pointLightPositions[i]);
            model = glm::scale(model, glm::vec3(0.2f)); // Make it a smaller cube
            LightShader.setMat4("model", model);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        //渲染天空盒
        glDepthFunc(GL_LEQUAL);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, SkyboxTexture);

        SkyboxShader.use();
        SkyboxShader.setMat4("view", view);                //设置观察矩阵；
        SkyboxShader.setMat4("projection", projection);    //设置投影矩阵；
        glBindVertexArray(SkyboxVAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glDepthFunc(GL_LESS);


        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }


    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &LightVAO);
    glDeleteBuffers(1, &EBO);


    glfwTerminate();

}
