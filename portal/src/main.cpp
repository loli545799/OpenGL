#include <windows.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>

#include "camera.h"
#include "scene.h"
#include "portal.h"

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void curse_poscallback(GLFWwindow *window, double x, double y);
GLFWwindow* createWindow(int width, int height, const char* title);
int gladInit();

Camera *p_camera;
Scene *p_scene;

// settings
unsigned int SCR_WIDTH = 1280;
unsigned int SCR_HEIGHT = 720;
const float PI = 3.1415926;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;





void mainloop(GLFWwindow *window)
{
    //Shader shader("resources/shaders/ambient.vs", "resources/shaders/ambient.fs");
    Shader shader("resources/shaders/phong.vs", "resources/shaders/phong.fs");
    
    glEnable(GL_DEPTH_TEST);


    while (!glfwWindowShouldClose(window)) {
        
        // per-frame time logic
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        if (deltaTime < 0.01) continue;
        lastFrame = currentFrame;
    

        p_scene->processInput(window, deltaTime);
    
        p_scene->update(deltaTime);

        glClearColor(0, 0.3, 0.2, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
        shader.use();

        p_scene->light.position  = p_scene->objects[1]->lastPos;
        shader.setVec3("light.position", p_scene->light.position);
        shader.setVec3("light.ambient", p_scene->light.ambient);
        shader.setVec3("light.diffuse", p_scene->light.diffuse);
        shader.setVec3("light.specular", p_scene->light.specular);

        p_scene->draw_all(shader);


        glfwSwapBuffers(window);
        glfwPollEvents();

    }



}



int main(int argc, char **argv)
{
    glfwInit();

    GLFWwindow* window = createWindow(SCR_WIDTH, SCR_HEIGHT, "portal");

    gladInit();

    // init
    Scene scene;
    p_scene = &scene;
    p_camera = &scene.camera;
    for (int i=1; i<argc; i++) {
        scene.init(argv[i]);
    }
    scene.portals[0].destPortal    = &scene.portals[1];
    scene.portals[1].destPortal    = &scene.portals[0];

    scene.light.ambient   = glm::vec3(0.5f, 0.4f, 0.4f);
    scene.light.diffuse   = glm::vec3(0.5f, 0.5f, 0.5f);
    scene.light.specular  = glm::vec3(1.0f, 1.0f, 1.0f);

    scene.camera.Position = glm::vec3(0, -3, 4);

    mainloop(window);

    glfwTerminate();

    return 0;
}











//========================================================================================
//========================================================================================





void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
    SCR_HEIGHT = height;
    SCR_WIDTH = width;
}

void curse_poscallback(GLFWwindow *window, double xposIn, double yposIn)
{
    static float lastX = SCR_WIDTH / 2.0f;
    static float lastY = SCR_HEIGHT / 2.0f;
    static bool firstMouse = true;


    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);
    

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    //std::cout <<lastX <<" " <<lastY <<"->" <<xpos <<" " <<ypos <<"(" <<xposIn <<" " <<yposIn <<")" <<std::endl;

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    p_camera->ProcessMouseMovement(xoffset, yoffset);
}


//========================================================================================
//========================================================================================

GLFWwindow* createWindow(int width, int height, const char* title)
{
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(width, height, title, NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return NULL;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    
    glfwSetCursorPosCallback(window, curse_poscallback);

    //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    return window;
}

int gladInit()
{
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    return 0;
}

