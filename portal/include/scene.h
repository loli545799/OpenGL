#ifndef __PORTAL_SCENE__
#define __PORTAL_SCENE__

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <learnopengl/model.h>
#include <learnopengl/camera.h>

#include <vector>
#include <string>
#include <iostream>

#include "dolce/CollideFine.h"
#include "Portal.h"
#include "trans.h"





class Object : public dolce::CollisionBox {
public:
    Model *p_model;
    std::string name;
    Trans trans;
    glm::vec3 lastPos;
    bool canmove;



public:
    Object(Model *pm=NULL, std::string nm="") {
        body = new dolce::RigidBody;
        p_model         = pm;
        name            = nm;
        canmove         = false;
    }

    ~Object() {
        delete body;
    }

    glm::mat4 get_posmodel() {
        auto matrix = body->getTransform();
        glm::mat4 res(1.0f);
        res[0][0] = matrix.data[0];
        res[0][1] = matrix.data[4];
        res[0][2] = matrix.data[8];
        res[0][3] = 0;

        res[1][0] = matrix.data[1];
        res[1][1] = matrix.data[5];
        res[1][2] = matrix.data[9];
        res[2][3] = 0;

        res[2][0] = matrix.data[2];
        res[2][1] = matrix.data[6];
        res[2][2] = matrix.data[10];
        res[2][3] = 0;

        res[3][0] = matrix.data[3];
        res[3][1] = matrix.data[7];
        res[3][2] = matrix.data[11];
        res[3][3] = 1;

        res = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1, 0, 0)) * res * glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1, 0, 0));
        return res;
    }


    bool test_intersect(Portal &portal) {
        // check if obj intersect with portal
        constexpr static dolce::real edges[12][6]{
            { 1,  1,  1,     1,  1, -1},
            { 1,  1, -1,    -1,  1, -1},
            {-1,  1, -1,    -1,  1,  1},
            {-1,  1,  1,     1,  1,  1},

            { 1,  1,  1,     1, -1,  1},
            { 1,  1, -1,     1, -1, -1},
            {-1,  1, -1,    -1, -1, -1},
            {-1,  1,  1,    -1, -1,  1},

            { 1, -1,  1,     1, -1, -1},
            { 1, -1, -1,    -1, -1, -1},
            {-1, -1, -1,    -1, -1,  1},
            {-1, -1,  1,     1, -1,  1},
        };
        bool is_intersect = false;
        for (int k = 0; k < 12; ++k) {
            dolce::vec3 p1(edges[k][0], edges[k][1], edges[k][2]);
            dolce::vec3 p2(edges[k][3], edges[k][4], edges[k][5]);
            p1.componentProductUpdate(halfsize);
            p2.componentProductUpdate(halfsize);

            p1 = body->getTransform().transform(p1);
            p2 = body->getTransform().transform(p2);

            glm::vec3 pt1(p1.x, -p1.z, p1.y);
            glm::vec3 pt2(p2.x, -p2.z, p2.y);
            if (portal.test_through(pt1, pt2) || portal.test_through(pt2, pt1)) {
                is_intersect = true;
                break;
            }
        }
        return is_intersect;
    }


    void Draw(Shader &shader) {
        if (p_model) {
            glm::mat4 model = get_posmodel();
            shader.setMat4("model", model * trans.get_model());
            p_model->Draw(shader);
        }
    }

};




class Scene {
public:
    std::vector<Object*> objects;
    std::vector<Portal*> walls;
    std::vector<dolce::CollisionPlane*> planes;
    Portal portals[2];
    Camera camera;

    struct {
        glm::vec3 position;     // 位置

        glm::vec3 ambient;      // 环境光
        glm::vec3 diffuse;      // 漫反射
        glm::vec3 specular;     // 镜面反射
    } light;

    // contact relative
    constexpr static unsigned max_contacts = 256;

    dolce::Contact contacts[max_contacts];

    dolce::CollisionData collision_data;

    dolce::ContactResolver resolver;


public:

    Scene() : resolver(max_contacts * 8) {
        collision_data.contact_array = contacts;
    }


    void init(const char* path);

    void add_object(Object *obj) {
        objects.push_back(obj);
    }

    void processInput(GLFWwindow *window, double deltaTime);
    void setDoor(int doorID);

    void draw_all(Shader &shader);
    void draw_normalobjs(Shader &shader, glm::mat4 view, glm::mat4 projection);
    void draw_portals(Shader &shader, glm::mat4 view, glm::mat4 projection);
    void update(float duration);

    void drawRecursivePortal(Portal &portal, const glm::mat4 &view, const glm::mat4 &projection, Shader &shader, int maxlevel, int level=1);

    void generateContacts();
    void updateObjects(float duration);
};





#endif