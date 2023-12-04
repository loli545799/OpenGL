#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <learnopengl/model.h>
#include <learnopengl/camera.h>

#include <vector>
#include <string>
#include <iostream>
#include <map>

#include "scene.h"
#include "utils.h"
#include <dolce/mathematics.h>

#include <cassert>

using namespace std;

static unsigned int SCR_WIDTH = 1280;
static unsigned int SCR_HEIGHT = 720;


static glm::mat4 const clippedProjMat(Portal &portal, glm::mat4 viewMat, glm::mat4 projMat);

static void strtouch(map<string, string> &m, const string &k, const string &def) {
    if (!m.count(k)) m[k] = def;
}


//=============================================================
//=============================================================


void Scene::init(const char *path)
{
    auto conf = readConfig(path);

    Object *tmpobj;
    Portal *tmptal;
    dolce::CollisionPlane *plane;
    dolce::Quaternion tmporientation;


    for (auto &[objname, obj]: conf) {
        if (obj["type"] == "half_plane") {
            plane = new dolce::CollisionPlane;
            assert(obj.count("dir_x"));
            assert(obj.count("dir_y"));
            assert(obj.count("dir_z"));
            strtouch(obj, "offset", "0");
            plane->direction    = dolce::vec3(stof(obj["dir_x"]), stof(obj["dir_z"]), -stof(obj["dir_y"]));
            plane->offset       = stof(obj["offset"]);
            planes.push_back(plane);
        }
        else if (obj["type"] == "virtual_wall") {
            tmptal = new Portal();

            strtouch(obj, "scal_x", "1");
            strtouch(obj, "scal_y", "1");
            strtouch(obj, "scal_z", "1");
            strtouch(obj, "scal_all", "1");
            tmptal->tran.scal    = glm::scale(glm::mat4(1.0f), glm::vec3(stof(obj["scal_x"]), stof(obj["scal_y"]), stof(obj["scal_z"])) * stof(obj["scal_all"]));

            strtouch(obj, "rot_x", "0");
            strtouch(obj, "rot_y", "0");
            strtouch(obj, "rot_z", "0");
            tmptal->tran.rot        = glm::rotate(glm::mat4(1.0f), glm::radians(stof(obj["rot_x"])), glm::vec3(1.0f, 0.0f, 0.0f)) * tmptal->tran.rot;
            tmptal->tran.rot        = glm::rotate(glm::mat4(1.0f), glm::radians(stof(obj["rot_y"])), glm::vec3(0.0f, 1.0f, 0.0f)) * tmptal->tran.rot;
            tmptal->tran.rot        = glm::rotate(glm::mat4(1.0f), glm::radians(stof(obj["rot_z"])), glm::vec3(0.0f, 0.0f, 1.0f)) * tmptal->tran.rot;

            strtouch(obj, "tran_x", "0");
            strtouch(obj, "tran_y", "0");
            strtouch(obj, "tran_z", "0");
            tmptal->tran.tran        = glm::translate(glm::mat4(1.0f), glm::vec3(stof(obj["tran_x"]), stof(obj["tran_y"]), stof(obj["tran_z"])));

            walls.push_back(tmptal);
        }
        else if (obj["type"] == "normal_obj") {
            
            assert(obj.count("path"));
            tmpobj = new Object(new Model(obj["path"]), objname);

            strtouch(obj, "scal_x", "1");
            strtouch(obj, "scal_y", "1");
            strtouch(obj, "scal_z", "1");
            strtouch(obj, "scal_all", "1");
            tmpobj->trans.scal  = glm::scale(glm::mat4(1.0f), glm::vec3(stof(obj["scal_x"]), stof(obj["scal_y"]), stof(obj["scal_z"])) * stof(obj["scal_all"]));

            strtouch(obj, "tran_x", "0");
            strtouch(obj, "tran_y", "0");
            strtouch(obj, "tran_z", "0");
            tmpobj->trans.tran  = glm::translate(glm::mat4(1.0f), glm::vec3(stof(obj["tran_x"]), stof(obj["tran_y"]), stof(obj["tran_z"])));

            strtouch(obj, "pos_x", "0");
            strtouch(obj, "pos_y", "0");
            strtouch(obj, "pos_z", "0");
            tmpobj->body->setPosition(dolce::vec3(stof(obj["pos_x"]), stof(obj["pos_z"]), -stof(obj["pos_y"])));
            tmpobj->lastPos = glm::vec3(stof(obj["pos_x"]), stof(obj["pos_y"]), stof(obj["pos_z"]));

            tmpobj->body->setOrientation(dolce::Quaternion(1, 0, 0, 0));

            tmpobj->body->setVelocity(dolce::vec3(0, 0, 0));
            tmpobj->body->setRotation(dolce::vec3(0, 0, 0));

            strtouch(obj, "halfsz_x", "1");
            strtouch(obj, "halfsz_y", "1");
            strtouch(obj, "halfsz_z", "1");
            strtouch(obj, "halfsz_all", "1");
            glm::vec3 tmphalfsz = glm::vec3(stof(obj["halfsz_x"]), stof(obj["halfsz_y"]), stof(obj["halfsz_z"])) * stof(obj["halfsz_all"]);
            tmpobj->halfsize = dolce::vec3(tmphalfsz.x, tmphalfsz.z, tmphalfsz.y);

            strtouch(obj, "rot_x", "0");
            strtouch(obj, "rot_y", "0");
            strtouch(obj, "rot_z", "0");
            tmporientation = dolce::Quaternion(1, 0, 0, 0);
            tmporientation.rotate(dolce::vec3(1, 0, 0), stof(obj["rot_x"]));
            tmporientation.rotate(dolce::vec3(0, 0, -1), stof(obj["rot_y"]));
            tmporientation.rotate(dolce::vec3(0, 1, 0), stof(obj["rot_z"]));
            tmpobj->body->setOrientation(tmporientation);

            strtouch(obj, "canmove", "0");
            if (tmpobj->canmove     = atoi(obj["canmove"].c_str())) {
                strtouch(obj, "density", "10");
                dolce::real density = stof(obj["density"]);
                dolce::real mass = 30;
                tmpobj->body->setMass(mass);

                dolce::mat3 tensor;
                tensor.setBlockInertiaTensor(tmpobj->halfsize, mass);
                tmpobj->body->setInertiaTensor(tensor);

                tmpobj->body->setAcceleration(dolce::vec3(0, -10.0f, 0));
            }
            else {
                tmpobj->body->setInverseMass(0);
                tmpobj->body->setInverseInertiaTensor(dolce::mat3());
                tmpobj->body->setAcceleration(dolce::vec3(0, 0, 0));
            }

            tmpobj->body->setLinearDamping(0.95f);
            tmpobj->body->setAngularDamping(0.8f);
            tmpobj->body->clearAccumulators();

            tmpobj->body->setCanSleep(false);
            tmpobj->body->setAwake();

            tmpobj->body->calculateDerivedData();
            tmpobj->calculateInternals();


            objects.push_back(tmpobj);
        }
    }


    for (auto &o: conf)
    {
        std::cout << "objname: " << o.first << std::endl;
        for (auto &a: o.second)
            std::cout << "key: " << a.first << "\t value: " << a.second << std::endl;
        cout <<endl;
    }

}



//=============================================================
//=============================================================

void Scene::setDoor(int doorID)
{
    Portal &portal = portals[doorID];
    glm::vec3 curpos = camera.Position;
    glm::vec3 dstpos;
    glm::vec3 dirvec = camera.Front;
    glm::vec3 normal;
    bool flag;
    float L, R, M;
    float mindis = 1e100;
    const float eps = 0.001;
    Portal *pwall = NULL;

    for (auto wall: walls) {
        flag = false;
        for (L=0.1, R=100, M=(L+R)/2; R-L>eps; M=(L+R)/2) {
            dstpos = curpos + dirvec * M;
            if (wall->test_through(curpos, dstpos)) {
                flag = true;
                R = M;
            }
            else {
                L = M;
            }
        }
        if (flag && M < mindis) {
            mindis = M;
            pwall = wall;
        }
    }

    if (pwall) {
        normal = pwall->get_normal();
        dstpos = curpos + dirvec * mindis;

        portal.show = true;
        portal.tran.tran        = glm::translate(glm::mat4(1.0f), dstpos + normal * 0.1f);
        portal.tran.rot         = pwall->tran.rot;
    }


}






//=============================================================
//=============================================================

static int draw_myself_flag = 0;

void Scene::draw_all(Shader &shader) {

    glm::mat4 view          = camera.GetViewMatrix();
    glm::mat4 projection    = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.01f, 100.0f);

    draw_myself_flag = 1;
    draw_portals(shader, view, projection);
    draw_myself_flag = 1;

    //for (auto w: walls) w->Draw(view, projection, shader);
    draw_normalobjs(shader, view, projection);
}


void Scene::draw_portals(Shader &shader, glm::mat4 view, glm::mat4 projection) {
    if (portals[0].show && portals[1].show) {
        drawRecursivePortal(portals[0], view, projection, shader, 3, 1);
        drawRecursivePortal(portals[1], view, projection, shader, 3, 1);
    }
    else {
        if (portals[0].show) portals[0].Draw(view, projection, shader);
        if (portals[1].show) portals[0].Draw(view, projection, shader);
    }
}


void Scene::draw_normalobjs(Shader &shader, glm::mat4 view, glm::mat4 projection) {
    shader.setMat4("projection", projection);
    shader.setMat4("view", view);

    for (auto o: objects) if (draw_myself_flag || o != objects[0]) {
        if (o == objects[1]) {
            shader.setVec3("light.ambient", glm::vec3(1.0f));
            o->Draw(shader);
            shader.setVec3("light.ambient", light.ambient);
        }
        else {
            o->Draw(shader);
        }
    }



    if (portals[0].show && portals[1].show) {
        for (auto o: objects) if (o->canmove) if (draw_myself_flag || o != objects[0]) {
            for (int i=0; i<2; i++) if (o->test_intersect(portals[i])) {
                glm::mat4 tmpmodel =              
                portals[i].destPortal->get_model()
                * glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f))
                * glm::inverse(portals[i].get_model())
                * o->get_posmodel();

                shader.setMat4("model", tmpmodel * o->trans.get_model());
                if (o == objects[1]) {
                    shader.setVec3("light.ambient", glm::vec3(1.0f));
                    o->p_model->Draw(shader);
                    shader.setVec3("light.ambient", light.ambient);
                }
                else {
                    o->p_model->Draw(shader);
                }
                break;
            }
        }
    }
}




// level 从 1 开始
void Scene::drawRecursivePortal(Portal &portal, const glm::mat4 &view, const glm::mat4 &projection, Shader &shader, int maxlevel, int level)
{
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);
    glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);
    glStencilMask(0xff);
    glStencilFunc(GL_EQUAL, level-1, 0xff);
    glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);

    // 绘制门的范围模板
    portal.Draw(view, projection, shader);



    // 新的 view 矩阵
    glm::mat4 destView = view
        * portal.get_model()
        * glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f))
        * glm::inverse(portal.destPortal->get_model());
    // 斜视锥
    glm::mat4 destProj = clippedProjMat(portal, destView, projection);
    //glm::mat4 destProj = projection;

    if (level < maxlevel) {
        drawRecursivePortal(portal, destView, destProj, shader, maxlevel, level+1);
    }

    glStencilFunc(GL_EQUAL, level, 0xff);
    glStencilMask(0x00);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);
    draw_normalobjs(shader, destView, destProj);


    glEnable(GL_DEPTH_TEST);
    glEnable(GL_STENCIL_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_ALWAYS);
    glStencilFunc(GL_EQUAL, level, 0xff);
    glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
    glStencilMask(0xff);
    portal.Draw(view, projection, shader);

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthFunc(GL_LESS);
    glStencilMask(0xff);
    glDisable(GL_STENCIL_TEST);    
}







void Scene::processInput(GLFWwindow *window, double deltaTime)
{

    glm::vec3 old_pos = camera.Position;


    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_LEFT))
        setDoor(0);
    else if (glfwGetMouseButton(window,GLFW_MOUSE_BUTTON_RIGHT))
        setDoor(1);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        camera.ProcessKeyboard(UP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        camera.ProcessKeyboard(DOWN, deltaTime);

    if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) {
        auto &o = objects[0];
        glm::vec3 tmpvec = camera.Right;
        tmpvec.z = 0;
        tmpvec = glm::normalize(tmpvec) * float(-20.0 * o->body->getMass());
        o->body->addForce(dolce::vec3(tmpvec.x, 0, -tmpvec.y));
    }
    if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) {
        auto &o = objects[0];
        glm::vec3 tmpvec = camera.Right;
        tmpvec.z = 0;
        tmpvec = glm::normalize(tmpvec) * float(20.0 * o->body->getMass());
        o->body->addForce(dolce::vec3(tmpvec.x, 0, -tmpvec.y));
    }
    if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS) {
        auto &o = objects[0];
        o->body->addForce(dolce::vec3(0, (20+20) * o->body->getMass(), 0));
    }
    if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS) {
        auto &o = objects[0];
        o->body->addForce(dolce::vec3(0, -(20+20) * o->body->getMass(), 0));
    }
    if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) {
        auto &o = objects[0];
        // o->body->addForce(dolce::vec3(0, 0, -20 * o->body->getMass()));
        glm::vec3 tmpvec = camera.Front;
        tmpvec.z = 0;
        tmpvec = glm::normalize(tmpvec) * float(20.0 * o->body->getMass());
        o->body->addForce(dolce::vec3(tmpvec.x, 0, -tmpvec.y));
    }
    if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) {
        auto &o = objects[0];
        glm::vec3 tmpvec = camera.Front;
        tmpvec.z = 0;
        tmpvec = glm::normalize(tmpvec) * float(-20.0 * o->body->getMass());
        o->body->addForce(dolce::vec3(tmpvec.x, 0, -tmpvec.y));
    }


    int pid = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
        portals[pid].tran.tran = glm::translate(glm::mat4(1.0f), glm::vec3(0.0, 0.03, 0.0)) * portals[pid].tran.tran;
    }
    if (glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS) {
        portals[pid].tran.tran = glm::translate(glm::mat4(1.0f), glm::vec3(0.0, -0.03, 0.0)) * portals[pid].tran.tran;
    }
    if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
        portals[pid].tran.tran = glm::translate(glm::mat4(1.0f), glm::vec3(-0.03, 0.0, 0.0)) * portals[pid].tran.tran;
    }
    if (glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS) {
        portals[pid].tran.tran = glm::translate(glm::mat4(1.0f), glm::vec3(0.03, 0.0, 0.0)) * portals[pid].tran.tran;
    }
    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS) {
        portals[pid].tran.rot = glm::rotate(glm::mat4(1.0f), glm::radians(1.0f), glm::vec3(0.0, 0.0, 1.0)) * portals[pid].tran.rot;
    }
    if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS) {
        portals[pid].tran.rot = glm::rotate(glm::mat4(1.0f), glm::radians(1.0f), glm::vec3(1.0, 0.0, 0.0)) * portals[pid].tran.rot;
    }

    if (portals[0].show && portals[1].show) for (auto obj: objects) if (obj->canmove) {

        glm::vec3 old_obj_pos = obj->lastPos;
        dolce::vec3 tmp_vec3 = obj->body->getPosition();
        glm::vec3 new_obj_pos(tmp_vec3.x, -tmp_vec3.z, tmp_vec3.y);

        for (int i = 0; i < 2; ++i) if (portals[i].test_through(old_obj_pos, new_obj_pos)) {
            dolce::vec3 position = obj->body->getPosition();
            glm::vec3 pos(position.x, -position.z, position.y);
            glm::mat4 tmpmodel = portals[i].destPortal->get_model()
                * glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f))
                * glm::inverse(portals[i].get_model());
            pos = glm::vec3(tmpmodel * glm::vec4(pos, 1.0f));
            obj->body->setPosition(dolce::vec3(pos.x, pos.z, -pos.y));

            glm::vec3 n1 = -glm::vec3(portals[i].tran.rot * glm::vec4(0, 0, 1, 1));
            glm::vec3 n2 = glm::vec3(portals[i].destPortal->tran.rot * glm::vec4(0, 0, 1, 1));
           

            dolce::vec3 velocity = obj->body->getVelocity();
            dolce::Quaternion orientation = obj->body->getOrientation();
            orientation.normalize();


            double dot = glm::dot(n1, n2);
            dolce::vec3 normal1(n1.x, n1.z, -n1.y);
            dolce::vec3 normal2(n2.x, n2.z, -n2.y);
            if (dot >= 0.99) {
                // no need to change
            }
            else if (dot <= -0.99) {
                velocity.invert();
                obj->body->setVelocity(velocity);

                dolce::real x = fabs(normal1.x);
                dolce::real y = fabs(normal1.y);
                dolce::real z = fabs(normal1.z);

                dolce::vec3 other = x < y ? (x < z ? dolce::vec3(1, 0, 0) : dolce::vec3(0, 0, 1)) : (y < z ? dolce::vec3(0, 1, 0) : dolce::vec3(0, 0, 1));

                dolce::vec3 ortho = normal1 % other;
                ortho.normalize();
                dolce::Quaternion q(0, ortho.x, ortho.y, ortho.z);
                q *= orientation;

                obj->body->setOrientation(q);
            }
            else {

                dolce::vec3 a = normal1 % normal2;
                dolce::Quaternion q(0, a.x, a.y, a.z);
                q.r = normal1.length() * normal2.length() + normal1 * normal2;
                q.normalize();

                dolce::Quaternion v = q;
                v *= dolce::Quaternion(0, velocity.x, velocity.y, velocity.z);
                v *= q.conjugate();
                
                q *= orientation;

                obj->body->setVelocity(dolce::vec3(v.i, v.j, v.k));
                obj->body->setOrientation(q);
            }
            break;
        }
    }


if (0) {
    camera.Position = objects[0]->lastPos;
}
else {
    glm::vec3 new_pos = camera.Position;
    for (int i=0; i<2; i++) if (portals[i].show && portals[i].test_through(old_pos, new_pos)) {
        std::cout <<camera.Position.x <<" " <<camera.Position.y <<" " <<camera.Position.z <<std::endl;
        glm::mat4 tmpmodel = portals[i].destPortal->get_model()
            * glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f))
            * glm::inverse(portals[i].get_model());

        camera.Position = glm::vec3(tmpmodel * glm::vec4(camera.Position, 1.0f));

        glm::vec3 Base = glm::vec3(tmpmodel * glm::vec4(0, 0, 0, 1));
        glm::vec3 Up = glm::vec3(tmpmodel * glm::vec4(camera.Up, 1.0f)) - Base;
        glm::vec3 Right = glm::vec3(tmpmodel * glm::vec4(camera.Right, 1.0f)) - Base;

        camera.Front = glm::normalize(glm::cross(Up, Right));
        camera.updateCameraVectors(false);
        std::cout <<camera.Position.x <<" " <<camera.Position.y <<" " <<camera.Position.z <<std::endl;
        break;
    }
}


}



//=======================================================
//=======================================================

static glm::mat4 const clippedProjMat(Portal &portal, glm::mat4 viewMat, glm::mat4 projMat) {

    glm::vec3 Normal = glm::normalize(glm::vec3(viewMat * (portal.destPortal->get_model() * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f) - portal.destPortal->get_model() * glm::vec4(0.0f, 0.0f, 0.0f, 0.0f))));
    glm::vec3 d_position = glm::vec3(viewMat * portal.destPortal->get_model() * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));

    glm::vec4 C(Normal, -glm::dot(Normal, d_position));

    glm::mat4 tmp = projMat;
    //for (int i=0; i<4; i++) for (int j=i+1; j<4; j++) swap(tmp[i][j], tmp[j][i]);

    glm::vec4 CC = glm::transpose(glm::inverse(tmp)) * C;

    glm::vec4 Q;
    Q.x = (glm::sign(CC.x) + projMat[2][0]) / projMat[0][0];
    Q.y = (glm::sign(CC.y) + projMat[2][1]) / projMat[1][1];
    Q.z = -1.0f;
    Q.w = (1.0f + projMat[2][2]) / projMat[3][2];
    
    C = C * (2.0f / glm::dot(C, Q));
    projMat[0][2] = C.x;

    projMat[0][2] = C.x;
    projMat[1][2] = C.y;
    projMat[2][2] = C.z + 1.0f;
    projMat[3][2] = C.w;

    return projMat;
}