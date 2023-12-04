#include "portal.h"


void Portal::Draw(const glm::mat4 &view, const glm::mat4 &projection, Shader &shader) {
    static const glm::mat4 hhhh = glm::scale(glm::mat4(1.0f), glm::vec3(width/2, height/2, 0.0001f)*0.01f);
    glm::mat4 model = tran.get_model() * hhhh;
    shader.setMat4("model", model);
    shader.setMat4("projection", projection);
    shader.setMat4("view", view);
    door.Draw(shader);
}


static void printvec4(glm::vec4 x) {
    std::cout <<x.x <<" " <<x.y <<" " <<x.z <<std::endl;
}



bool Portal::test_through(glm::vec3 from_, glm::vec3 to_) {
    
    glm::vec4 from = glm::inverse(tran.get_model()) * glm::vec4(from_, 1.0f);
    glm::vec4 to = glm::inverse(tran.get_model()) * glm::vec4(to_, 1.0f);

    //printvec4(from);
    //printvec4(to);
    
    const float mxpos = 0.0005;

    if ((from.z > mxpos && to.z < mxpos)) {
        float t = to.z / (to.z-from.z);
        float x = to.x + t*(from.x-to.x);
        float y = to.y + t*(from.y-to.y);
        //std::cout <<x <<" " <<y <<std::endl;
        return fabs(x)<=width/2 && fabs(y)<=height/2;
    }
    else {
        return false;
    }
}