#ifndef __HEHEPIG_TRANS__
#define __HEHEPIG_TRANS__

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

struct Trans {
    glm::mat4 rot;
    glm::mat4 scal;
    glm::mat4 tran;

    Trans() {
        rot             = glm::mat4(1.0f);
        scal            = glm::mat4(1.0f);
        tran            = glm::mat4(1.0f);
    }
    glm::mat4 get_model() {return tran*rot*scal;}
};

#endif