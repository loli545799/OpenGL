#include "scene.h"

void Scene::updateObjects(float duration) {
    for (auto it = objects.begin(); it != objects.end(); ++it) {
        dolce::vec3 pos = (*it)->body->getPosition();
        (*it)->lastPos = glm::vec3(pos.x, -pos.z, pos.y);
        (*it)->body->integrate(duration);
        (*it)->calculateInternals();
    }
}

void Scene::generateContacts() {
    // dolce::CollisionPlane plane;
    // plane.direction = dolce::vec3(0, 1, 0);
    // plane.offset = 0;

    collision_data.reset(max_contacts);
    collision_data.friction = (dolce::real)0.5;
    collision_data.restitution = (dolce::real)0.8;
    collision_data.tolerance = (dolce::real)0.1;

    vector<Object*> obj_canmove;
    vector<Object*> obj_static;

    for (auto op: objects) {
        if (op->canmove) obj_canmove.push_back(op);
        else obj_static.push_back(op);
    }

    // contact
    for (auto it1 = obj_canmove.begin(); it1 != obj_canmove.end(); ++it1) {

        bool getting_through_portal = (portals[0].show && portals[1].show) && ((*it1)->test_intersect(portals[0]) || (*it1)->test_intersect(portals[1]));

        if (!getting_through_portal) {
            // with halfplane
            for (auto pp: planes) {
                if (!collision_data.hasMoreContacts())
                    return;
                dolce::CollisionDetector::boxAndHalfSpace(**it1, *pp, &collision_data);
            }

            // with static obj (walls)
            for (auto it2 = obj_static.begin(); it2 != obj_static.end(); ++it2) {
                if (!collision_data.hasMoreContacts())
                    return;
                dolce::CollisionDetector::boxAndBox(**it1, **it2, &collision_data);
            }
        }

        // with another movable obj
        for (auto it2 = it1+1; it2 != obj_canmove.end(); ++it2) {
            if (!collision_data.hasMoreContacts())
                return;
            dolce::CollisionDetector::boxAndBox(**it1, **it2, &collision_data);
        }
    }

}

void Scene::update(float duration) {
    //float duration = 0.005f;

    updateObjects(duration);

    generateContacts();

    resolver.resolveContacts(collision_data.contact_array, collision_data.contact_count, duration);
}