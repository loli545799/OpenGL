#pragma once
#include "Contacts.h"


namespace dolce{
class IntersectionTests;
class CollisionDetector;

class CollisionPrimitive {
public:
    friend class IntersectionTests;
    friend class CollisionDetector;

    RigidBody *body;
    mat4 offset;

    void calculateInternals();

    vec3 getAxis(unsigned index) const;

    const mat4& getTransform() const;
protected:
    mat4 transform;
};

class CollisionSphere : public CollisionPrimitive {
public:
    real radius;
};

class CollisionPlane {
public:
    vec3 direction;
    real offset;
};

class CollisionBox : public CollisionPrimitive {
public:
    vec3 halfsize;
};

class IntersectionTests {
public:
    static bool sphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane);
    static bool sphereAndSphere(const CollisionSphere &one, const CollisionSphere &two);
    static bool boxAndBox(const CollisionBox &one, const CollisionBox &two);
    static bool boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane);
};

struct CollisionData {
    Contact *contact_array;
    Contact *contacts;

    int contacts_left;
    unsigned contact_count;

    real friction;
    real restitution;
    real tolerance;

    bool hasMoreContacts() const;

    void reset(unsigned max_contacts);

    void addContacts(unsigned count);
};

class CollisionDetector {
public:
    static unsigned sphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane, CollisionData *data
    );
    static unsigned sphereAndTruePlane(const CollisionSphere &sphere, const CollisionPlane &plane, CollisionData *data);
    static unsigned sphereAndSphere(const CollisionSphere &one, const CollisionSphere &two, CollisionData *data);
    static unsigned boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane, CollisionData *data);
    static unsigned boxAndBox(const CollisionBox &one, const CollisionBox &two, CollisionData *data);
    static unsigned boxAndPoint(const CollisionBox &box, const vec3 &point, CollisionData *data);
    static unsigned boxAndSphere(const CollisionBox &box, const CollisionSphere &sphere, CollisionData *data);
};
}