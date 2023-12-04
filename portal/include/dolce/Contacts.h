#pragma once
#include "Body.h"

namespace dolce{
class ContactResolver;

class Contact {
    friend class ContactResolver;
protected:
    mat3 contact_to_world;
    
    vec3 contact_velocity;
    
    real desired_delta_velocity;

    vec3 relative_contact_position[2];

protected:
    void calculateInternals(real duration);

    void swapBodies();

    void matchAwakeState();

    void calculateDesiredDeltaVelocity(real duration);

    vec3 calculateLocalVelocity(unsigned body_index, real duration);

    void calculateContactBasis();

    void applyImpulse(const vec3 &impulse, RigidBody *body, vec3 *velocity_change, vec3 *rotation_change);

    void applyVelocityChange(vec3 velocity_change[2], vec3 rotation_change[2]);

    void applyPositionChange(vec3 linear_change[2], vec3 angular_change[2], real penetration);

    vec3 calculateFrictionLessImpulse(mat3 *inverseInertiaTensor);

    vec3 calculateFrictionImpulse(mat3 *inverseInertiaTensor);

public:
    RigidBody *body[2];
    
    real friction;
    
    real restitution;

    vec3 contact_point;
    vec3 contact_normal;

    real penetration;

    void setBodyData(RigidBody *one, RigidBody *two, real friction, real restitution);
};


class ContactResolver{
protected:
    unsigned velocity_iterations;
    unsigned position_iterations;
    real velocity_epsilon;
    real position_epsilon;
public:
    unsigned velocity_iterations_used;
    unsigned position_iterations_used;

private:
    bool valid_settings;

public:
    ContactResolver(unsigned iterations, real velocity_epsilon = (real)0.01, real position_epsilon = (real)0.01);

    ContactResolver(unsigned velocity_iterations, unsigned position_iterations, real velocity_epsilon = (real)0.01, real position_epsilon = (real)0.01);

    bool isValid();

    void setIterations(unsigned velocity_iterations, unsigned position_iterations);

    void setIterations(unsigned iterations);

    void setEpsilon(real velocity_epsilon, real position_epsilon);

    void resolveContacts(Contact *contactArray, unsigned num_contacts, real duration);

protected:
    void prepareContacts(Contact *contact_array, unsigned num_contacts, real duration);

    void adjustVelocities(Contact *contact_array, unsigned num_contacts, real duration);

    void adjustPositions(Contact *contact_array, unsigned num_contacts, real duration);
};

class ContactGenerator {
public:
    virtual unsigned addContact(Contact *contact, unsigned limit) const = 0;
};


}