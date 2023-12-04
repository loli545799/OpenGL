#pragma once
#include "mathematics.h"

namespace dolce {
class RigidBody {
protected:
    real inverse_mass;
    vec3 position;
    Quaternion orientation;
    vec3 velocity;
    vec3 rotation;
    vec3 acceleration;
    vec3 last_frame_acceleration;

    mat4 transform_matrix;
    mat3 inverse_inertia_tensor;
    mat3 inverse_inertia_tensor_world;

    real linear_damping;
    real angular_damping;
    
    real motion;
    bool is_awake;
    bool can_sleep;

    vec3 force_accum;
    vec3 torque_accum;
    
public:
    void calculateDerivedData();

    void setInertiaTensor(const mat3 &inertia_tensor);
    mat3 getInertiaTensor() const;
    mat3 getInertiaTensorWorld() const;
    void setInverseInertiaTensor(const mat3 &inverse_inertia_tensor);
    mat3 getInverseInertiaTensor() const;
    mat3 getInverseInertiaTensorWorld() const;

    void setDamping(const real linear_damping, const real angular_damping);
    void setLinearDamping(const real linear_damping);
    void setAngularDamping(const real angular_damping);
    real getLinearDamping() const;
    real getAngularDamping() const;

    void addRotation(const vec3 &delta);
    void addTorque(const vec3 &torque);
    void addForce(const vec3 &force);
    void addForceAtBodyPoint(const vec3 &force, const vec3 &point);
    void addForceAtPoint(const vec3 &force, const vec3 &point);

    void setAwake(const bool awake = true);
    bool getAwake() const;
    void setCanSleep(const bool can_sleep);

    void integrate(const real duration);
    void clearAccumulators();

    void setMass(const real mass);
    real getMass() const;
    void setInverseMass(const real inverse_mass);
    real getInverseMass() const;
    bool hasFiniteMass() const;

    void setPosition(const vec3 &position);
    void setVelocity(const vec3 &velocity);
    void setAcceleration(const vec3 &acceleration);
    void setRotation(const vec3 &rotation);
    void setOrientation(const Quaternion &orientation);
    vec3 getPosition() const;
    vec3 getVelocity() const;
    vec3 getAcceleration() const;
    vec3 getRotation() const;
    Quaternion getOrientation() const;

    mat4 getTransform() const;
    vec3 getLastFrameAcceleration() const;

    vec3 getPointInLocalSpace(const vec3 &point);
    vec3 getPointInWorldSpace(const vec3 &point);
    vec3 getDirectionInLocalSpace(const vec3 &direction);
    vec3 getDirectionInWorldSpace(const vec3 &direction);
};

}