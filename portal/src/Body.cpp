#include <dolce/Body.h>
#include <iostream>
#include <assert.h>

using namespace dolce;

/**
 * Internal function to do an intertia tensor transform by a quaternion.
 * Note that the implementation of this function was created by an
 * automated code-generator and optimizer.
 */
static inline void _transformInertiaTensor(mat3 &iitWorld,
                                           const Quaternion &q,
                                           const mat3 &iitBody,
                                           const mat4 &rotmat)
{
    real t4 = rotmat.data[0]*iitBody.data[0]+
        rotmat.data[1]*iitBody.data[3]+
        rotmat.data[2]*iitBody.data[6];
    real t9 = rotmat.data[0]*iitBody.data[1]+
        rotmat.data[1]*iitBody.data[4]+
        rotmat.data[2]*iitBody.data[7];
    real t14 = rotmat.data[0]*iitBody.data[2]+
        rotmat.data[1]*iitBody.data[5]+
        rotmat.data[2]*iitBody.data[8];
    real t28 = rotmat.data[4]*iitBody.data[0]+
        rotmat.data[5]*iitBody.data[3]+
        rotmat.data[6]*iitBody.data[6];
    real t33 = rotmat.data[4]*iitBody.data[1]+
        rotmat.data[5]*iitBody.data[4]+
        rotmat.data[6]*iitBody.data[7];
    real t38 = rotmat.data[4]*iitBody.data[2]+
        rotmat.data[5]*iitBody.data[5]+
        rotmat.data[6]*iitBody.data[8];
    real t52 = rotmat.data[8]*iitBody.data[0]+
        rotmat.data[9]*iitBody.data[3]+
        rotmat.data[10]*iitBody.data[6];
    real t57 = rotmat.data[8]*iitBody.data[1]+
        rotmat.data[9]*iitBody.data[4]+
        rotmat.data[10]*iitBody.data[7];
    real t62 = rotmat.data[8]*iitBody.data[2]+
        rotmat.data[9]*iitBody.data[5]+
        rotmat.data[10]*iitBody.data[8];

    iitWorld.data[0] = t4*rotmat.data[0]+
        t9*rotmat.data[1]+
        t14*rotmat.data[2];
    iitWorld.data[1] = t4*rotmat.data[4]+
        t9*rotmat.data[5]+
        t14*rotmat.data[6];
    iitWorld.data[2] = t4*rotmat.data[8]+
        t9*rotmat.data[9]+
        t14*rotmat.data[10];
    iitWorld.data[3] = t28*rotmat.data[0]+
        t33*rotmat.data[1]+
        t38*rotmat.data[2];
    iitWorld.data[4] = t28*rotmat.data[4]+
        t33*rotmat.data[5]+
        t38*rotmat.data[6];
    iitWorld.data[5] = t28*rotmat.data[8]+
        t33*rotmat.data[9]+
        t38*rotmat.data[10];
    iitWorld.data[6] = t52*rotmat.data[0]+
        t57*rotmat.data[1]+
        t62*rotmat.data[2];
    iitWorld.data[7] = t52*rotmat.data[4]+
        t57*rotmat.data[5]+
        t62*rotmat.data[6];
    iitWorld.data[8] = t52*rotmat.data[8]+
        t57*rotmat.data[9]+
        t62*rotmat.data[10];
}

/**
 * Inline function that creates a transform matrix from a
 * position and orientation.
 */
static inline void _calculateTransformMatrix(mat4 &transformMatrix,
                                             const vec3 &position,
                                             const Quaternion &orientation)
{
    transformMatrix.data[0] = 1-2*orientation.j*orientation.j-
        2*orientation.k*orientation.k;
    transformMatrix.data[1] = 2*orientation.i*orientation.j -
        2*orientation.r*orientation.k;
    transformMatrix.data[2] = 2*orientation.i*orientation.k +
        2*orientation.r*orientation.j;
    transformMatrix.data[3] = position.x;

    transformMatrix.data[4] = 2*orientation.i*orientation.j +
        2*orientation.r*orientation.k;
    transformMatrix.data[5] = 1-2*orientation.i*orientation.i-
        2*orientation.k*orientation.k;
    transformMatrix.data[6] = 2*orientation.j*orientation.k -
        2*orientation.r*orientation.i;
    transformMatrix.data[7] = position.y;

    transformMatrix.data[8] = 2*orientation.i*orientation.k -
        2*orientation.r*orientation.j;
    transformMatrix.data[9] = 2*orientation.j*orientation.k +
        2*orientation.r*orientation.i;
    transformMatrix.data[10] = 1-2*orientation.i*orientation.i-
        2*orientation.j*orientation.j;
    transformMatrix.data[11] = position.z;
}

void RigidBody::calculateDerivedData() {
    // not sure
    orientation.normalize();

    _calculateTransformMatrix(transform_matrix, position, orientation);
    _transformInertiaTensor(inverse_inertia_tensor_world, orientation, inverse_inertia_tensor, transform_matrix);
}

void RigidBody::integrate(real duration) {
    if (!is_awake)
        return;

    last_frame_acceleration = acceleration;
    last_frame_acceleration += force_accum * inverse_mass;

    vec3 angular_acceleration = inverse_inertia_tensor_world.transform(torque_accum);

    velocity += last_frame_acceleration * duration;
    rotation += angular_acceleration * duration;

    velocity *= pow(linear_damping, duration);
    rotation *= pow(angular_damping, duration);

    position += velocity * duration;
    orientation.addScaledVector(rotation, duration);

    calculateDerivedData();
    clearAccumulators();

    if (can_sleep) {
        real current_motion = velocity.scalarProduct(velocity) + rotation.scalarProduct(rotation);

        real bias = pow(0.5, duration);
        motion = bias * motion + (1 - bias) * current_motion;

        if (motion < sleep_epsilon)
            setAwake(false);
        else if (motion > 10 * sleep_epsilon) 
            motion = 10 * sleep_epsilon;
    }
}

void RigidBody::clearAccumulators() {
    force_accum.clear();
    torque_accum.clear();
}

void RigidBody::setMass(const real mass) {
    assert(mass != 0);
    this->inverse_mass = (real)1.0 / mass;
}

real RigidBody::getMass() const {
    if (inverse_mass == 0) {
        return REAL_MAX;
    }
    else {
        return (real)1.0 / inverse_mass;
    }
}

void RigidBody::setInverseMass(const real inverse_mass) {
    this->inverse_mass = inverse_mass;
}

real RigidBody::getInverseMass() const {
    return inverse_mass;
}
bool RigidBody::hasFiniteMass() const {
    return inverse_mass >= 0.0f;
}

void RigidBody::setInertiaTensor(const mat3 &inertia_tensor) {
    this->inverse_inertia_tensor.setInverse(inertia_tensor);
    // _checkInverseInertiaTensor(inverse_inertia_tensor);
}

mat3 RigidBody::getInertiaTensor() const {
    mat3 it;
    it.setInverse(this->inverse_inertia_tensor);
    return it;
}

mat3 RigidBody::getInertiaTensorWorld() const {
    mat3 it;
    it.setInverse(inverse_inertia_tensor_world);
    return it;
}

void RigidBody::setInverseInertiaTensor(const mat3 &inverse_inertia_tensor) {
    // _checkInverseInertiaTensor(inverse_inertia_tensor);
    this->inverse_inertia_tensor = inverse_inertia_tensor;
}

mat3 RigidBody::getInverseInertiaTensor() const {
    return this->inverse_inertia_tensor;
}

mat3 RigidBody::getInverseInertiaTensorWorld() const {
    return this->inverse_inertia_tensor_world;
}

void RigidBody::setDamping(const real linear_damping, const real angular_damping) {
    this->linear_damping = linear_damping;
    this->angular_damping = angular_damping;
}

void RigidBody::setLinearDamping(const real linear_damping) {
    this->linear_damping = linear_damping;
}

void RigidBody::setAngularDamping(const real angular_damping) {
    this->angular_damping = angular_damping;
}

real RigidBody::getLinearDamping() const {
    return this->linear_damping;
}

real RigidBody::getAngularDamping() const {
    return this->angular_damping;
}

void RigidBody::setPosition(const vec3 &position) {
    this->position = position;
}

void RigidBody::setVelocity(const vec3 &velocity) {
    this->velocity = velocity;
}

void RigidBody::setAcceleration(const vec3 &acceleration) {
    this->acceleration = acceleration;
}

void RigidBody::setRotation(const vec3 &rotation) {
    this->rotation = rotation;
}

void RigidBody::setOrientation(const Quaternion &orientation) {
    this->orientation = orientation;
    this->orientation.normalize();
}

vec3 RigidBody::getPosition() const {
    return this->position;
}

vec3 RigidBody::getVelocity() const {
    return this->velocity;
}

vec3 RigidBody::getAcceleration() const {
    return this->acceleration;
}

vec3 RigidBody::getRotation() const {
    return this->rotation;
}

Quaternion RigidBody::getOrientation() const {
    return this->orientation;
}

mat4 RigidBody::getTransform() const {
    return this->transform_matrix;
}

vec3 RigidBody::getLastFrameAcceleration() const {
    return last_frame_acceleration;
}

void RigidBody::addRotation(const vec3 &delta) {
    rotation += delta;
}

void RigidBody::addTorque(const vec3 &torque) {
    torque_accum += torque;
    is_awake = true;
}

void RigidBody::addForce(const vec3 &force) {
    this->force_accum += force;
    is_awake = true;
}

void RigidBody::addForceAtBodyPoint(const vec3 &force, const vec3 &point) {
    vec3 pt = getPointInWorldSpace(point);
    addForceAtPoint(force, pt);
}

void RigidBody::addForceAtPoint(const vec3 &force, const vec3 &point) {
    vec3 pt = point;
    pt -= position;

    force_accum += force;
    torque_accum += pt % force;

    is_awake = true;
}


vec3 RigidBody::getPointInLocalSpace(const vec3 &point) {
    return transform_matrix.transformInverse(point);
}

vec3 RigidBody::getPointInWorldSpace(const vec3 &point) {
    return transform_matrix.transform(point);
}

vec3 RigidBody::getDirectionInLocalSpace(const vec3 &direction) {
    return transform_matrix.transformInverseDirection(direction);
}

vec3 RigidBody::getDirectionInWorldSpace(const vec3 &direction) {
    return transform_matrix.transformDirection(direction);
}


void RigidBody::setAwake(const bool awake) {
    if (awake) {
        is_awake = true;
        this->motion = sleep_epsilon * 2.0f;
    }
    else {
        is_awake = false;
        velocity.clear();
        rotation.clear();
    }
}

bool RigidBody::getAwake() const {
    return this->is_awake;
}

void RigidBody::setCanSleep(const bool can_sleep) {
    this->can_sleep = can_sleep;
    if (!can_sleep & !is_awake)
        setAwake(true);
}