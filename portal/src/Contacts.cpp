#include <dolce/Contacts.h>
#include <memory.h>
#include <assert.h>

using namespace dolce;

void Contact::setBodyData(RigidBody *one, RigidBody *two, real friction, real restitution) {
    this->body[0] = one;
    this->body[1] = two;
    this->friction = friction;
    this->restitution = restitution;
}

void Contact::matchAwakeState() {
    // collisions with the world never cause a body to wake up
    if (!body[1]) {
        return;
    }

    bool body0awake = body[0]->getAwake();
    bool body1awake = body[1]->getAwake();

    // wake up the only sleeping one
    if (body0awake ^ body1awake) {
        if (body0awake)
            body[1]->setAwake(true);
        else 
            body[0]->setAwake(true);
    }
}

void Contact::swapBodies() {
    contact_normal *= -1;
    RigidBody *tmp = body[0];
    body[0] = body[1];
    body[1] = tmp;
}

inline void Contact::calculateContactBasis() {
    vec3 contact_tangent[2];

    if (fabs(contact_normal.x) > fabs(contact_normal.y)) {
        const real s = (real)1.0f / sqrt(contact_normal.z * contact_normal.z + contact_normal.x * contact_normal.x);

        contact_tangent[0].x = contact_normal.z * s;
        contact_tangent[0].y = 0;
        contact_tangent[0].z = -contact_normal.x * s;

        contact_tangent[1].x = contact_normal.y * contact_tangent[0].x;
        contact_tangent[1].y = contact_normal.z * contact_tangent[0].x - contact_normal.x * contact_tangent[0].z;
        contact_tangent[1].z = -contact_normal.y * contact_tangent[0].x;
    }
    else {
        const real s = (real)1.0f / sqrt(contact_normal.z * contact_normal.z + contact_normal.y * contact_normal.y);

        contact_tangent[0].x = 0;
        contact_tangent[0].y = -contact_normal.z * s;
        contact_tangent[0].z = contact_normal.y * s;

        contact_tangent[1].x = contact_normal.y * contact_tangent[0].z - contact_normal.z * contact_tangent[0].y;
        contact_tangent[1].y = -contact_normal.x * contact_tangent[0].z;
        contact_tangent[1].z = contact_normal.x = contact_tangent[0].y;
    }

    // not sure
    contact_to_world.setComponents(contact_normal, contact_tangent[0], contact_tangent[1]);
}

vec3 Contact::calculateLocalVelocity(unsigned body_index, real duration) {
    RigidBody *this_body = body[body_index];

    vec3 velocity = this_body->getRotation() % relative_contact_position[body_index];
    velocity += this_body->getVelocity();

    // not sure
    vec3 contact_velocity = contact_to_world.transformTranspose(velocity);

    vec3 acc_velocity = this_body->getLastFrameAcceleration() * duration;

    acc_velocity = contact_to_world.transformTranspose(acc_velocity);
    acc_velocity.x = 0;

    contact_velocity += acc_velocity;

    return contact_velocity;
}

void Contact::calculateDesiredDeltaVelocity(real duration) {
    const static real velocity_limit = (real)0.25f;

    real velocity_from_acc = 0;
    if (body[0]->getAwake()) {
        velocity_from_acc += body[0]->getLastFrameAcceleration() * duration * contact_normal;
    }

    if (body[1] && body[1]->getAwake()) {
        velocity_from_acc -= body[1]->getLastFrameAcceleration() * duration * contact_normal;
    }

    real this_restitution = restitution;
    if (fabs(contact_velocity.x) < velocity_limit) {
        this_restitution = (real)0.0f;
    }

    desired_delta_velocity = -contact_velocity.x - this_restitution * (contact_velocity.x - velocity_from_acc);
}

void Contact::calculateInternals(real duration) {
    if (!body[0])
        swapBodies();
    assert(body[0]);

    calculateContactBasis();

    relative_contact_position[0] = contact_point - body[0]->getPosition();
    if (body[1]) {
        relative_contact_position[1] = contact_point - body[1]->getPosition();
    }

    contact_velocity = calculateLocalVelocity(0, duration);
    if (body[1]) {
        contact_velocity -= calculateLocalVelocity(1, duration);
    }

    calculateDesiredDeltaVelocity(duration);
}

void Contact::applyVelocityChange(vec3 velocity_change[2], vec3 rotation_change[2]) {
    mat3 inverse_inertia_tensor[2];
    inverse_inertia_tensor[0] = body[0]->getInverseInertiaTensorWorld();
    if (body[1]) {
        inverse_inertia_tensor[1] = body[1]->getInverseInertiaTensorWorld();
    }
    vec3 impulse_contact;
    if (friction == (real)0.0)  {
        impulse_contact = calculateFrictionLessImpulse(inverse_inertia_tensor);
    }
    else {
        impulse_contact = calculateFrictionImpulse(inverse_inertia_tensor);
    }

    vec3 impulse = contact_to_world.transform(impulse_contact);

    vec3 impulsive_torque = relative_contact_position[0] % impulse;
    rotation_change[0] = inverse_inertia_tensor[0].transform(impulsive_torque);
    velocity_change[0].clear();
    velocity_change[0].addScaledVector(impulse, body[0]->getInverseMass());

    body[0]->setVelocity(body[0]->getVelocity() + velocity_change[0]);
    body[0]->setRotation(body[0]->getRotation() + rotation_change[0]);

    if (body[1]) {
        vec3 impulsive_torque = impulse % relative_contact_position[1];
        rotation_change[1] = inverse_inertia_tensor[1].transform(impulsive_torque);
        velocity_change[1].clear();
        velocity_change[1].addScaledVector(impulse, -body[1]->getInverseMass());

        body[1]->setVelocity(body[1]->getVelocity() + velocity_change[1]);
        body[1]->setRotation(body[1]->getRotation() + rotation_change[1]);
    }
}

inline vec3 Contact::calculateFrictionLessImpulse(mat3 *inverse_inertia_tensor) {
    vec3 impulse_contact;

    vec3 delta_vel_world = relative_contact_position[0] % contact_normal;
    delta_vel_world = inverse_inertia_tensor[0].transform(delta_vel_world);
    delta_vel_world = delta_vel_world % relative_contact_position[0];

    real delta_velocity = delta_vel_world * contact_normal;
    delta_velocity += body[0]->getInverseMass();

    if (body[1]) {
        vec3 delta_vel_world = relative_contact_position[1] % contact_normal;
        delta_vel_world = inverse_inertia_tensor[1].transform(delta_vel_world);
        delta_vel_world = delta_vel_world % relative_contact_position[1];

        delta_velocity += delta_vel_world * contact_normal;
        delta_velocity += body[1]->getInverseMass();
    }

    impulse_contact.x = desired_delta_velocity / delta_velocity;
    impulse_contact.y = 0;
    impulse_contact.z = 0;
    return impulse_contact;
}

inline vec3 Contact::calculateFrictionImpulse(mat3 *inverse_inertia_tensor) {
    vec3 impulse_contact;
    real inverse_mass = body[0]->getInverseMass();

    mat3 impulse_to_torque;
    impulse_to_torque.setSkewSymmetric(relative_contact_position[0]);

    mat3 delta_vel_world = impulse_to_torque;
    delta_vel_world *= inverse_inertia_tensor[0];
    delta_vel_world *= impulse_to_torque;
    delta_vel_world *= -1;

    if (body[1]) {
        impulse_to_torque.setSkewSymmetric(relative_contact_position[1]);

        mat3 delta_vel_world2 = impulse_to_torque;
        delta_vel_world2 *= inverse_inertia_tensor[1];
        delta_vel_world2 *= impulse_to_torque;
        delta_vel_world2 *= -1;

        delta_vel_world += delta_vel_world2;

        inverse_mass += body[1]->getInverseMass();
    }

    mat3 delta_velocity = contact_to_world.transpose();
    delta_velocity *= delta_vel_world;
    delta_velocity *= contact_to_world;

    delta_velocity.data[0] += inverse_mass;
    delta_velocity.data[4] += inverse_mass;
    delta_velocity.data[8] += inverse_mass;

    mat3 impulse_matrix = delta_velocity.inverse();

    vec3 vel_kill(desired_delta_velocity, -contact_velocity.y, -contact_velocity.z);

    impulse_contact = impulse_matrix.transform(vel_kill);

    real planar_impulse = sqrt(impulse_contact.y * impulse_contact.y + impulse_contact.z * impulse_contact.z);
    if (planar_impulse > impulse_contact.x * friction) {
        impulse_contact.y /= planar_impulse;
        impulse_contact.z /= planar_impulse;

        impulse_contact.x = delta_velocity.data[0] + delta_velocity.data[1] * friction * impulse_contact.y + delta_velocity.data[2] * friction * impulse_contact.z;
        impulse_contact.x = desired_delta_velocity / impulse_contact.x;
        impulse_contact.y *= friction * impulse_contact.x;
        impulse_contact.z *= friction * impulse_contact.x;
    }
    return impulse_contact;
}

void Contact::applyPositionChange(vec3 linear_change[2], vec3 angular_change[2], real penetration) {
    const real angular_limit = (real)0.2f;
    real angular_move[2];
    real linear_move[2];

    real total_inertia = 0;
    real linear_inertia[2];
    real angular_inertia[2];

    for (unsigned i = 0; i < 2; ++i) {
        if (body[i]) {
            mat3 inverse_inertia_tensor = body[i]->getInverseInertiaTensorWorld();
            
            vec3 angular_inertia_world = relative_contact_position[i] % contact_normal;
            angular_inertia_world = inverse_inertia_tensor.transform(angular_inertia_world);
            angular_inertia_world = angular_inertia_world % relative_contact_position[i];
            angular_inertia[i] = angular_inertia_world * contact_normal;

            linear_inertia[i] = body[i]->getInverseMass();

            total_inertia += linear_inertia[i] + angular_inertia[i];
        }
    }

    for (unsigned i = 0; i < 2; ++i) {
        if (body[i]) {
            real sign = (i == 0) ? 1 : -1;
            angular_move[i] = sign * penetration * (angular_inertia[i] / total_inertia);
            linear_move[i] = sign * penetration * (linear_inertia[i] / total_inertia);

            vec3 projection = relative_contact_position[i];
            projection.addScaledVector(contact_normal, -relative_contact_position[i].scalarProduct(contact_normal));

            real max_magnitude = angular_limit * projection.length();

            if (angular_move[i] < -max_magnitude) {
                real total_move = angular_move[i] + linear_move[i];
                angular_move[i] = -max_magnitude;
                linear_move[i] = total_move - angular_move[i];
            }
            else if (angular_move[i] > max_magnitude) {
                real total_move = angular_move[i] + linear_move[i];
                angular_move[i] = max_magnitude;
                linear_move[i] = total_move - angular_move[i];
            }

            if (angular_move[i] == 0) {
                angular_change[i].clear();
            }
            else {
                vec3 target_angular_direction = relative_contact_position[i] % contact_normal;

                mat3 inverse_inertia_tensor = body[i]->getInverseInertiaTensorWorld();

                angular_change[i] = inverse_inertia_tensor.transform(target_angular_direction) * (angular_move[i] / angular_inertia[i]);
            }

            linear_change[i] = contact_normal * linear_move[i];

            vec3 pos = body[i]->getPosition();
            pos.addScaledVector(contact_normal, linear_move[i]);
            body[i]->setPosition(pos);

            Quaternion q = body[i]->getOrientation();
            q.addScaledVector(angular_change[i], (real)1.0);
            body[i]->setOrientation(q);

            if (!body[i]->getAwake())
                body[i]->calculateDerivedData();
        }
    }
}

bool ContactResolver::isValid() {
    return (velocity_iterations > 0) && (position_iterations > 0) && (position_epsilon >= 0.0f) && (velocity_epsilon >= 0.0f);
}

// Contact resolver implementation

ContactResolver::ContactResolver(unsigned iterations, real velocity_epsilon, real position_epsilon) {
    setIterations(iterations, iterations);
    setEpsilon(velocity_epsilon, position_epsilon);
}

ContactResolver::ContactResolver(unsigned velocity_itearions, unsigned position_iterations, real velocity_epsilon, real position_epsilon) {
    setIterations(velocity_iterations, position_iterations);
    setEpsilon(velocity_epsilon, position_epsilon);
}

void ContactResolver::setIterations(unsigned iterations) {
    setIterations(iterations, iterations);
}

void ContactResolver::setIterations(unsigned velocity_iterations, unsigned position_iterations) {
    this->velocity_iterations = velocity_iterations;
    this->position_iterations = position_iterations;
}

void ContactResolver::setEpsilon(real velocity_epsilon, real position_epsilon) {
    this->velocity_epsilon = velocity_epsilon;
    this->position_epsilon = position_epsilon;
}

void ContactResolver::resolveContacts(Contact *contacts, unsigned num_contacts, real duration) {
    if (num_contacts == 0)
        return;
    if (!isValid())
        return;

    prepareContacts(contacts, num_contacts, duration);

    adjustPositions(contacts, num_contacts, duration);

    adjustVelocities(contacts, num_contacts, duration);
}

void ContactResolver::prepareContacts(Contact *contacts, unsigned num_contacts, real duration) {
    Contact *last_contact = contacts + num_contacts;
    for (Contact *contact = contacts; contact < last_contact; ++contact) {
        contact->calculateInternals(duration);
    }
}

void ContactResolver::adjustVelocities(Contact *c, unsigned num_contacts, real duration) {
    vec3 velocity_change[2], rotation_change[2];
    vec3 delta_vel;

    velocity_iterations_used = 0;
    while (velocity_iterations_used < velocity_iterations) {
        real max = velocity_epsilon;
        unsigned index = num_contacts;
        for (unsigned i = 0; i < num_contacts; ++i) {
            if (c[i].desired_delta_velocity > max) {
                max = c[i].desired_delta_velocity;
                index = i ;
            }
        }
        if (index == num_contacts)
            break;
        
        c[index].matchAwakeState();
        c[index].applyVelocityChange(velocity_change, rotation_change);

        for (unsigned i = 0; i < num_contacts; ++i) {
            for (unsigned b = 0; b < 2; ++b) {
                if (c[i].body[b]) {
                    for (unsigned d = 0; d < 2; ++d) {
                        if (c[i].body[b] == c[index].body[d]) {
                            delta_vel = velocity_change[d] + (rotation_change[d] % c[i].relative_contact_position[b]);

                            c[i].contact_velocity += c[i].contact_to_world.transformTranspose(delta_vel) * (b ? -1.0f : 1.0f);
                            c[i].calculateDesiredDeltaVelocity(duration);
                        }
                    }
                }
            }
        }
        ++velocity_iterations_used;
    } // end of while
}

void ContactResolver::adjustPositions(Contact *c, unsigned num_contacts, real duration) {
    // ?
    unsigned i, index;
    vec3 linear_change[2], angular_change[2];
    real max;
    vec3 delta_position;

    position_iterations_used = 0;
    while (position_iterations_used < position_iterations) {
        max = position_epsilon;
        index = num_contacts;
        for (i = 0; i < num_contacts; ++i) {
            if (c[i].penetration > max) {
                max = c[i].penetration;
                index = i;
            }
        }
        if (index == num_contacts)
            break;
        c[index].matchAwakeState();

        c[index].applyPositionChange(linear_change, angular_change, max);

        for (i = 0; i < num_contacts; ++i) {
            for (unsigned b = 0; b < 2; ++b) {
                if (c[i].body[b]) {
                    for (unsigned d = 0; d < 2; ++d) {
                        if (c[i].body[b] == c[index].body[d]) {
                            delta_position = linear_change[d] + (angular_change[d] % c[i].relative_contact_position[b]);

                            c[i].penetration += delta_position.scalarProduct(c[i].contact_normal) * (b ? 1 : -1);
                        }
                    }
                }
            }
        }
        ++position_iterations_used;
    }
}