#include <dolce/CollideFine.h>
#include <memory.h>
#include <assert.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>

using namespace dolce;

vec3 CollisionPrimitive::getAxis(unsigned index) const {
    return transform.getAxisVector(index);
}

const mat4& CollisionPrimitive::getTransform() const {
    return this->transform;
}

bool CollisionData::hasMoreContacts() const {
    return contacts_left > 0;
}

void CollisionData::reset(unsigned max_contacts) {
    contacts_left = max_contacts;
    contact_count = 0;
    contacts = contact_array;
}

void CollisionData::addContacts(unsigned count) {
    contacts_left -= count;
    contact_count += count;
    contacts += count;
}

void CollisionPrimitive::calculateInternals() {
    transform = body->getTransform() * offset;
}

bool IntersectionTests::sphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane) {
    real ball_distance = plane.direction * sphere.getAxis(3) - sphere.radius;
    return ball_distance <= plane.offset;
}

bool IntersectionTests::sphereAndSphere(const CollisionSphere &one, const CollisionSphere &two) {
    vec3 midline = one.getAxis(3) - two.getAxis(3);
    return midline.squared_length() < (one.radius + two.radius) * (one.radius + two.radius);
}

static inline real transformToAxis(const CollisionBox &box, const vec3 &axis) {
    return box.halfsize.x * fabs(axis * box.getAxis(0)) + box.halfsize.y * fabs(axis * box.getAxis(1)) + box.halfsize.z * fabs(axis * box.getAxis(2));
}

static inline bool overlapOnAxis(const CollisionBox &one, const CollisionBox &two, const vec3 &axis, const vec3 &to_centre) {
    real one_project = transformToAxis(one, axis);
    real two_project = transformToAxis(two, axis);
    real distance = fabs(to_centre * axis);

    return (distance < one_project + two_project);
}

#define TEST_OVERLAP(axis) overlapOnAxis(one, two, (axis), to_centre)

bool IntersectionTests::boxAndBox(const CollisionBox &one, const CollisionBox &two) {
    vec3 to_centre = two.getAxis(3) - one.getAxis(3);

    return (
        // check on box one's axes first
        TEST_OVERLAP(one.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(2)) &&
        // And on two's
        TEST_OVERLAP(two.getAxis(0)) &&
        TEST_OVERLAP(two.getAxis(1)) &&
        TEST_OVERLAP(two.getAxis(0)) &&
        // Now on the cross products
        TEST_OVERLAP(one.getAxis(0) % two.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(0) % two.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(0) % two.getAxis(2)) &&
        TEST_OVERLAP(one.getAxis(1) % two.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(1) % two.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(1) % two.getAxis(2)) &&
        TEST_OVERLAP(one.getAxis(2) % two.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(2) % two.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(2) % two.getAxis(2))
    );
}

#undef TEST_OVERLAP

bool IntersectionTests::boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane) {
    real projected_radius = transformToAxis(box, plane.direction);
    real box_distance = plane.direction * box.getAxis(3) - projected_radius;
    return box_distance <= plane.offset;
}

unsigned CollisionDetector::sphereAndTruePlane(const CollisionSphere &sphere, const CollisionPlane &plane, CollisionData *data) {
    if (data->contacts_left <= 0)
        return 0;
    vec3 position = sphere.getAxis(3);
    real centre_distance = plane.direction * position - plane.offset;

    if (centre_distance * centre_distance > sphere.radius * sphere.radius) {
        return 0;
    }

    vec3 normal = plane.direction;
    real penetration = -centre_distance;
    if (centre_distance < 0) {
        normal *= -1;
        penetration = -penetration;
    }
    penetration += sphere.radius;

    Contact *contact = data->contacts;
    contact->contact_normal = normal;
    contact->penetration = penetration;
    contact->contact_point = position - plane.direction * centre_distance;
    contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::sphereAndHalfSpace(const CollisionSphere &sphere, const CollisionPlane &plane, CollisionData *data) {
    if (data->contacts_left <= 0)
        return 0;
    vec3 position = sphere.getAxis(3);

    real ball_distance = plane.direction * position - sphere.radius - plane.offset;

    if (ball_distance >= 0)
        return 0;

    Contact *contact = data->contacts;
    contact->contact_normal = plane.direction;
    contact->penetration = -ball_distance;
    contact->contact_point = position - plane.direction * (ball_distance + sphere.radius);
    contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);
    
    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::sphereAndSphere(const CollisionSphere &one, const CollisionSphere &two, CollisionData *data) {
    if (data->contacts_left <= 0) {
        return 0;
    }

    vec3 position_one = one.getAxis(3);
    vec3 position_two = two.getAxis(3);

    vec3 midline = position_one - position_two;
    real size = midline.length();

    if (size <= 0.0f || size >= one.radius + two.radius) {
        return 0;
    }

    vec3 normal = midline * ((real)(1.0f) / size);
    Contact *contact = data->contacts;
    contact->contact_normal = normal;
    contact->contact_point  = position_one + midline * ((real)0.5);
    contact->penetration = (one.radius + two.radius - size);
    contact->setBodyData(one.body, two.body, data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

static inline real penetrationOnAxis(const CollisionBox &one, const CollisionBox &two, const vec3 &axis, const vec3 &to_centre) {
    real one_project = transformToAxis(one, axis);
    real two_project = transformToAxis(two, axis);

    real distance = fabs(to_centre * axis);

    return one_project + two_project - distance;
}

static inline bool tryAxis(const CollisionBox &one, const CollisionBox &two, vec3 axis, const vec3 &to_centre, unsigned index, real &smallest_penetration, unsigned &smallest_case) {
    if (axis.squared_length() < 0.0001)
        return true;
    axis.normalize();

    real penetration = penetrationOnAxis(one, two, axis, to_centre);
    if (penetration < 0) {
        return false;
    }
    if (penetration < smallest_penetration) {
        smallest_penetration = penetration;
        smallest_case = index;
    }
    return true;
}

void fillPointFaceBoxBox(const CollisionBox &one, const CollisionBox &two, const vec3 &to_centre, CollisionData *data, unsigned best, real pen) {
    Contact *contact = data->contacts;
    vec3 normal = one.getAxis(best);
    if (one.getAxis(best) * to_centre > 0) {
        normal = normal * -1.0f;
    }

    vec3 vertex = two.halfsize;
    if (two.getAxis(0) * normal < 0)
        vertex.x = -vertex.x;
    if (two.getAxis(1) * normal < 0)
        vertex.y = -vertex.y;
    if (two.getAxis(2) * normal < 0)
        vertex.z = -vertex.z;
    
    contact->contact_normal = normal;
    contact->penetration = pen;
    contact->contact_point = two.getTransform() * vertex;
    contact->setBodyData(one.body, two.body, data->friction, data->restitution);
}

static inline vec3 contactPoint(const vec3 &p_one, const vec3 &d_one, real one_size, const vec3 &p_two, const vec3 &d_two, real two_size, bool use_one) {
    vec3 to_st, c_one, c_two;
    real dp_sta_one, dp_sta_two, dp_one_two, sm_one, sm_two;
    real denom, mua, mub;

    sm_one = d_one.squared_length();
    sm_two = d_two.squared_length();
    dp_one_two = d_two * d_one;

    to_st = p_one - p_two;
    dp_sta_one = d_one * to_st;
    dp_sta_two = d_two * to_st;

    denom = sm_one * sm_two - dp_one_two * dp_one_two;

    if (fabs(denom) < 0.0001f) {
        return use_one ? p_one : p_two;
    }

    mua = (dp_one_two * dp_sta_two - sm_two * dp_sta_one) / denom;
    mub = (sm_one * dp_sta_two - dp_one_two * dp_sta_one) / denom;

    if (mua > one_size || mua < -one_size || mub > two_size || mub < -two_size) {
        return use_one ? p_one : p_two;
    }
    else {
        c_one = p_one + d_one * mua;
        c_two = p_two + d_two * mub;
        return c_one * 0.5f + c_two * 0.5f;
    }
}

#define CHECK_OVERLAP(axis, index) \
    if (!tryAxis(one, two, (axis), to_centre, (index), pen, best)) return 0;

unsigned CollisionDetector::boxAndBox(const CollisionBox &one, const CollisionBox &two, CollisionData *data) {
    vec3 to_centre = two.getAxis(3) - one.getAxis(3);

    real pen = REAL_MAX;
    unsigned best = 0xffffff;

    CHECK_OVERLAP(one.getAxis(0), 0);
    CHECK_OVERLAP(one.getAxis(1), 1);
    CHECK_OVERLAP(one.getAxis(2), 2);

    CHECK_OVERLAP(two.getAxis(0), 0);
    CHECK_OVERLAP(two.getAxis(1), 1);
    CHECK_OVERLAP(two.getAxis(2), 2);

    unsigned best_single_axis = best;
    CHECK_OVERLAP(one.getAxis(0) * two.getAxis(0), 6);
    CHECK_OVERLAP(one.getAxis(0) * two.getAxis(1), 7);
    CHECK_OVERLAP(one.getAxis(0) * two.getAxis(2), 8);
    CHECK_OVERLAP(one.getAxis(1) * two.getAxis(0), 9);
    CHECK_OVERLAP(one.getAxis(1) * two.getAxis(1), 10);
    CHECK_OVERLAP(one.getAxis(1) * two.getAxis(2), 11);
    CHECK_OVERLAP(one.getAxis(2) * two.getAxis(0), 12);
    CHECK_OVERLAP(one.getAxis(2) * two.getAxis(1), 13);
    CHECK_OVERLAP(one.getAxis(2) * two.getAxis(2), 14);

    assert(best != 0xffffff);

    if (best < 3) {
        fillPointFaceBoxBox(one, two, to_centre, data, best, pen);
        data->addContacts(1);
        return 1;
    }
    else if (best < 6) {
        fillPointFaceBoxBox(two, one, to_centre * -1.0f, data, best - 3, pen);
        data->addContacts(1);
        return 1;
    }
    else {
        best -= 6;
        unsigned one_axis_index = best / 3;
        unsigned two_axis_index = best % 3;
        vec3 one_axis = one.getAxis(one_axis_index);
        vec3 two_axis = two.getAxis(two_axis_index);
        vec3 axis = one_axis % two_axis;
        axis.normalize();

        if (axis * to_centre > 0)
            axis = axis * -1.0f;

        vec3 pt_on_one_edge = one.halfsize;
        vec3 pt_on_two_edge = two.halfsize;
        for (unsigned i = 0; i < 3; ++i) {
            if (i == one_axis_index) 
                pt_on_one_edge[i] = 0;
            else if (one.getAxis(i) * axis > 0)
                pt_on_one_edge[i] = -pt_on_one_edge[i];

            if (i == two_axis_index)
                pt_on_two_edge[i] = 0;
            else if (two.getAxis(i) * axis < 0)
                pt_on_two_edge[i] = -pt_on_two_edge[i];
        }

        pt_on_one_edge = one.transform * pt_on_one_edge;
        pt_on_two_edge = two.transform * pt_on_two_edge;
        
        vec3 vertex = contactPoint(pt_on_one_edge, one_axis, one.halfsize[one_axis_index], pt_on_two_edge, two_axis, two.halfsize[two_axis_index], best_single_axis > 2);
        
        Contact *contact = data->contacts;

        contact->penetration = pen;
        contact->contact_normal = axis;
        contact->contact_point = vertex;
        contact->setBodyData(one.body, two.body, data->friction, data->restitution);
        data->addContacts(1);
        return 1;
    }
    return 0;
}
#undef CHECK_OVERLAP

unsigned CollisionDetector::boxAndPoint(const CollisionBox &box, const vec3 &point, CollisionData *data) {
    vec3 rel_pt = box.transform.transformInverse(point);
    vec3 normal;
    real min_depth = box.halfsize.x - fabs(rel_pt.x);
    if (min_depth < 0)
        return 0;
    normal = box.getAxis(0) * (rel_pt.x < 0 ? -1.0f : 1.0f);

    real depth = box.halfsize.y - fabs(rel_pt.y);
    if (depth < 0) {
        return 0;
    }
    else if (depth < min_depth) {
        min_depth = depth;
        normal = box.getAxis(1) * (rel_pt.y < 0 ? -1.0f : 1.0f);
    }

    depth = box.halfsize.z - fabs(rel_pt.z);
    if (depth < 0)
        return 0;
    else if (depth < min_depth) {
        min_depth = depth;
        normal = box.getAxis(2) * (rel_pt.z < 0 ? -1.0f : 1.f);
    }

    Contact *contact = data->contacts;
    contact->contact_normal = normal;
    contact->contact_point = point;
    contact->penetration = min_depth;
    contact->setBodyData(box.body, NULL, data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::boxAndSphere(const CollisionBox &box, const CollisionSphere &sphere, CollisionData *data) {
    vec3 centre = sphere.getAxis(3);
    vec3 rel_centre = box.transform.transformInverse(centre);

    if (fabs(rel_centre.x) - sphere.radius > box.halfsize.x || fabs(rel_centre.y) - sphere.radius > box.halfsize.y || fabs(rel_centre.z) - sphere.radius > box.halfsize.z) {
        return 0;
    }

    vec3 closest_pt(0, 0, 0);
    real dist;

    dist = rel_centre.x;
    if (dist > box.halfsize.x) 
        dist = box.halfsize.x;
    if (dist < -box.halfsize.x)
        dist = -box.halfsize.x;
    closest_pt.x = dist;

    dist = rel_centre.y;
    if (dist > box.halfsize.y) 
        dist = box.halfsize.y;
    if (dist < -box.halfsize.y)
        dist = -box.halfsize.y;
    closest_pt.y = dist;

    dist = rel_centre.z;
    if (dist > box.halfsize.z) 
        dist = box.halfsize.z;
    if (dist < -box.halfsize.z)
        dist = -box.halfsize.z;
    closest_pt.z = dist;

    dist = (closest_pt - rel_centre).squared_length();
    if (dist > sphere.radius * sphere.radius)
        return 0;
    
    vec3 closest_pt_world = box.transform.transform(closest_pt);

    Contact* contact = data->contacts;
    contact->contact_normal = (closest_pt_world - centre);
    contact->contact_normal.normalize();
    contact->contact_point = closest_pt_world;
    contact->penetration = sphere.radius - fabs(dist);
    contact->setBodyData(box.body, sphere.body, data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane, CollisionData *data) {
    if (data->contacts_left <= 0)
        return 0;

    if (!IntersectionTests::boxAndHalfSpace(box, plane)) {
        return 0;
    }

    const static real mults[8][3] = {
        {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1},
        {1, 1, -1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, -1}
    };

    Contact *contact = data->contacts;
    unsigned contacts_used = 0;
    for (unsigned i = 0; i < 8; ++i) {
        vec3 vertex_pos(mults[i][0], mults[i][1], mults[i][2]);
        vertex_pos.componentProductUpdate(box.halfsize);
        vertex_pos = box.transform.transform(vertex_pos);

        real vertex_distance = vertex_pos * plane.direction;

        if (vertex_distance <= plane.offset) {
            contact->contact_point = plane.direction;
            contact->contact_point *= (vertex_distance - plane.offset);
            contact->contact_point += vertex_pos;
            contact->contact_normal = plane.direction;
            contact->penetration = plane.offset - vertex_distance;

            contact->setBodyData(box.body, NULL, data->friction, data->restitution);

            ++contact;
            ++contacts_used;
            if (contacts_used == (unsigned)data->contacts_left){
                // data->addContacts(contacts_used) // not use ?
                return contacts_used;
            } 
        }
    }
    data->addContacts(contacts_used);
    return contacts_used;
}