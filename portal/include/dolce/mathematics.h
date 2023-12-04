#pragma once
#include "precision.h"
#include <math.h>

namespace dolce {

extern real sleep_epsilon;
// void setSleepEpsilon(real value);
// real getSleepEpsilon();

class vec3 {

public:
    real x;
    real y;
    real z;

public:
    vec3(const real x = 0, const real y = 0, const real z = 0) 
        : x(x), y(y), z(z) {}

    // const static vec3 GRAVITY;
    // const static vec3 UP;
    // const static vec3 RIGHT;
    // const static vec3 X;
    // const static vec3 Y;
    // const static vec3 Z;

    real operator[](unsigned i) const {
        switch (i) {
            case 0:
                return x;
            case 1:
                return y;
            default:
                return z;
        }
    }

    real& operator[](unsigned i) {
        switch(i) {
            case 0:
                return x;
            case 1:
                return y;
            default:
                return z;
        }
    }

    vec3& operator+=(const vec3 &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    vec3 operator+(const vec3 &v) const {
        return vec3(x + v.x, y + v.y, z + v.z);
    }

    vec3& operator-=(const vec3 &v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    vec3 operator-(const vec3 &v) const {
        return vec3(x - v.x, y - v.y, z - v.z);
    }

    vec3& operator*=(const real value) {
        x *= value;
        y *= value;
        z *= value;
        return *this;
    }

    vec3 operator*(const real value) const {
        return vec3(x * value, y * value, z * value);
    }

    vec3 componentProduct(const vec3 &v) const {
        return vec3(x * v.x, y * v.y, z * v.z);
    }

    void componentProductUpdate(const vec3 &v) {
        x *= v.x;
        y *= v.y;
        z *= v.z;
    }

    vec3& operator%=(const vec3 &v) {
        *this = cross(*this, v);
        return *this;
    }

    vec3 operator%(const vec3 &v) const {
        return vec3(y * v.z - z * v.y,
                    z * v.x - x * v.z,
                    x * v.y - y * v.x);
    }

    static vec3 cross(const vec3 &v1, const vec3 &v2) {
        return vec3(v1.y * v2.z - v1.z * v2.y,
                    v1.z * v2.x - v1.x * v2.z,
                    v1.x * v2.y - v1.y * v2.x);
    }

    real scalarProduct(const vec3 &v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    real operator*(const vec3 &v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    void addScaledVector(const vec3 &v, real scale) {
        x += v.x * scale;
        y += v.y * scale;
        z += v.z * scale;
    }

    real length() const {
        return fabs(x * x + y * y + z * z);
    }

    real squared_length() const {
        return x * x + y * y + z * z;
    }

    void trim(real size) {
        if (squared_length() > size * size) {
            normalize();
            x *= size;
            y *= size;
            z *= size;
        }
    }

    void normalize() {
        real len = length();
        if (len > 0) {
            (*this) *= (real)1 / len;
        }
    }

    vec3 unit() {
        vec3 res = *this;
        res.normalize();
        return res;
    }

    bool operator==(const vec3& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const vec3& other) const {
        return !(*this == other);
    }

    void clear() {
        x = y = z = 0;
    }

    void invert() {
        x = -x;
        y = -y;
        z = -z;
    }
};

class Quaternion {
public:
    union {
        struct {
            // real component of the quaternion
            real r;
            // three components of the quaternion
            real i;
            real j;
            real k;
        };
        real data[4];
    };

    Quaternion() : r(1), i(0), j(0), k(0) {}

    Quaternion(const real r, const real i, const real j, const real k) 
        : r(r), i(i), j(j), k(k) {}

    void normalize() {
        real d = r * r + i * i + j * j + k * k;
        if (d < REAL_EPSILON) {
            r = 1;
            return;
        }

        d = (real)1.0 / sqrt(d);
        r *= d;
        i *= d;
        j *= d;
        k *= d;
    }

    void operator*=(const Quaternion &multiplier) {
        Quaternion q = *this;
        r = q.r * multiplier.r - q.i * multiplier.i -
            q.j * multiplier.j - q.k * multiplier.k;
        i = q.r * multiplier.i + q.i * multiplier.r +
            q.j * multiplier.k - q.k * multiplier.j;
        j = q.r * multiplier.j + q.j * multiplier.r + 
            q.k * multiplier.i - q.i * multiplier.k;
        k = q.r * multiplier.k + q.k * multiplier.r +
            q.i * multiplier.j - q.j * multiplier.i;
    }

    void addScaledVector(const vec3 &v, real scale) {
        Quaternion q(0, v.x * scale, v.y * scale, v.z * scale);
        q *= *this;
        r += q.r * (real)0.5;
        i += q.i * (real)0.5;
        j += q.j * (real)0.5;
        k += q.k * (real)0.5;
    }

    void rotateByVector(const vec3 &v) {
        Quaternion q(0, v.x, v.y, v.z);
        (*this) *= q;
    }

    Quaternion conjugate() const {
        return Quaternion(r, -i, -j, -k);
    }

    void rotate(const dolce::vec3 &axis, dolce::real angle) {
        constexpr static dolce::real PI = 3.1415926;
        dolce::real radius = angle / 2 * PI / 180;
        Quaternion q(cos(radius), sin(radius) * axis.x, sin(radius) * axis.y, sin(radius) * axis.z);
        q *= (*this);
        *this = q;
    }
};

class mat4 {
public:
    real data[12];

    mat4() {
        // initialize as identity matrix
        data[0] = 1;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;
        data[4] = 0;
        data[5] = 1;
        data[6] = 0;
        data[7] = 0;
        data[8] = 0;
        data[9] = 0;
        data[10] = 1;
        data[11] = 0;
    }

    void setDiagonal(real a, real b, real c) {
        data[0] = a;
        data[5] = b;
        data[10] = c;
    }

    mat4 operator*(const mat4 &o) const {
        mat4 res;
        res.data[0] = (o.data[0] * data[0]) + (o.data[4] * data[1]) + (o.data[8] * data[2]);
        res.data[4] = (o.data[0] * data[4]) + (o.data[4] * data[5]) + (o.data[8] * data[6]);
        res.data[8] = (o.data[0] * data[8]) + (o.data[4] * data[9]) + (o.data[8] * data[10]);

        res.data[1] = (o.data[1] * data[0]) + (o.data[5] * data[1]) + (o.data[9] * data[2]);
        res.data[5] = (o.data[1] * data[4]) + (o.data[5] * data[5]) + (o.data[9] * data[6]);
        res.data[9] = (o.data[1] * data[8]) + (o.data[5] * data[9]) + (o.data[9] * data[10]);

        res.data[2] = (o.data[2] * data[0]) + (o.data[6] * data[1]) + (o.data[10] * data[2]);
        res.data[6] = (o.data[2] * data[4]) + (o.data[6] * data[5]) + (o.data[10] * data[6]);
        res.data[10] = (o.data[2] * data[8]) + (o.data[6] * data[9]) + (o.data[10] * data[10]);

        res.data[3] = (o.data[3] * data[0]) + (o.data[7] * data[1]) + (o.data[11] * data[2]) + data[3];
        res.data[7] = (o.data[3] * data[4]) + (o.data[7] * data[5]) + (o.data[11] * data[6]) + data[7];
        res.data[11] = (o.data[3] * data[8]) + (o.data[7] * data[9]) + (o.data[11] * data[10]) + data[11];

        return res;
    }

    vec3 operator*(const vec3 &v) const {
        return vec3(
            v.x * data[0] + v.y * data[1] + v.z * data[2] + data[3],
            v.x * data[4] + v.y * data[5] + v.z * data[6] + data[7],
            v.x * data[8] + v.y * data[9] + v.z * data[10] + data[11]
        );
    }

    vec3 transform(const vec3 &v) const {
        return (*this) * v;
    }

    real getDeterminant() const;

    void setInverse(const mat4 &m);

    mat4 inverse() const {
        mat4 res;
        res.setInverse(*this);
        return res;
    }

    void invert() {
        setInverse(*this);
    }

    vec3 transformDirection(const vec3 &v) const {
        return vec3(v.x * data[0] + v.y * data[1] + v.z * data[2],
                    v.x * data[4] + v.y * data[5] + v.z * data[6],
                    v.x * data[8] + v.y * data[9] + v.z * data[10]);
    }

    vec3 transformInverseDirection(const vec3 &v) const {
        return vec3(v.x * data[0] + v.y * data[4] + v.z * data[8],
                    v.x * data[1] + v.y * data[5] + v.z * data[9],
                    v.x * data[2] + v.y * data[6] + v.z * data[10]);
    }

    vec3 transformInverse(const vec3 &v) const {
        vec3 tmp = v;
        tmp.x -= data[3];
        tmp.y -= data[7];
        tmp.z -= data[11];
        return vec3(tmp.x * data[0] + tmp.y * data[4] + tmp.z * data[8],
                    tmp.x * data[1] + tmp.y * data[5] + tmp.z * data[9],
                    tmp.x * data[2] + tmp.y * data[6] + tmp.z * data[10]);
    }

    vec3 getAxisVector(int i) const {
        return vec3(data[i], data[i + 4], data[i + 8]);
    }

    void setOrientationAndPos(const Quaternion &q, const vec3 &v) {
        data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
        data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
        data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
        data[3] = v.x;

        data[4] = 2 * q.i * q.j - 2 * q.k * q.r;
        data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
        data[6] = 2 * q.j * q.k + 2 * q.i * q.r;
        data[7] = v.y;

        data[8] = 2 * q.i * q.k + 2 * q.j * q.r;
        data[9] = 2 * q.j * q.k - 2 * q.i * q.r;
        data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
        data[11] = v.z;
    }

    void fillGLArray4x4(float array[16]) const {
        array[0] = (float)data[0];
        array[1] = (float)data[4];
        array[2] = (float)data[8];
        array[3] = (float)0;

        array[4] = (float)data[1];
        array[5] = (float)data[5];
        array[6] = (float)data[9];
        array[7] = (float)0;

        array[8] = (float)data[2];
        array[9] = (float)data[6];
        array[10] = (float)data[10];
        array[11] = (float)0;

        array[12] = (float)data[3];
        array[13] = (float)data[7];
        array[14] = (float)data[11];
        array[15] = (float)0;
    }
};

class mat3 {
public:
    real data[9];

    mat3() {
        for (int i = 0; i < 9; ++i) {
            data[i] = 0;
        }
    }

    mat3(const vec3 &c1, const vec3 &c2, const vec3 &c3) {
        setComponents(c1, c2, c3);
    }

    mat3(real e0, real e1, real e2, 
         real e3, real e4, real e5, 
         real e6, real e7, real e8) {
        data[0] = e0;
        data[1] = e1;
        data[2] = e2;

        data[3] = e3;
        data[4] = e4;
        data[5] = e5;

        data[6] = e6;
        data[7] = e7;
        data[8] = e8;
    }

    void setDiagnoal(real a, real b, real c) {
        setInertiaTensorCoeffs(a, b, c);
    }

    void setInertiaTensorCoeffs(real ix, real iy, real iz, real ixy = 0, real ixz = 0, real iyz = 0) {
        data[0] = ix;
        data[1] = data[3] = -ixy;
        data[2] = data[6] = -ixz;
        data[4] = iy;
        data[5] = data[7] = -iyz;
        data[8] = iz;
    }

    void setBlockInertiaTensor(const vec3 &halfsize, real mass) {
        vec3 square = halfsize.componentProduct(halfsize);
        setInertiaTensorCoeffs(0.3f * mass * (square.y + square.z),
                               0.3f * mass * (square.x + square.z),
                               0.3f * mass * (square.x + square.y));
    }

    void setSkewSymmetric(const vec3 &v) {
        //  0 -z  y
        //  z  0 -x
        // -y  x  0
        data[0] = data[4] = data[8] = 0;
        data[1] = -v.z;
        data[2] = v.y;
        data[3] = v.z;
        data[5] = -v.x;
        data[6] = -v.y;
        data[7] = v.x;
    }

    void setComponents(const vec3 &c1, const vec3 &c2, const vec3 &c3) {
        data[0] = c1.x;
        data[1] = c2.x;
        data[2] = c3.x;

        data[3] = c1.y;
        data[4] = c2.y;
        data[5] = c3.y;

        data[6] = c1.z;
        data[7] = c2.z;
        data[8] = c3.z;
    }

    vec3 operator*(const vec3 &v) const {
        return vec3(v.x * data[0] + v.y * data[1] + v.z * data[2],
                    v.x * data[3] + v.y * data[4] + v.z * data[5],
                    v.x * data[6] + v.y * data[7] + v.z * data[8]);
    }

    vec3 transform(const vec3 &v) const {
        return (*this) * v;
    }

    vec3 transformTranspose(const vec3 &v) const {
        return vec3(v.x * data[0] + v.y * data[3] + v.z * data[6],
                    v.x * data[1] + v.y * data[4] + v.z * data[7],
                    v.x * data[2] + v.y * data[5] + v.z * data[8]);
    }

    vec3 getRowVector(int i) const {
        return vec3(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
    }

    vec3 getAxisVector(int i) const {
        return vec3(data[i], data[i + 3], data[i + 6]);
    }

    void setInverse(const mat3 &m) {
        real t4 = m.data[0] * m.data[4];
        real t6 = m.data[0] * m.data[5];
        real t8 = m.data[1] * m.data[3];
        real t10 = m.data[2] * m.data[3];
        real t12 = m.data[1] * m.data[6];
        real t14 = m.data[2] * m.data[6];

        // Calculate the determinant
        real t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] +
                    t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

        // Make sure the determinant is non-zero.
        if (t16 == (real)0.0f) 
            return;
        real t17 = 1 / t16;

        data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
        data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
        data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
        data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
        data[4] = (m.data[0] * m.data[8] - t14) * t17;
        data[5] = -(t6-t10) * t17;
        data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
        data[7] = -(m.data[0] * m.data[7] - t12) * t17;
        data[8] = (t4-t8) * t17;
    }

    mat3 inverse() const {
        mat3 res;
        res.setInverse(*this);
        return res;
    }

    void invert() {
        setInverse(*this);
    }


    void setTranspose(const mat3 &m) {
        data[0] = m.data[0];
        data[1] = m.data[3];
        data[2] = m.data[6];
        data[3] = m.data[1];
        data[4] = m.data[4];
        data[5] = m.data[7];
        data[6] = m.data[2];
        data[7] = m.data[5];
        data[8] = m.data[8];
    }

    mat3 transpose() const {
        mat3 result;
        result.setTranspose(*this);
        return result;
    }

    mat3 operator*(const mat3 &o) const {
        return mat3(
            data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
            data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
            data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

            data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
            data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
            data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

            data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
            data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
            data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]);
    }

    void operator*=(const mat3 &o) {
        real tmp[9];
        tmp[0] = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
        tmp[1] = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
        tmp[2] = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];

        tmp[3] = data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6];
        tmp[4] = data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7];
        tmp[5] = data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8];

        tmp[6] = data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6];
        tmp[7] = data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7];
        tmp[8] = data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8];

        for (int i = 0; i < 9; ++i) {
            data[i] = tmp[i];
        }
    }

    void operator*=(const real scalar) {
        for (int i = 0; i < 9; ++i) {
            data[i] *= scalar;
        }
    }

    void operator+=(const mat3 &o) {
        for (int i = 0; i < 9; ++i) {
            data[i] += o.data[i];
        }
    }

    void setOrientation(const Quaternion &q) {
        data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
        data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
        data[2] = 2 * q.i * q.k - 2 * q.j * q.r;

        data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
        data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
        data[5] = 2 * q.j * q.k + 2 * q.i * q.r;

        data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
        data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
        data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
    }

    static mat3 linearInterpolate(const mat3 &a, const mat3 &b, real p);

};

};