#ifndef COORDINATETRANSFORM_H
#define COORDINATETRANSFORM_H

#include <math.h>
#include <string>

/**
 * DCMs, euler angles, and quaternions define a rotation from frame A to
 * frame B. For example; a motion capture quaternion defines a rotation
 * FROM the local INERTIAL frame TO the BODY frame.
 *
 * The coorTrans functions transform a quaternion q1 (which describes
 * rotation from local inertial frame A to body frame B1) to quaternion q
 * (which describes rotation from local inertial frame A to body frame B2).
 * That is, a rotation from A -> B1 and followed by a rotation from B1 -> B2.
 */

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void dcm2quat(float R[3][3], float q[4]);

void euler2dcm321(float phi, float theta, float psi, float R[3][3]);

void euler2quat321(float phi, float theta, float psi, float q[4]);

void quat2dcm(float q[4], float R[3][3]);

void dcm2euler321(float R[3][3], float &phi, float &theta, float &psi);

void quat2euler321(float q[4], float &phi, float &theta, float &psi);

void quatmult(float q1[4], float q2[4], float q[4]);

void quatinv(float q1[4], float q[4]);

void posTrans(std::string orientation, float pos[3], float corrpos[3]);

void attTrans(std::string orientation, float q1[4], float q[4]);

void attTransCorr(std::string orientation, float q1[4], float &dpsi, float q[4]);

void poseTransArb(std::string orientation, float q1[4], float X[3], float dA[3], float corrq[4], float corrX[3]);

#endif // COORDINATETRANSFORM_H
