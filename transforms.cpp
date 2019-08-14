#include "transforms.h"

/**
 * @brief dcm2quat - converts DCM to quaternion, where q[0] is scalar element
 * @param R - input DCM
 * @param q - output Quaternion
 */
void dcm2quat(float R[3][3], float q[4])
{
    float q0, q1, q2, q3;
    q0 = sqrt(0.25*(1 + R[0][0] + R[1][1] + R[2][2]));
    q1 = sqrt(0.25*(1 + R[0][0] - R[1][1] - R[2][2]));
    q2 = sqrt(0.25*(1 - R[0][0] + R[1][1] - R[2][2]));
    q3 = sqrt(0.25*(1 - R[0][0] - R[1][1] + R[2][2]));

    if ((q0 > q1) && (q0 > q2) && (q0 > q3)){
        q[0] = q0;
        q[1] = (R[1][2] - R[2][1])/(4*q0);
        q[2] = (R[2][0] - R[0][2])/(4*q0);
        q[3] = (R[0][1] - R[1][0])/(4*q0);
    } else if ((q1 > q0) && (q1 > q2) && (q1 > q3)){
        q[1] = q1;
        q[0] = (R[1][2] - R[2][1])/(4*q1);
        q[2] = (R[0][1] + R[1][0])/(4*q1);
        q[3] = (R[2][0] + R[0][2])/(4*q1);
    } else if ((q2 > q0) && (q2 > q1) && (q2 > q3)){
        q[2] = q2;
        q[0] = (R[2][0] - R[0][2])/(4*q2);
        q[1] = (R[0][1] + R[1][0])/(4*q2);
        q[3] = (R[1][2] + R[2][1])/(4*q2);
    } else {
        q[3] = q3;
        q[0] = (R[0][1] - R[1][0])/(4*q3);
        q[1] = (R[2][0] + R[0][2])/(4*q3);
        q[2] = (R[1][2] + R[2][1])/(4*q3);
    }
}

/**
 * @brief euler2dcm321 - converts Euler angles roll, pitch, yaw to a DCM via a
 * 321 rotation (or rotation in yaw, followed by rotation in pitch, then roll)
 * @param phi - input roll angle (rad)
 * @param theta - input pitch angle (rad)
 * @param psi - input yaw angle (rad)
 * @param R - output DCM
 */
void euler2dcm321(float phi, float theta, float psi, float R[3][3])
{
    R[0][0] = cos(theta)*cos(psi); R[0][1] = cos(theta)*sin(psi); R[0][2] = -sin(theta);
    R[1][0] = -cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi); R[1][1] = cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi); R[1][2] = sin(phi)*cos(theta);
    R[2][0] = sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi); R[2][1] = -sin(phi)*cos(psi) + cos(phi)*sin(theta)*sin(psi); R[2][2] = cos(phi)*cos(theta);
}

/**
 * @brief euler2quat321 - converts Euler angles roll, pitch, yaw to a quaternion via a
 * 321 rotation (or rotation in yaw, followed by rotation in pitch, then roll)
 * @param phi - input roll angle (rad)
 * @param theta - input pitch angle (rad)
 * @param psi - input yaw angle (rad)
 * @param q - output quaternion (q[0] is scalar element)
 */
void euler2quat321(float phi, float theta, float psi, float q[4])
{
    float R[3][3];
    euler2dcm321(phi, theta, psi, R);
    dcm2quat(R, q);
}

/**
 * @brief quat2dcm - converts a quaternion to a DCM
 * @param q - input quaternion (q[0] is scalar element)
 * @param R - output DCM
 */
void quat2dcm(float q[4], float R[3][3])
{
    R[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
    R[0][1] = 2*(q[1]*q[2] + q[0]*q[3]);
    R[0][2] = 2*(q[1]*q[3] - q[0]*q[2]);
    R[1][0] = 2*(q[1]*q[2] - q[0]*q[3]);
    R[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
    R[1][2] = 2*(q[2]*q[3] + q[0]*q[1]);
    R[2][0] = 2*(q[1]*q[3] + q[0]*q[2]);
    R[2][1] = 2*(q[2]*q[3] - q[0]*q[1]);
    R[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
}

/**
 * @brief dcm2euler321 - converts a DCM to Euler angles via a 321 rotation (or
 * rotation in yaw, followed by rotation in pitch, then roll)
 * @param R - input DCM
 * @param phi - output roll angle (rad)
 * @param theta - output pitch angle (rad)
 * @param psi - output yaw angle (rad)
 */
void dcm2euler321(float R[3][3], float &phi, float &theta, float &psi)
{
    phi     = atan2(R[1][2], R[2][2]);
    theta   = -asin(R[0][2]);
    psi     = atan2(R[0][1], R[0][0]);
}

/**
 * @brief quat2euler321 - converts a quaternion to euler angles via a 321 rotation
 * (or a rotation in yaw, followed by rotation in pitch, then roll)
 * @param q - input quaternion (q[0] scalar part)
 * @param phi - output roll (rad)
 * @param theta - output pitch (rad)
 * @param psi - output yaw (rad)
 */
void quat2euler321(float q[4], float &phi, float &theta, float &psi)
{
    float R[3][3];
    quat2dcm(q,R);
    dcm2euler321(R, phi, theta, psi);
}


void posTrans(std::string orientation, float pos[3], float corrpos[3])
{
    float qrot[4];

    if (orientation == "forward") {
        qrot[0] =  0.5;
        qrot[1] =  0.5;
        qrot[2] =  0.5;
        qrot[3] = -0.5;
    }else if (orientation == "backward"){
        qrot[0] =  0.5;
        qrot[1] =  0.5;
        qrot[2] = -0.5;
        qrot[3] =  0.5;
    }else if (orientation == "left"){
        qrot[0] =  0.5*sqrt(2);
        qrot[1] =  0.5*sqrt(2);
        qrot[2] =  0.;
        qrot[3] =  0.;
    }else if (orientation == "right"){
        qrot[0] =  0.;
        qrot[1] =  0.;
        qrot[2] =  0.5*sqrt(2);
        qrot[3] = -0.5*sqrt(2);
    }else {
    //case "up":
    //case "down":
        qrot[0] =  0.5;
        qrot[1] =  0.5;
        qrot[2] =  0.5;
        qrot[3] = -0.5;
    }

    float qrotinv[4], X[4];
    X[0] = 0;
    X[1] = pos[0];
    X[2] = pos[1];
    X[3] = pos[2];
    quatinv(qrot, qrotinv);

    float qtemp[4];
    quatmult(qrotinv, X, qtemp);
    quatmult(qtemp, qrot, X);

    corrpos[0] = X[1];
    corrpos[1] = X[2];
    corrpos[2] = X[3];
}

/**
 * @brief coodTrans - transform input quaternion from body frame B1 to
 * body frame B2 (where B2 is FRD: x is forward, y is right, z is down)
 * @param orientation - the original body frame:
 *      forward - x is right, y is up, z is backward
 *      backward - x is left, y is up, z is forward
 *      right - x is backward, y is up, z is left
 *      left - x is forward, y is up, z is right
 * @param q1 - input attitude quaternion from local inertial frame to body frame B1
 * @param q - output attitude quaternion from local inertial frame to body frame B2 (FRD)
 */
void attTrans(std::string orientation, float q1[4], float q[4])
{
    float qrot[4];

    if (orientation == "forward") {
        qrot[0] =  0.5;
        qrot[1] = -0.5;
        qrot[2] = -0.5;
        qrot[3] =  0.5;
    }else if (orientation == "backward"){
        qrot[0] =  0.5;
        qrot[1] = -0.5;
        qrot[2] =  0.5;
        qrot[3] = -0.5;
    }else if (orientation == "left"){
        qrot[0] =  0.5*sqrt(2);
        qrot[1] = -0.5*sqrt(2);
        qrot[2] =  0.;
        qrot[3] =  0.;
    }else if (orientation == "right"){
        qrot[0] =  0.;
        qrot[1] =  0.;
        qrot[2] = -0.5*sqrt(2);
        qrot[3] =  0.5*sqrt(2);
    }else {
    //case "up":
    //case "down":
        qrot[0] =  0.5;
        qrot[1] = -0.5;
        qrot[2] = -0.5;
        qrot[3] =  0.5;
    }
    float qrotinv[4];
    quatinv(qrot,qrotinv);

    float qtemp[4];
    quatmult(qrot,q1,qtemp);
    quatmult(qtemp,qrotinv,q);
}

/**
 * @brief coorTransCorr -transform input quaternion from body frame B1 to
 * body frame B2 (where B2 is FRD: x is forward, y is right, z is down), and adjust
 * for slight body axis orientation error
 * @param orientation - the original body frame:
 *      forward - x is right, y is up, z is backward
 *      backward - x is left, y is up, z is forward
 *      right - x is backward, y is up, z is left
 *      left - x is forward, y is up, z is right
 * @param q1 - input attitude quaternion from local inertial frame to body frame B1
 * @param dpsi - input yaw error (rad)
 * @param q - output attitude quaternion from local inertial frame to body frame B2 (FRD)
 */
void attTransCorr(std::string orientation, float q1[4], float &dpsi, float q[4])
{
    float dq[4];
    float dphi = 0; float dtheta = 0;
    euler2quat321(dphi, dtheta, dpsi, dq);

    float qrot[4];

    if (orientation == "forward") {
        qrot[0] =  0.5;
        qrot[1] = -0.5;
        qrot[2] = -0.5;
        qrot[3] =  0.5;
    }else if (orientation == "backward"){
        qrot[0] =  0.5;
        qrot[1] = -0.5;
        qrot[2] =  0.5;
        qrot[3] = -0.5;
    }else if (orientation == "left"){
        qrot[0] =  0.5*sqrt(2);
        qrot[1] = -0.5*sqrt(2);
        qrot[2] =  0.;
        qrot[3] =  0.;
    }else if (orientation == "right"){
        qrot[0] =  0.;
        qrot[1] =  0.;
        qrot[2] = -0.5*sqrt(2);
        qrot[3] =  0.5*sqrt(2);
    }else {
    //case "up":
    //case "down":
        qrot[0] =  0.5;
        qrot[1] = -0.5;
        qrot[2] = -0.5;
        qrot[3] =  0.5;
    }
    float qrotinv[4];
    quatinv(qrot,qrotinv);

    float dqinv[4];
    quatinv(dq,dqinv);

    float qtemp1[4], qtemp2[4], qtemp3[4];
    quatmult(qrot,dq,qtemp1);
    quatmult(qtemp1, q1, qtemp2);
    quatmult(qtemp2,dqinv, qtemp3);
    quatmult(qtemp3, qrotinv, q);
}

/**
 * @brief quatmult - multiplies quaternion q1 and quaternion q2
 * @param q1 - first input quaternion (q[0] scalar part)
 * @param q2 - second input quaternion (q[0] scalar part)
 * @param q - output quaternion (q[0] scalar part)
 */
void quatmult(float q1[4], float q2[4], float q[4])
{
    q[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    q[1] = q1[1]*q2[0] + q1[0]*q2[1] - q1[3]*q2[2] + q1[2]*q2[3];
    q[2] = q1[2]*q2[0] + q1[3]*q2[1] + q1[0]*q2[2] - q1[1]*q2[3];
    q[3] = q1[3]*q2[0] - q1[2]*q2[1] + q1[1]*q2[2] + q1[0]*q2[3];
}

void quatinv(float q1[4], float q[4])
{
    float qnorm = q1[0]*q1[0] + q1[1]*q1[1] + q1[2]*q1[2] + q1[3]*q1[3];
    if (qnorm > 0.01 && qnorm < 100.0){
        q[0] = q1[0]/qnorm;
        q[1] = -q1[1]/qnorm;
        q[2] = -q1[2]/qnorm;
        q[3] = -q1[3]/qnorm;
    } else {
        q[0] = 1;
        q[1] = 0;
        q[2] = 0;
        q[3] = 0;
    }
}


/**
 * @brief poseTransArb -transform input quaternion from body frame B1 to
 * body frame B2 (where B2 is FRD: x is forward, y is right, z is down), and adjust
 * for slight body axis orientation error. Also adjust position coordinate.
 * @param orientation - the original body frame:
 *      forward - x is right, y is up, z is backward
 *      backward - x is left, y is up, z is forward
 *      right - x is backward, y is up, z is left
 *      left - x is forward, y is up, z is right
 * @param q1 - input attitude quaternion from local inertial frame to body frame B1
 * @param X - input position estimate
 * @param dA - input angle error (rad) - error from prescribed orientation
 * @param corrq - output attitude quaternion from local inertial frame to body frame B2 (FRD)
 * @param corrX - output position in NED coordinates
 */
void poseTransArb(std::string orientation, float q1[4], float X[3], float dA[3], float corrq[4], float corrX[3])
{
    /*double phi, theta, psi;
    quat2euler321(q1, &phi, &theta, &psi);
    phi -= dA[0];
    theta -= dA[1];
    euler2quat321(&phi, &theta, &psi, q1);
    */
    float dphi = 0; float dtheta = 0; float dpsi = dA[2];
    float dQ[4];
    euler2quat321(dphi, dtheta, dpsi, dQ);
    float qrot[4];

    if (orientation == "forward") {
        qrot[0] =  0.5;
        qrot[1] =  0.5;
        qrot[2] =  0.5;
        qrot[3] = -0.5;
    }else if (orientation == "backward"){
        qrot[0] =  0.5;
        qrot[1] =  0.5;
        qrot[2] = -0.5;
        qrot[3] =  0.5;
    }else if (orientation == "left"){
        qrot[0] =  0.5*sqrt(2);
        qrot[1] =  0.5*sqrt(2);
        qrot[2] =  0.;
        qrot[3] =  0.;
    }else if (orientation == "right"){
        qrot[0] =  0.;
        qrot[1] =  0.;
        qrot[2] =  0.5*sqrt(2);
        qrot[3] = -0.5*sqrt(2);
    }else {
    //case "up":
    //case "down":
        qrot[0] =  0.5;
        qrot[1] =  0.5;
        qrot[2] =  0.5;
        qrot[3] = -0.5;
    }
    float qrotinv[4];
    quatinv(qrot,qrotinv);

    float dQinv[4];
    quatinv(dQ,dQinv);

    float X4[4];
    X4[0] = 0;
    X4[1] = X[0];
    X4[2] = X[1];
    X4[3] = X[2];
    float qtemp1[4], qtemp1inv[4], temp1[4];
    quatmult(qrot,dQinv,qtemp1);
    quatinv(qtemp1,qtemp1inv);
    quatmult(qtemp1inv,X4,temp1);
    quatmult(temp1, qtemp1,X4);
    corrX[0] = X4[1];
    corrX[1] = X4[2];
    corrX[2] = X4[3];

    float corrqtemp[4];
    quatmult(qtemp1inv, q1, temp1);
    quatmult(temp1,qtemp1, corrqtemp);

    float phi, theta, psi;
    quat2euler321(corrqtemp, phi, theta, psi);
    phi -= dA[0];
    theta -= dA[1];
    euler2quat321(phi, theta, psi, corrq);
    /*double phi, theta, psi;
    quat2euler321(q1, &phi, &theta, &psi);
    phi -= dA[0];
    theta -= dA[1];
    euler2quat321(&phi, &theta, &psi, corrq);*/
}

