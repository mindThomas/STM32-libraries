//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================

/* Modified version for use in STM32H7 project by
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//-------------------------------------------------------------------------------------------
// Definitions

#define sampleFreqDef   512.0f          // sample frequency in Hz
#define betaDef         0.1f            // 2 * proportional gain

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick{
private:
    static float invSqrt(float x);
    float beta;				// algorithm gain
    float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
    float qDot0, qDot1, qDot2, qDot3; // rate of change in quaternion
    float invSampleFreq;
    float roll, pitch, yaw; // Orientation in rpy
    float roll_vel, pitch_vel, yaw_vel; // Angular velocity in rpy
    char anglesComputed;
    void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations
public:
    Madgwick(float sampleFrequency = sampleFreqDef, float _beta = betaDef);
    void Reset();
    void Reset(float ax, float ay, float az);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float _beta) { float beta_prev = beta; beta = _beta; updateIMU(gx,gy,gz,ax,ay,az); beta = beta_prev; } // hack to allow temporary beta values
    //float getPitch(){return atan2f(2.0f * q2 * q3 - 2.0f * q0 * q1, 2.0f * q0 * q0 + 2.0f * q3 * q3 - 1.0f);};
    //float getRoll(){return -1.0f * asinf(2.0f * q1 * q3 + 2.0f * q0 * q2);};
    //float getYaw(){return atan2f(2.0f * q1 * q2 - 2.0f * q0 * q3, 2.0f * q0 * q0 + 2.0f * q1 * q1 - 1.0f);};
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;
    }
    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }
    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180.0f;
    }
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }
    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }
    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
    float getRollVel() {
        if (!anglesComputed) computeAngles();
        return roll_vel * 57.29578f;
    }
    float getPitchVel() {
        if (!anglesComputed) computeAngles();
        return pitch_vel * 57.29578f;
    }
    float getYawVel() {
        if (!anglesComputed) computeAngles();
        return yaw_vel * 57.29578f + 180.0f;
    }
    float getRollVelRadians() {
        if (!anglesComputed) computeAngles();
        return roll_vel;
    }
    float getPitchVelRadians() {
        if (!anglesComputed) computeAngles();
        return pitch_vel;
    }
    float getYawVelRadians() {
        if (!anglesComputed) computeAngles();
        return yaw_vel;
    }
    void getQuaternion(float q_out[4])
    {
        q_out[0] = q0;
        q_out[1] = q1;
        q_out[2] = q2;
        q_out[3] = q3;
    }
    void getQuaternionDerivative(float q_dot_out[4])
    {
        q_dot_out[0] = qDot0;
        q_dot_out[1] = qDot1;
        q_dot_out[2] = qDot2;
        q_dot_out[3] = qDot3;
    }
};
#endif
