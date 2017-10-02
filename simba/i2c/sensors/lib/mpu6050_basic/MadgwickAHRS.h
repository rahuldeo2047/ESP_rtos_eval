//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

struct Quat
{
    float w;// = 1.0f;
    float x;// = 0.0f;
    float y;// = 0.0f;
    float z;// = 0.0f;
};

struct Vec3
{
    float x;// = 0.0f;
    float y;// = 0.0f;
    float z;// = 0.0f;
};


//----------------------------------------------------------------------------------------------------
// Variable declaration
extern int instability_fix;
extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
struct Quat q;
//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(struct Quat *q, float gx, float gy, float gz, float ax, float ay, float az);
//void toEulerAngle(const struct Quat* q, double * roll, double * pitch, double * yaw);
void toEulerAngle(const struct Quat * q, struct Vec3 * v);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
