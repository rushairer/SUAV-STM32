

#pragma once     

//------------------------------------------------------------------------------------
#include "../platform.h" 


//------------------------------------------------------------------------------------
#define XYZ_AXIS_COUNT  3

#define DEGREES_TO_DECIDEGREES(angle) (angle * 10)
#define DECIDEGREES_TO_DEGREES(angle) (angle / 10)
#define DECIDEGREES_TO_RADIANS(angle) ((angle / 10.0f) * 0.0174532925f)
#define DEGREES_TO_RADIANS(angle)     ((angle) * 0.0174532925f)

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in FALSE gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf
#define SPIN_RATE_LIMIT 20

// Floating point 3 vector.
typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def; 

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

typedef struct accDeadband_s {
    uint8_t xy;                 // set the acc deadband for xy-Axis
    uint8_t z;                  // set the acc deadband for z-Axis, this ignores small accelerations
} accDeadband_t;	

typedef struct imuRuntimeConfig_s {
    float dcm_ki;
    float dcm_kp;
} imuRuntimeConfig_t;



//extern int16_t roll_sensor, pitch_sensor, yaw_sensor;
extern int16_t angle[XYZ_AXIS_COUNT];                                             // degree * 100;
extern int32_t accSmooth[XYZ_AXIS_COUNT];

//------------------------------------------------------------------------------------
void imu_init(void);
void imuCalculateAcceleration(float deltaT);
void imuCalculateEstimatedAttitude(void);
void imu_test(void);





