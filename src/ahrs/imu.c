
#include "imu.h"
#include "accgyro.h" 
#include "../fc/fc.h"
#include "../ahrs/altitude.h"
#include "../common/mlib.h"
#include "../schedule/ticks.h"
#include <math.h>
#include <stdio.h>



float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;             // quaternion of sensor frame relative to earth frame 
static float rMat[3][3];

int16_t angle[XYZ_AXIS_COUNT];                                // degree * 100;

int32_t accSmooth[XYZ_AXIS_COUNT] = {0,0,ACC_1G};
static imuRuntimeConfig_t imuConfig;
static imuRuntimeConfig_t *imuRuntimeConfig = &imuConfig;

//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------ 
void imuComputeRotationMatrix(float (*_rMat)[3], float* q)
{
    float q1q1 = SQ(q[1]);
    float q2q2 = SQ(q[2]);
    float q3q3 = SQ(q[3]);

    float q0q1 = q[0] * q[1];
    float q0q2 = q[0] * q[2];
    float q0q3 = q[0] * q[3];
    float q1q2 = q[1] * q[2];
    float q1q3 = q[1] * q[3];
    float q2q3 = q[2] * q[3];

    _rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    _rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    _rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    _rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    _rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    _rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    _rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    _rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    _rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuTransformVectorBodyToEarth(t_fp_vector * v, float (*_rMat)[3])
{
    float x,y,z;

    /* From body frame to earth frame */
    x = _rMat[0][0] * v->V.X + _rMat[0][1] * v->V.Y + _rMat[0][2] * v->V.Z;
    y = _rMat[1][0] * v->V.X + _rMat[1][1] * v->V.Y + _rMat[1][2] * v->V.Z;
    z = _rMat[2][0] * v->V.X + _rMat[2][1] * v->V.Y + _rMat[2][2] * v->V.Z;

    v->V.X = x;
    v->V.Y = y;//-y;
    v->V.Z = z;
}

//------------------------------------------------------------------------------------------------
// create a quaternion from Euler angles
void from_euler(float roll, float pitch, float yaw, float* q)
{
    float cr2 = cosf(roll*0.5f);
    float cp2 = cosf(pitch*0.5f);
    float cy2 = cosf(yaw*0.5f);
    float sr2 = sinf(roll*0.5f);
    float sp2 = sinf(pitch*0.5f);
    float sy2 = sinf(yaw*0.5f);

    q[0] = cr2*cp2*cy2 + sr2*sp2*sy2;
    q[1] = sr2*cp2*cy2 - cr2*sp2*sy2;
    q[2] = cr2*sp2*cy2 + sr2*cp2*sy2;
    q[3] = cr2*cp2*sy2 - sr2*sp2*cy2;
}

// create eulers from a quaternion
void to_euler(float *roll, float *pitch, float *yaw)
{
    if (roll) {
        *roll = (atan2f(2.0f*(q0*q1 + q2*q3), 1 - 2.0f*(q1*q1 + q2*q2)));
    }
    if (pitch) {
        // we let safe_asin() handle the singularities near 90/-90 in pitch
        *pitch = sin_approx(2.0f*(q0*q2 - q3*q1));
    }
    if (yaw) {
        *yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1 - 2.0f*(q2*q2 + q3*q3));
    }
}


//------------------------------------------------------------------------------------------------
// rotate acc into Earth frame and calculate acceleration in it
void imuCalculateAcceleration(float deltaT)
{			
    t_fp_vector accel_ned;
    float q[4], _rMat[3][3];
    float angleRad[3];

    //----------------------------------------------------------------------  
    to_euler(&angleRad[0], &angleRad[1], 0);
    from_euler(angleRad[0], angleRad[1], 0, q);
    imuComputeRotationMatrix(_rMat, q);

    //----------------------------------------------------------------------
    accel_ned.V.X = -accgyro.accData[1]; 
    accel_ned.V.Y = accgyro.accData[0]; 
    accel_ned.V.Z = accgyro.accData[2];

    imuTransformVectorBodyToEarth(&accel_ned, _rMat);                 // 机体加速度转换成地球加速度(已经去除航向角影响)

    //----------------------------------------------------------------------  
    accel_ned.V.X -= accgyro.accTrim[0];
    accel_ned.V.Y -= accgyro.accTrim[1];
    accel_ned.V.Z -= (acc_1G + accgyro.accTrim[2]);	  

    accgyro.accEarth[2] = accel_ned.V.Z;

    //    printf("%d, %d, %d\n", (int)accel_ned.V.Z, angle[0]/100, angle[1]/100);

    // apply Deadband to reduce integration drift and vibration influence
    accSumZ[0] += applyDeadband((int32_t)(accel_ned.V.X), 0);
    accSumZ[1] += applyDeadband((int32_t)(accel_ned.V.Y), 0); 
    accSumZ[2] += applyDeadband((int32_t)(accel_ned.V.Z), 0); 		

    // sum up Values for later integration to get velocity and distance
    accSumZTimer += deltaT;			
    accSumZCount++;
}

static bool imuUseFastGains(void)
{
  if(powerupTimer >= 100) { // 2s
    powerupTimer = 100;
  } 
  return (powerupTimer < 100);
}

static float imuGetPGainScaleFactor(void)
{
    if (imuUseFastGains()) {
        return 10.0f;
		} else {
        return 1.0f;
    }
}

static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki
    float recipNorm;
    float ex = 0, ey = 0, ez = 0;
    float qa, qb, qc;
	  float dcmKpGain;
	  float q[4];

    // Calculate general spin rate (rad/s)
    float spin_rate = sqrtf(SQ(gx) + SQ(gy) + SQ(gz));

    // Use measured acceleration vector
    recipNorm = SQ(ax) + SQ(ay) + SQ(az);
    if (useAcc && recipNorm > 0.01f) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(recipNorm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if(imuRuntimeConfig->dcm_ki > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            float dcmKiGain = imuRuntimeConfig->dcm_ki;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    }
    else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Calculate kP gain. If we are acquiring initial attitude (not armed and within 20 sec from powerup) scale the kP to converge faster
    dcmKpGain = imuRuntimeConfig->dcm_kp * imuGetPGainScaleFactor();

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
				
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(SQ(q0) + SQ(q1) + SQ(q2) + SQ(q3));
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Pre-compute rotation matrix from quaternion
		q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
    imuComputeRotationMatrix(rMat, q);                            
}

static bool imuIsAccelerometerHealthy(void)
{
    int32_t axis;
    int32_t accMagnitude = 0;

    for (axis = 0; axis < 3; axis++) {
        accMagnitude += (int32_t)accSmooth[axis] * accSmooth[axis];
    }

    accMagnitude = accMagnitude * 100 / (SQ((int32_t)acc_1G));

    // Accept accel readings only in range 0.90g - 1.10g
    return (bool)((81 < accMagnitude) && (accMagnitude < 121));
}

void imuUpdateEulerAngles(void)
{		
    angle[ROLL]  = (int32_t)(atan2_approx(rMat[2][1], rMat[2][2]) * (18000.0f / M_PI));
    angle[PITCH] = (int32_t)(((0.5f * M_PI) - acos_approx(-rMat[2][0])) * (18000.0f / M_PI));
    angle[YAW]   = (int32_t)((-atan2_approx(rMat[1][0], rMat[0][0]) * (18000.0f / M_PI)));		
						
		//输出-180 ~ +180
    if(abs(angle[ROLL]) >= 9000){
  	  if(angle[PITCH] > 0) angle[PITCH] = 18000 - angle[PITCH];
		  else angle[PITCH] = -18000 - angle[PITCH];		
    }				
}

void imuCalculateEstimatedAttitude(void)
{
    static uint16_t previousIMUUpdateTime;
    uint16_t currentTime;
    float deltaT;
  
    bool useAcc = FALSE;
	  int32_t gyroData[3];
	  uint8_t axis;

    //-------------------------------------------------------------------------------------------------
    currentTime = micros16();
    deltaT = (uint16_t)(currentTime-previousIMUUpdateTime)*1e-6;
    previousIMUUpdateTime = currentTime;  
  
	  if (deltaT > 0.2f) {
        return;
    }

    for(axis=0; axis<3; axis++) {
      accSmooth[axis] = (accSmooth[axis]*7 + accgyro.accData[axis]) / 8;
      gyroData[axis] = accgyro.gyroData[axis];      
    }    
    
    if (imuIsAccelerometerHealthy()) {
        useAcc = TRUE;
    }
		
    //-------------------------------------------------------------------------------------------------
		//1) 更新四元数
    imuMahonyAHRSupdate(deltaT,
                        gyroData[0] * gyroScale, gyroData[1] * gyroScale, gyroData[2] * gyroScale,
                        useAcc, accSmooth[0], accSmooth[1], accSmooth[2]);
    
    //2) 生成欧拉角
    imuUpdateEulerAngles();		             

		//3) 由转移矩阵计算出机身各轴加速度
    imuCalculateAcceleration(deltaT); 
        
}

void imu_init(void)
{  
	  float q[4];
	
	  imuRuntimeConfig->dcm_kp = 2500.0f / 10000.0f;
	  imuRuntimeConfig->dcm_ki = 25.0f / 10000.0f;
	
		q[0] = q0; q[1] = q1; q[2] = q2; q[3] = q3;
    imuComputeRotationMatrix(rMat, q);
  
}

void imu_test(void)
{  
//  static uint8_t printfcnt;
  
  float deltaT;
  static uint16_t previousTime;
  uint16_t currentTime = micros16();

  deltaT = (uint16_t)(currentTime-previousTime)*1e-6;
  
  // 2ms
  if(deltaT >= 0.002f) {
    previousTime = currentTime; 
    
    accgyro_update();
    imuCalculateEstimatedAttitude();
    
//    if(printfcnt++ >= 5) {
//      printfcnt = 0;
//      printf("%02f,%02f,%02f\n", (float)angle[ROLL]/100.0f, (float)angle[PITCH]/100.0f, (float)angle[YAW]/100.0f);
//    }
  } 
  
}
