
#pragma once     

//------------------------------------------------------------------------------------
#include "../platform.h" 



//------------------------------------------------------------------------------------
#define M_PI      (3.14159265358979323846f)


#define SQ(x) ((x)*(x))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define ABS(x) ((x) > 0 ? (x) : -(x))

#define DEG_TO_RAD      (M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / M_PI)

#ifndef TRUE
  #define TRUE  1
  #define FALSE 0
#endif

typedef enum {
    X = 0,
    Y,
    Z
} axis_e;

#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi 

//------------------------------------------------------------------------------------
int16_t constrain(int16_t a,int16_t x,int16_t y);
float constrain_float(float a,float x,float y);
int32_t abs(int32_t v);
int16_t max(int16_t x,int16_t y);
int32_t applyDeadband(int32_t value, int32_t deadband);
int32_t quickMedianFilter3(int32_t * v);
int32_t quickMedianFilter5(int32_t * v);
int32_t quickMedianFilter7(int32_t * v);

float invSqrt(float x);
float radians(float deg);
float degrees(float rad);	 

float sin_approx(float x);
float cos_approx(float x);
float atan2_approx(float y, float x);
float acos_approx(float x);




