

#include "accgyro.h"
#include "mpu6050.h"
#include "../common/mlib.h"
#include "../common/filter.h"
#include <string.h>
#include <stdio.h>
#include "../schedule/ticks.h"
#include "alignment.h"
#include "../iap/iap.h"

accgyro_s accgyro;

static pt1Filter_t gyroFilter[3];
static biquadFilter_t accFilter[3];

int16_t acc_1G;
float gyroScale;    // gyro output scaled to rad per second

void gyro_calbration(accgyro_s* gyro);
void acc_calbration(accgyro_s* acc);

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
void accgyro_init(void)
{
  uint8_t axis;
  
  memset(&accgyro, 0, sizeof(accgyro));
  accgyro.accgyro_init = mpu6050_init;
  accgyro.accgyro_read = mpu6050_read;  
  accgyro.accgyro_init();  
  
  for(axis=0; axis<3; axis++) {
    biquadFilterInitLPF(&accFilter[axis], 30, 1250);
    pt1FilterInit(&gyroFilter[axis], 240, 1.25e-3);   
    if(axis == 2) {
      accgyro.accData[axis] = ACC_1G;
    }     
  }
  
  acc_1G = ACC_1G;
  gyroScale = 1.0f / 16.4f * (M_PI / 180.0f);  
  
  if(cfgTemp.good == 0x5a5aa5a5) {
    for(axis=0; axis<3; axis++) {
      accgyro.accRaw_offset[axis] = cfgTemp.acc_offset[axis];
      accgyro.gyroRaw_offset[axis]= cfgTemp.gyro_offset[axis];
    }
    #ifdef ACCGYRO_ACC_Z_TEMPTURE_CMP
    tempture_init = cfgTemp.temperature;
    f_temptureInited = 1;
    #endif
  } else {
    cfgTemp.good = 0x5a5aa5a5;
    accgyro.gyroCalibrating = 1;
    accgyro.accCalibrating = 1; 
    #ifdef ACCGYRO_ACC_Z_TEMPTURE_CMP
    f_temptureInited = 0;
    #endif
  }
  
}

uint8_t accgyro_update(void)
{
  uint8_t axis;
  
  if(accgyro.accgyro_read(accgyro.accRaw, accgyro.gyroRaw)) {      
    apply_sensor_alignment(accgyro.accRaw, accgyro.accRaw, CW270_DEG);
    apply_sensor_alignment(accgyro.gyroRaw, accgyro.gyroRaw, CW270_DEG);
    
    // 偏差较准
    if(accgyro.gyroCalibrating) {
      gyro_calbration(&accgyro);
    }
    if(accgyro.accCalibrating) {
      #ifdef ACCGYRO_ACC_Z_TEMPTURE_CMP
      f_temptureInited = 0;
      #endif
      acc_calbration(&accgyro);
    }
    
    // 原始数据-偏差
    for(axis=0; axis<3; axis++){
      accgyro.accValue[axis] = accgyro.accRaw[axis] - accgyro.accRaw_offset[axis];
      accgyro.gyroValue[axis]= accgyro.gyroRaw[axis]- accgyro.gyroRaw_offset[axis];
    }
    
    // 数据滤波
    for(axis=0; axis<3; axis++){
      accgyro.accData[axis] = biquadFilterApply(&accFilter[axis], accgyro.accValue[axis]);
      accgyro.gyroData[axis]= pt1FilterApply(&gyroFilter[axis], accgyro.gyroValue[axis]);
    }     
        
    return TRUE;
  }
  return FALSE;
}


//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
void gyro_calbration(accgyro_s* gyro)
{
  static int32_t gyroSum[3];
  static int32_t gyroInit[3];
  static uint16_t count;
  static uint8_t gyroCalInit = 1;
  
  uint8_t axis;
  int32_t dif;
  
  // 开始较准，初始化变量。
  if(gyroCalInit) {
    for(axis=0; axis<3; axis++){
      gyroSum[axis] = 0;
      gyroInit[axis] = gyro->gyroRaw[axis];
      count = 0;
    }    
    gyroCalInit = 0;
  }
  
  // 较准中，如果偏差大于设定值，重新开始。
  #define GYRO_CALI_DIFF_LIMIT    50
  for(axis=0; axis<3; axis++) {
    dif = gyroInit[axis] - gyro->gyroRaw[axis];
    if(abs(dif) < GYRO_CALI_DIFF_LIMIT) {
      gyroSum[axis] += gyro->gyroRaw[axis];
    } else {
      gyroCalInit = 1;
      count = 0;
    }
  }
  
  // 较准完成，取均值作为offset。同时置位gyroCalInit，留作下次较准使用。
  if(++count >= 256) {
    for(axis=0; axis<3; axis++) {
      gyro->gyroRaw_offset[axis] = gyroSum[axis] / count;
      cfgTemp.gyro_offset[axis] = gyro->gyroRaw_offset[axis];
    }
    gyro->gyroCalibrating = 0;
    gyroCalInit = 1;
    cfg_save_temp();
  }  
}

void acc_calbration(accgyro_s* acc)
{
  static int32_t accSum[3];
  static int32_t gyroInit[3];
  static uint16_t count;
  static uint8_t accCalInit = 1;
  
  uint8_t axis;
  int32_t dif;
  
  // 开始较准，初始化变量。
  if(accCalInit) {
    for(axis=0; axis<3; axis++){
      accSum[axis] = 0;
      gyroInit[axis] = acc->gyroRaw[axis];
      count = 0;
      acc->accTrim[axis] = 0;
    }    
    accCalInit = 0;
  }
  
  // 较准中，如果陀螺仪偏差大于设定值，重新开始。
  #define ACC_CALI_GYRO_DIFF_LIMIT    50
  for(axis=0; axis<3; axis++) {
    dif = gyroInit[axis] - acc->gyroRaw[axis];
    if(abs(dif) < ACC_CALI_GYRO_DIFF_LIMIT) {
      accSum[axis] += acc->accRaw[axis];
    } else {
      accCalInit = 1;
      count = 0;
    }
  }
  
  // 较准完成，取均值作为offset。同时置位accCalInit，留作下次较准使用。
  if(++count >= 256) {
    for(axis=0; axis<3; axis++) {
      acc->accRaw_offset[axis] = accSum[axis] / count;
      if(axis == 2) {
        acc->accRaw_offset[axis] -= acc_1G;
      }
      cfgTemp.acc_offset[axis] = acc->accRaw_offset[axis];
    }
    #ifdef ACCGYRO_ACC_Z_TEMPTURE_CMP
    cfgTemp.temperature = tempture_init;
    #endif
    acc->accCalibrating = 0;
    accCalInit = 1;
    powerupTimer = 0;
    cfg_save_temp();
  }  
}


void accgyro_test(void)
{
  static uint8_t printfcnt;
  
  float deltaT;
  static uint16_t previousTime;
  uint16_t currentTime = micros16();

  deltaT = (uint16_t)(currentTime-previousTime)*1e-6;
  
  // 2ms
  if(deltaT >= 0.002f) {
    previousTime = currentTime; 
    
    accgyro_update();
    
    if(printfcnt++ >= 5) {
      printfcnt = 0;
      printf("%d,%d,%d,%d\n", accgyro.gyroRaw[0], accgyro.gyroData[0], accgyro.accRaw[0], accgyro.accData[0]);
    }
  } 
  
}




