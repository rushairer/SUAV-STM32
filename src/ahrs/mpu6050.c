

#include "mpu6050.h"
#include "../bus/i2c_soft.h"
#include "../schedule/ticks.h"
#include <stdio.h>
#include "../common/mlib.h" 
#include "../msp/msp.h"

#define MPU6050_ADDR    (0x68<<1)

uint8_t mpu6050TestTimer;

int16_t accRaw[3], gyroRaw[3];
int16_t temperuteRaw;

#ifdef ACCGYRO_ACC_Z_TEMPTURE_CMP
float tempture_init;
uint8_t f_temptureInited;
float tempTureCurrent;
#endif

static uint8_t _mpu_half_resolution;
static void mpu6050FindRevision(void);

void mpu6050_init(void)
{
    senseor_init_roll:
    delay_ms(100);
    msp();
    if(soft_i2c_write_reg(MPU6050_ADDR, 0x6B, 0x80)) goto senseor_init_roll; //Reset	
    delay_ms(100);

    soft_i2c_write_reg(MPU6050_ADDR, 0x6B, 0x03);             		   
    delay_ms(2); 			
    soft_i2c_write_reg(MPU6050_ADDR, 0x6C, 0x00);     // enable gyros      	   
    delay_ms(2); 			  
    soft_i2c_write_reg(MPU6050_ADDR, 0x19, 0x01);     // 1kHz/(1+1) = 500Hz   SAMPRT DIV 
    delay_ms(15);
    soft_i2c_write_reg(MPU6050_ADDR, 0x1A, 0x03);     // CONFIG  (EXT_SYNC_SET && DLPF_CFG)  Gyro LPF 184Hz
    soft_i2c_write_reg(MPU6050_ADDR, 0x1B, 0x18); 	  // GYRO_CONFIG (gyr +-2000dps)
    soft_i2c_write_reg(MPU6050_ADDR, 0x1C, 0x10);   	// ACCEL_CONFIG (acc +-8g)	
//    soft_i2c_write_reg(MPU6050_ADDR, 0x1C, 0x18);   	// ACCEL_CONFIG (acc +-16g)	

    //soft_i2c_write_reg(MPU6050_ADDR, 0x1D, 0x05);     // ACC 10.2hz

    soft_i2c_write_reg(MPU6050_ADDR, 0x6A, 0x00);     // Disable FIFO
    delay_ms(10);		

    //-------------------------------------------------------------------------------------
    mpu6050FindRevision();
    
//    printf("half? %d", _mpu_half_resolution);
    
    #ifdef ACCGYRO_ACC_Z_TEMPTURE_CMP
    f_temptureInited = 0;
    #endif

}

uint8_t mpu6050_read(int32_t* acc, int32_t* gyro)
{				
    uint8_t IMUBuffer[14]; 
    uint8_t axis;
    
    if(!soft_i2c_buffer_read(MPU6050_ADDR, MPU_RA_ACCEL_XOUT_H, IMUBuffer, 14)){
        //    for(axis=0; axis<14; axis++) {
        //      printf("%c ", IMUBuffer[axis]);
        //    }
        //    printf("\r\n");

        accRaw[0]     = (int16_t)(((uint16_t)IMUBuffer[0] << 8)  | (uint16_t)IMUBuffer[1]);
        accRaw[1]     = (int16_t)(((uint16_t)IMUBuffer[2] << 8)  | (uint16_t)IMUBuffer[3]);
        accRaw[2]     = (int16_t)(((uint16_t)IMUBuffer[4] << 8)  | (uint16_t)IMUBuffer[5]);
        temperuteRaw  = (int16_t)(((uint16_t)IMUBuffer[6] << 8)  | (uint16_t)IMUBuffer[7]);
        gyroRaw[0]    = (int16_t)(((uint16_t)IMUBuffer[8] << 8)  | (uint16_t)IMUBuffer[9]);
        gyroRaw[1]    = (int16_t)(((uint16_t)IMUBuffer[10] << 8) | (uint16_t)IMUBuffer[11]);
        gyroRaw[2]    = (int16_t)(((uint16_t)IMUBuffer[12] << 8) | (uint16_t)IMUBuffer[13]);	     

        #ifdef ACCGYRO_ACC_Z_TEMPTURE_CMP 
        tempTureCurrent = (float)temperuteRaw/340.0f + 36.53f;
        if(!f_temptureInited) {
            tempture_init = tempTureCurrent;
            f_temptureInited = 1;
        }        
        accRaw[2] -= (tempTureCurrent - tempture_init) * 12.0f;
        #endif
        
        for(axis=0; axis<3; axis++){
            acc[axis] = accRaw[axis];
            if(_mpu_half_resolution) {
                acc[axis] *= 2;
            }
            gyro[axis] = gyroRaw[axis];
        }    
        
        return TRUE; 
    }	  

    return FALSE;
}

static void mpu6050FindRevision(void)
{
    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c

    // determine product ID and revision
    uint8_t readBuffer[6];
  
    uint8_t ack = !soft_i2c_buffer_read(MPU6050_ADDR, MPU_RA_XA_OFFS_H, readBuffer, 6);
    uint8_t revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
  
//    for(uint8_t i=0; i<6;i++)
//      printf("%c ", readBuffer[i]);
//    printf("\r\n");
  
    if (ack && revision) {
        // Congrats, these parts are better
        if (revision == 1) {
            _mpu_half_resolution = 1;
           printf("MPU6050 HALF_RESOLUTION\r\n"); 
        } else if (revision == 2) {
            _mpu_half_resolution = 0;
        } else if ((revision == 3) || (revision == 7)) {
            _mpu_half_resolution = 0;
        } else {          
           printf("MPU6050 REVESION ERROR!\r\n");             
        }
    } else {
        uint8_t productId;
        ack = !soft_i2c_buffer_read(MPU6050_ADDR, MPU_RA_PRODUCT_ID, &productId, 1);
        revision = productId & 0x0F;
        if (!ack || revision == 0) {
           printf("MPU6050 REVESION ERROR!\r\n");    
        } else if (revision == 4) {
            _mpu_half_resolution = 1;
           printf("MPU6050 HALF_RESOLUTION\r\n"); 
        } else {
            _mpu_half_resolution = 0;
        }
    }
}

void mpu6050_test(void)
{
  int32_t acc[3], gyro[3];
  
  if(mpu6050TestTimer >= 5) {
    mpu6050TestTimer = 0;
    
    mpu6050_read(acc, gyro);
    printf("%d,%d,%d; %d,%d,%d\r\n", acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2]);
  }
}


