
#include "MPU.h"
void initMPU()
{
    I2Cdev::writeBits(SENSOR_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, 1);
    I2Cdev::writeBits(SENSOR_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, 0);
    I2Cdev::writeBits(SENSOR_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, 0);
    I2Cdev::writeBit(SENSOR_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, false);

    I2Cdev::writeByte(SENSOR_ADDRESS, 0x6B, 0b10000000); // Reset device
    delay(100);
    I2Cdev::writeByte(SENSOR_ADDRESS, 0x68, 0b00000111); // Reset sensors
    delay(100);

    I2Cdev::writeByte(SENSOR_ADDRESS, 0x6B, 0b00000001);               // Internal Clock set to Gyro output
    I2Cdev::writeByte(SENSOR_ADDRESS, 0x6A, 0b00000100);               // Reset FIFO
    I2Cdev::writeByte(SENSOR_ADDRESS, 0x19, SAMPLING_FREQUENCY_100HZ); // Sample frequency = 1000 / (1 + n)
    I2Cdev::writeByte(SENSOR_ADDRESS, 0x1A, 0b00000001);               // DLPF on set to lowest filter
    I2Cdev::writeByte(SENSOR_ADDRESS, 0x1C, 0b00000000);               // Accelerometer sensitivity
    I2Cdev::writeByte(SENSOR_ADDRESS, 0x23, 0b01111000);               // Load Accelerometer (3° bit) & gyro (6° = x, 5° = y, 4° = z bit) to FIFO
    I2Cdev::writeByte(SENSOR_ADDRESS, 0x38, 0b00000001);               // Data ready Interrupt
    I2Cdev::writeByte(SENSOR_ADDRESS, 0x6A, 0b01000100);               // Enable FIFO
}