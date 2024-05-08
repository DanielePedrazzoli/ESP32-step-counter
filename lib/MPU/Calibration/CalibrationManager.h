
#ifndef _MPU6050_6AXIS_MOTIONAPPS612_H_
#include <MPU6050_6Axis_MotionApps612.h>
#endif

#ifndef _CALIBRATION_MANAGER_
#define _CALIBRATION_MANAGER_

#include <Preferences.h>

class CalibrationManager
{
private:
    int16_t accX_offest = 63;
    int16_t accY_offest = -1302;
    int16_t accZ_offest = 1082;
    int16_t gyroX_offest = 238;
    int16_t gyroY_offest = -43;
    int16_t gyroZ_offest = 28;

    MPU6050 *accelgyro;
    Preferences *sensorSettings;

    void setCalibration();
    void meansensors();
    void calibration();

    int16_t ax, ay, az, gx, gy, gz;
    int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
    int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

    // Change this 3 variables if you want to fine tune the skecth to your needs.
    int buffersize = 1000; // Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
    int acel_deadzone = 8; // Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
    int giro_deadzone = 1; // Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

    bool isAlreadyCalibrated;

public:
    CalibrationManager(MPU6050 *, Preferences *);

    void startCalibration();
    void importCalibrationData();
    int needCalibration();
};
#endif
