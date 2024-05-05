
#include "CalibrationManager.h"
#include <Arduino.h>

CalibrationManager::CalibrationManager(MPU6050 *sensorPointer, Preferences *preferencesPointer)
{
    accelgyro = sensorPointer;
    sensorSettings = preferencesPointer;
}

void CalibrationManager::startCalibration()
{
    if (state == 0)
    {
        Serial.println("\nReading sensors for first time...");
        this->meansensors();
        state++;
        delay(1000);
    }

    if (state == 1)
    {
        Serial.println("\nCalculating offsets...");
        this->calibration();
        state++;
        delay(1000);
    }

    if (state == 2)
    {
        this->meansensors();
        Serial.println("\nFINISHED!");
        Serial.print("\nSensor readings with offsets:\t");
        Serial.print(mean_ax);
        Serial.print("\t");
        Serial.print(mean_ay);
        Serial.print("\t");
        Serial.print(mean_az);
        Serial.print("\t");
        Serial.print(mean_gx);
        Serial.print("\t");
        Serial.print(mean_gy);
        Serial.print("\t");
        Serial.println(mean_gz);
        Serial.print("Your offsets:\t");
        Serial.print(ax_offset);
        Serial.print("\t");
        Serial.print(ay_offset);
        Serial.print("\t");
        Serial.print(az_offset);
        Serial.print("\t");
        Serial.print(gx_offset);
        Serial.print("\t");
        Serial.print(gy_offset);
        Serial.print("\t");
        Serial.println(gz_offset);
        // Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
        // Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
        // Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");

        sensorSettings->putInt("acc_x_offset", ax_offset);
        sensorSettings->putInt("acc_y_offset", ay_offset);
        sensorSettings->putInt("acc_z_offset", az_offset);
        sensorSettings->putInt("gryo_x_offset", gx_offset);
        sensorSettings->putInt("gryo_y_offset", gy_offset);
        sensorSettings->putInt("gryo_z_offset", gz_offset);
        sensorSettings->putBool("calDataReady", true);

        Serial.println("Saving data locally\n Restart the device");
    }
}

void CalibrationManager::importCalibrationData()
{
    Serial.println("Reading data from last calibration session");
    sensorSettings->begin("myPrefs", false);
    bool doesExist = sensorSettings->isKey("calDataReady");
    if (doesExist)
    {
        accX_offest = sensorSettings->getInt("acc_x_offset");
        accY_offest = sensorSettings->getInt("acc_y_offset");
        accZ_offest = sensorSettings->getInt("acc_z_offset");
        gyroX_offest = sensorSettings->getInt("gyro_x_offset");
        gyroY_offest = sensorSettings->getInt("gyro_y_offset");
        gyroZ_offest = sensorSettings->getInt("gryo_z_offset");
        Serial.println("Calibration data imported correctly");

        Serial.print("Your offsets:\t");
        Serial.print(accX_offest);
        Serial.print("\t");
        Serial.print(accY_offest);
        Serial.print("\t");
        Serial.print(accZ_offest);
        Serial.print("\t");
        Serial.print(gyroX_offest);
        Serial.print("\t");
        Serial.print(gyroY_offest);
        Serial.print("\t");
        Serial.println(gyroZ_offest);
    }
    else
    {
        Serial.println("No calibration data avaiable. Reccomended run a new calibration session");
        accX_offest = 0;
        accY_offest = 0;
        accZ_offest = 0;
        gyroX_offest = 0;
        gyroY_offest = 0;
        gyroZ_offest = 0;
    }

    setCalibration();
}

void CalibrationManager::setCalibration()
{
    this->accelgyro->setXAccelOffset(this->accX_offest);
    this->accelgyro->setYAccelOffset(this->accY_offest);
    this->accelgyro->setZAccelOffset(this->accZ_offest);
    this->accelgyro->setXGyroOffset(this->gyroX_offest);
    this->accelgyro->setYGyroOffset(-this->gyroY_offest);
    this->accelgyro->setZGyroOffset(this->gyroZ_offest);
}

void CalibrationManager::meansensors()
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    while (i < (buffersize + 101))
    {
        // read raw accel/gyro measurements from device
        this->accelgyro->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i > 100 && i <= (buffersize + 100))
        { // First 100 measures are discarded
            buff_ax = buff_ax + ax;
            buff_ay = buff_ay + ay;
            buff_az = buff_az + az;
            buff_gx = buff_gx + gx;
            buff_gy = buff_gy + gy;
            buff_gz = buff_gz + gz;
        }
        if (i == (buffersize + 100))
        {
            mean_ax = buff_ax / buffersize;
            mean_ay = buff_ay / buffersize;
            mean_az = buff_az / buffersize;
            mean_gx = buff_gx / buffersize;
            mean_gy = buff_gy / buffersize;
            mean_gz = buff_gz / buffersize;
        }
        i++;
        delay(2); // Needed so we don't get repeated measures
    }
}

void CalibrationManager::calibration()
{
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8;

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;
    while (1)
    {
        int ready = 0;
        this->accelgyro->setXAccelOffset(ax_offset);
        this->accelgyro->setYAccelOffset(ay_offset);
        this->accelgyro->setZAccelOffset(az_offset);

        this->accelgyro->setXGyroOffset(gx_offset);
        this->accelgyro->setYGyroOffset(gy_offset);
        this->accelgyro->setZGyroOffset(gz_offset);

        this->meansensors();
        Serial.println("...");

        if (abs(mean_ax) <= acel_deadzone)
            ready++;
        else
            ax_offset = ax_offset - mean_ax / acel_deadzone;

        if (abs(mean_ay) <= acel_deadzone)
            ready++;
        else
            ay_offset = ay_offset - mean_ay / acel_deadzone;

        if (abs(16384 - mean_az) <= acel_deadzone)
            ready++;
        else
            az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

        if (abs(mean_gx) <= giro_deadzone)
            ready++;
        else
            gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

        if (abs(mean_gy) <= giro_deadzone)
            ready++;
        else
            gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

        if (abs(mean_gz) <= giro_deadzone)
            ready++;
        else
            gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

        if (ready == 6)
            break;
    }
}