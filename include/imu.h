#include <Arduino.h>
#include <Wire.h>

#define IMU_ADDRESS 0x68

int16_t manual_acc_pitch_cal_value = 0;
int16_t manual_acc_roll_cal_value = 0;
uint8_t use_manual_calibration = false;    // Set to false or true;
int16_t manual_gyro_pitch_cal_value = 0;
int16_t manual_gyro_roll_cal_value = 0;
int16_t manual_gyro_yaw_cal_value = 0;
int16_t temperature, count_var, cal_int;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int32_t acc_total_vector;
float roll_level_adjust, pitch_level_adjust;
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

bool auto_level = true;                 //Auto level on (true) or off (false).


void imu_read() {
    Wire.beginTransmission(IMU_ADDRESS);                       //Start communication with the gyro.
    Wire.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                     //End the transmission.
    Wire.requestFrom(IMU_ADDRESS, 14);                         //Request 14 bytes from the MPU 6050.
    acc_y = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_x variable.
    acc_x = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_y variable.
    acc_z = Wire.read() << 8 | Wire.read();                    //Add the low and high byte to the acc_z variable.
    temperature = Wire.read() << 8 | Wire.read();              //Add the low and high byte to the temperature variable.
    gyro_roll = Wire.read() << 8 | Wire.read();                //Read high and low part of the angular data.
    gyro_pitch = Wire.read() << 8 | Wire.read();               //Read high and low part of the angular data.
    gyro_yaw = Wire.read() << 8 | Wire.read();                 //Read high and low part of the angular data.
    gyro_pitch *= -1;                                            //Invert the direction of the axis.
    gyro_yaw *= -1;                                              //Invert the direction of the axis.

    acc_y -= manual_acc_pitch_cal_value;                         //Subtact the manual accelerometer pitch calibration value.
    acc_x -= manual_acc_roll_cal_value;                          //Subtact the manual accelerometer roll calibration value.
    gyro_roll -= manual_gyro_roll_cal_value;                     //Subtact the manual gyro roll calibration value.
    gyro_pitch -= manual_gyro_pitch_cal_value;                   //Subtact the manual gyro pitch calibration value.
    gyro_yaw -= manual_gyro_yaw_cal_value;                       //Subtact the manual gyro yaw calibration value.
}

void imu_calibrate() {
    if (use_manual_calibration)cal_int = 2000;                                          //If manual calibration is used set cal_int to 2000 to skip the calibration.
    else {
        cal_int = 0;                                                                      //If manual calibration is not used.
        manual_gyro_pitch_cal_value = 0;                                                  //Set the manual pitch calibration variable to 0.
        manual_gyro_roll_cal_value = 0;                                                   //Set the manual roll calibration variable to 0.
        manual_gyro_yaw_cal_value = 0;                                                    //Set the manual yaw calibration variable to 0.
    }

    if (cal_int != 2000) {
        //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
        for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
            if (cal_int % 25 == 0) digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));    //Change the led status every 125 readings to indicate calibration.
            imu_read();                                                                //Read the gyro output.
            gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
            gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
            gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
            delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
        }
        //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
        gyro_roll_cal /= 2000;                                                            //Divide the roll total by 2000.
        gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
        gyro_yaw_cal /= 2000;                                                             //Divide the yaw total by 2000.
        manual_gyro_pitch_cal_value = gyro_pitch_cal;                                     //Set the manual pitch calibration variable to the detected value.
        manual_gyro_roll_cal_value = gyro_roll_cal;                                       //Set the manual roll calibration variable to the detected value.
        manual_gyro_yaw_cal_value = gyro_yaw_cal;                                         //Set the manual yaw calibration variable to the detected value.
    }
}

void imu_setup() {
    Wire.begin();
    Wire.setClock(400000); // 400 kHz I2C clock.

    Wire.beginTransmission(IMU_ADDRESS);                         //Start communication with the MPU-6050.
    Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
    Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(IMU_ADDRESS);                         //Start communication with the MPU-6050.
    Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(IMU_ADDRESS);                         //Start communication with the MPU-6050.
    Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
    Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(IMU_ADDRESS);                         //Start communication with the MPU-6050.
    Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
    Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
    Wire.endTransmission();                                      //End the transmission with the gyro.

    if (!use_manual_calibration) {
        //Create a 5 second delay before calibration.
        for (count_var = 0; count_var < 1250; count_var++) {        //1250 loops of 4 microseconds = 5 seconds
            if (count_var % 125 == 0) {                               //Every 125 loops (500ms).
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));                   //Change the led status.
            }
            delay(4);                                                 //Delay 4 microseconds
        }
        count_var = 0;                                              //Set start back to 0.
    }
    Serial.println("Calibrating Imu");
    imu_calibrate();                                             //Calibrate the gyro offset.
    Serial.println("IMU Setup Complete");
}

void imu_loop() {
    imu_read();                                                 //Read the raw acc and gyro data from the MPU-6050.

    //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
    gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
    gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += (float)gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll += (float)gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
    angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));    //Calculate the total accelerometer vector.

    if (abs(acc_y) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
        angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
    }
    if (abs(acc_x) < acc_total_vector) {                                             //Prevent the asin function to produce a NaN.
        angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.
    }

    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                   //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                      //Correct the drift of the gyro roll angle with the accelerometer roll angle.

    pitch_level_adjust = angle_pitch * 15;                                           //Calculate the pitch angle correction.
    roll_level_adjust = angle_roll * 15;                                             //Calculate the roll angle correction.

    if (!auto_level) {                                                               //If the quadcopter is not in auto-level mode
        pitch_level_adjust = 0;                                                        //Set the pitch angle correction to zero.
        roll_level_adjust = 0;                                                         //Set the roll angle correcion to zero.
    }
}