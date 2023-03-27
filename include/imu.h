#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

MPU6050 mpu;
MPU6050 raw;

#define INTERRUPT_PIN 37 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13		 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;	// set true if DMP init was successful
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;		 // [w, x, y, z]         quaternion container
VectorInt16 aa;		 // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;	 // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];		 // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 gyro;	 // [x, y, z]			gyro vector

float roll, pitch, yaw;
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
float ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
float ax_mean, ay_mean, az_mean, gx_mean, gy_mean, gz_mean;
int16_t ax, ay, az, gx, gy, gz;

float vz = .0f;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
	mpuInterrupt = true;
}

void calibrate();

void imu_setup()
{
	Wire.begin();
	Wire.setClock(400000); // 400 kHz I2C clock.

	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	devStatus = mpu.dmpInitialize();

	mpu.setXGyroOffset(187);
	mpu.setYGyroOffset(9);
	mpu.setZGyroOffset(19);
	mpu.setXAccelOffset(-862);
	mpu.setYAccelOffset(-495);
	mpu.setZAccelOffset(1076);

	if (devStatus == 0)
	{
		// calibrate();

		mpu.setDMPEnabled(true);

		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		dmpReady = true;

		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		Serial.print("DMP Initialization failed (code ");
		Serial.print(devStatus);
		Serial.println(")");
	}
	Serial.println("IMU Setup Complete");
}

void imu_loop()
{
	if (!dmpReady)
		return;

	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	fifoCount = mpu.getFIFOCount();

	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		mpu.resetFIFO();
	}
	else if (mpuIntStatus & 0x02)
	{
		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();

		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

		roll = ypr[2] * RAD_TO_DEG;
		pitch = ypr[1] * RAD_TO_DEG;
		yaw = ypr[0] * RAD_TO_DEG;
		if (yaw < 0)
			yaw += 360; // yaw stays between 0 and 360

		raw.getRotation(&gx, &gy, &gz);

		gyroX = gx / 131.0;
		gyroY = -gy / 131.0;
		gyroZ = -gz / 131.0;

		raw.getAcceleration(&ax, &ay, &az);

		accelX = (ax / 16384.0) * gravity.x;
		accelY = (ay / 16384.0) * gravity.y;
		accelZ = (az / 16384.0) * gravity.z;

		VectorFloat a;
		a.x = accelX;
		a.y = accelY;
		a.z = accelZ;

		a.rotate(&q);

		vz += (-(a.z - 1) * 0.002f); // cm/s
	}
}

void meanSensor()
{
	int buffer_size = 1000;
	int32_t ax_raw = 0, ay_raw = 0, az_raw = 0, gx_raw = 0, gy_raw = 0, gz_raw = 0;
	int32_t mean[6] = { 0, 0, 0, 0, 0, 0 };
	int i = 0;

	while (i < (buffer_size + 101))
	{
		ax_raw = mpu.getAccelerationX();
		ay_raw = mpu.getAccelerationY();
		az_raw = mpu.getAccelerationZ();
		gx_raw = mpu.getRotationX();
		gy_raw = mpu.getRotationY();
		gz_raw = mpu.getRotationZ();

		if (i > 100 && i <= (buffer_size + 100)) {
			mean[0] += ax_raw;
			mean[1] += ay_raw;
			mean[2] += az_raw;
			mean[3] += gx_raw;
			mean[4] += gy_raw;
			mean[5] += gz_raw;
		}

		if (i == (buffer_size + 100)) {
			ax_mean = mean[0] / buffer_size;
			ay_mean = mean[1] / buffer_size;
			az_mean = mean[2] / buffer_size;
			gx_mean = mean[3] / buffer_size;
			gy_mean = mean[4] / buffer_size;
			gz_mean = mean[5] / buffer_size;
		}

		i++;
		delay(2);
	}
}

void calibrate()
{
	int accel_deadzone = 8;
	int gyro_deadzone = 1;

	mpu.setXAccelOffset(0);
	mpu.setYAccelOffset(0);
	mpu.setZAccelOffset(0);
	mpu.setXGyroOffset(0);
	mpu.setYGyroOffset(0);
	mpu.setZGyroOffset(0);

	meanSensor();

	ax_offset = -ax_mean / 8;
	ay_offset = -ay_mean / 8;
	az_offset = (16384 - az_mean) / 8;

	gx_offset = -gx_mean / 4;
	gy_offset = -gy_mean / 4;
	gz_offset = -gz_mean / 4;

	bool ready[6] = { false, false, false, false, false, false };
	while (1)
	{

		mpu.setXAccelOffset(ax_offset);
		mpu.setYAccelOffset(ay_offset);
		mpu.setZAccelOffset(az_offset);

		mpu.setXGyroOffset(gx_offset);
		mpu.setYGyroOffset(gy_offset);
		mpu.setZGyroOffset(gz_offset);

		meanSensor();

		if (!ready[0] && abs(ax_mean) <= accel_deadzone) ready[0] = true;
		else if (ready[0] == false) ax_offset -= ax_mean / accel_deadzone;
		
		if (!ready[1] && abs(ay_mean) <= accel_deadzone) ready[1] = true;
		else if (ready[1] == false) ay_offset -= ay_mean / accel_deadzone;

		if (!ready[2] && abs(16384 - az_mean) <= accel_deadzone) ready[2] = true;
		else if (ready[2] == false) az_offset += (16384 - az_mean) / accel_deadzone;

		if (!ready[3] && abs(gx_mean) <= gyro_deadzone) ready[3] = true;
		else if (ready[3] == false) gx_offset -= gx_mean / gyro_deadzone;

		if (!ready[4] && abs(gy_mean) <= gyro_deadzone) ready[4] = true;
		else if (ready[4] == false) gy_offset -= gy_mean / gyro_deadzone;

		if (!ready[5] && abs(gz_mean) <= gyro_deadzone) ready[5] = true;
		else if (ready[5] == false) gz_offset -= gz_mean / gyro_deadzone;

		Serial.print(ready[0]);
		Serial.print(" ");
		Serial.print(ready[1]);
		Serial.print(" ");
		Serial.print(ready[2]);
		Serial.print(" ");
		Serial.print(ready[3]);
		Serial.print(" ");
		Serial.print(ready[4]);
		Serial.print(" ");
		Serial.print(ready[5]);
		Serial.print("\t");
		Serial.print(ax_offset);
		Serial.print(" ");
		Serial.print(ay_offset);
		Serial.print(" ");
		Serial.print(az_offset);
		Serial.print(" ");
		Serial.print(gx_offset);
		Serial.print(" ");
		Serial.print(gy_offset);
		Serial.print(" ");
		Serial.print(gz_offset);
		Serial.println();

		if (ready[0] && ready[1] && ready[2] && ready[3] && ready[4] && ready[5]) break;
	}

	mpu.PrintActiveOffsets();
}