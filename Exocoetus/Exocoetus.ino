#include <SparkFun_MS5803_I2C.h>	// Test comment for github
#include <Kalman.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <PID_v1.h>
#include <Servo.h>
#include <SPI.h>					// for SD card, datalogging
#include <SD.h>						// for SD card, datalogging

/*Pins Used in Code
Servos: 9, 10
Motors: 5, 6
SD:
Pressure Transducer: SDA, SCL
IMU: SDA, SCL
LED:
Solenoid:
*/

Adafruit_9DOF                 dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro = Adafruit_L3GD20_Unified(20303);
MS5803						  transducer = MS5803(ADDRESS_HIGH);

uint32_t timer;

Kalman kalmanY;

Servo servo1;


double gyroY;
double kalAngleY;
double gyro_data_roll;

double Servo_Setpoint, Servo_Input, Servo_Output;
double Motor_Setpoint, Motor_Input, Motor_Output;
double Press_Setpoint, Press_Input, Press_Output;

double Desired_Yaw;
double xaccel, yaccel, zaccel, heading, depth;

float gbias;
float abias;

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;
double base_altitude = 16.0;		// Altitude (in meters) of Piscataway NJ, 16m above sea level
double pressure_gage;

PID servoPID(&Servo_Input, &Servo_Output, &Servo_Setpoint, 1, .5, 0, REVERSE);
PID motorPID(&Motor_Input, &Motor_Output, &Motor_Setpoint, 1, .5, 0, DIRECT);
PID pressPID(&Press_Input, &Press_Output, &Press_Setpoint, 1, .5, 0, REVERSE);

double Left_Motor_Pin = 5; //UPDATE FOR CORRECT MOTOR PIN
double Right_Motor_Pin = 6; //UPDATE FOR CORRECT MOTOR PIN

File exocoetusData;


//Initializes and validates all sensors and components
//INCLUDE VAIDATION FOR PRESSURE TRANSDUCER
void initSensors()
{
	if (!accel.begin())
	{
		Serial.println(F("no LSM303 detected ... Check wiring!"));
		while (1);
	}
	if (!mag.begin())
	{
		Serial.println("no LSM303 detected ... Check wiring!");
		while (1);
	}
	if (!gyro.begin())
	{
		Serial.println("no L3GD20 detected ... Check wiring!");
		while (1);
	}
	servo1.attach(9);
	if (!servo1.attached())
	{
		Serial.println("no Servo detected ... Check wiring!");
		while (1);
	}

}


void setup(void)
{
	Serial.begin(14400);
	Serial.println(F("Exocoetus Data Log")); Serial.println("");

	while (!Serial) {
		; // wait for serial port to connect.
	}

	Serial.print("Initializing SD card...");
	pinMode(10, OUTPUT);
	if (!SD.begin(10)) {
		Serial.println("Card failed, or not present");
		//while (1);
	}
	Serial.println("SD card initialized.");

	Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading")); Serial.println("");

	/* Initialise the sensors and wait for sensors to stabalize*/
	initSensors();
	delay(500);

	sensors_event_t accel_event;
	sensors_event_t gyro_event;
	sensors_event_t mag_event;

	sensors_vec_t orientation;
	sensors_vec_t mag_orientation;

	/* Adjust abias and qbias so pitch is accurate*/
	gbias =1;
	kalmanY.setQbias(gbias);

	accel.getEvent(&accel_event);
	gyro.getEvent(&gyro_event);
	mag.getEvent(&mag_event);

	dof.accelGetOrientation(&accel_event, &orientation);
	dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &mag_orientation);

	kalmanY.setAngle(orientation.pitch);

	//gyro_data_roll = orientation.roll;
	timer = micros();

	Desired_Yaw = mag_orientation.heading;

	servo1.write(90);

	transducer.reset();		// reset/begin pressure transducer for operation
	transducer.begin();

	pressure_baseline = transducer.getPressure(ADC_4096);
	
	servoPID.SetMode(AUTOMATIC);
	motorPID.SetMode(DIRECT);
}


void loop(void)
{
	sensors_event_t mag_event;
	sensors_event_t accel_event;
	sensors_event_t gyro_event;

	sensors_vec_t orientation; //rename to accel orientation for clarification
	sensors_vec_t mag_orientation;


	mag.getEvent(&mag_event);		//measured in micro-Tesla (\muT) >> MAGNETIC FIELD STRENGTH, points to north
	accel.getEvent(&accel_event);	//measured in m/s^2 >> linear ACCELERATION
	gyro.getEvent(&gyro_event);		//measured in rad/s >> angular SPEED


	//dof.magTiltCompensation(SENSOR_AXIS_X, &mag_event, &accel_event); DO I NEED THIS

	dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &mag_orientation);
	dof.accelGetOrientation(&accel_event, &orientation);

	double dt = micros() - timer / 1000000; // Calculate delta time
	timer = micros();


	double gyroYrate = gyro_event.gyro.y *57.3 - 6.8; // Convert to deg/s


	kalAngleY = kalmanY.getAngle(orientation.pitch, gyroYrate, dt);

	heading = mag_orientation.heading;
	xaccel = accel_event.acceleration.x;
	yaccel = accel_event.acceleration.y;
	zaccel = accel_event.acceleration.z;

	// ---------------------- //
	// --- Pitch PID Part --- //
	// ---------------------- //
	Servo_Input = kalAngleY;							// pitch does not calculate well when IMU is updside down
	if (abs(Servo_Input) > 3) {
		if (Servo_Input > 0) {
			servoPID.SetOutputLimits(90, 180);		// arduino pwm limits are (0,255)
			servoPID.Compute();
			servo1.write(Servo_Output);
		}
		else {
			servoPID.SetOutputLimits(0, 90);
			servoPID.Compute();
			servo1.write(Servo_Output);
		}
	}
	else {
		servo1.write(90);
	}
	
	// ----------------------------- //
	// --- End of Pitch PID Part --- //
	// ----------------------------- //

	// -------------------- //
	// --- SD Card Part --- //
	// -------------------- //
	// Print data to serial port and write to SD card
	/*
	exocoetusData = SD.open("data.txt", FILE_WRITE);
	if (exocoetusData) {
	exocoetusData.print(kalAngleY); exocoetusData.print("   ");						// write pitch
	exocoetusData.print(heading); exocoetusData.print("   ");						// write heading
	//Serial.print("gyrorate:"); Serial.print(gyroYrate); Serial.print("   ");
	exocoetusData.print(xaccel); exocoetusData.print("   ");						//*
	exocoetusData.print(yaccel); exocoetusData.print("   ");						//* write xyz accels in m/s^2
	exocoetusData.print(zaccel); exocoetusData.print("   ");						//*
	exocoetusData.println("");
	exocoetusData.close();
	}
	*/
	// --------------------------- //
	// --- End of SD Card Part --- //
	// --------------------------- //
	
	//Print to serial port too
	//Serial.print("Pitch:"); Serial.print(kalAngleY); Serial.print("\t");						// print pitch
	//Serial.print("Yaw:"); Serial.print(mag_orientation.heading); Serial.print("\t");			// print heading
    Serial.print("gyrorate:"); Serial.print(gyroYrate); Serial.print("\t");
	Serial.print("X_accel:"); Serial.print(accel_event.acceleration.x); Serial.print("\t");	//*
	//Serial.print("Y_accel:"); Serial.print(accel_event.acceleration.y); Serial.print("\t");	//* printing xyz accels in m/s^2
	//Serial.print("Z_accel:"); Serial.print(accel_event.acceleration.z); Serial.print("\t");	//*
	//Serial.println("");																//print new line
	

	// --- Depth Measurement --- //
	pressure_abs = transducer.getPressure(ADC_4096);	// Reads pressure from the sensor in mbar (1 mbar = 100 Pa)
	pressure_gage = (pressure_abs * 100) - 101325;		// absolute pressure in Pa - atm pressure in Pa
	//depth = pressure_gage / (1000 * 9.81);				// depth = pressure/(density*gravity); density = 1000 kg/m^3, gravity = 9.81 m/s^2
	//Serial.print("Depth: "); Serial.print(depth); Serial.print("\t");

	/*
	// --- COMMENT OUT DEPTH PID PART IF YOU WANT THIS PROG. TO RUN DURING TEST OTHERWISE, SHIT WILL GET STUCK IN WHILE LOOP AND YOU WON'T GET OUTPUTS TO SERIAL MONITOR --- //
	// -----------------------//
	// --- Depth PID Part --- //	Input = pitch of Exocoetus, Setpoint = +-45 deg, Output = Some servo angle < 20 deg from 0 deg position
	// -----------------------//
	while (depth > 1.2 || depth < 0.8) {			// The idea is to pitch Exocoetus up or down at a certain angle when outside depth range
		Press_Input = kalAngleY;
		if (depth > 1.2) {
			Press_Setpoint = 45;							// is this pitch up?  Let's set a convention ASAP
			pressPID.SetOutputLimits(90, 120);
			pressPID.Compute();						// Setpoint = 45 deg; this seems steep, but just use this value for now
			servo1.write(Press_Output);

		}
		else if (depth < 0.8) {
			Press_Setpoint = -45;						// Setpoints found in this while loop can be stuffed into void_setup(), ex: Setpoint2 = 45;
			pressPID.SetOutputLimits(70, 90);			// Setpoint2 = Setpoint2 * -1;  The Setpoint2 declare in if-statement right above this isn't required then
			pressPID.Compute();
			servo1.write(Press_Output);
		}
		else {
			servo1.write(90);	// set servo to mid-point when within depth range, 0.8 < good < 1.2
		}
		
	}*/
	// ----------------------------- //
	// --- End of Depth PID Part --- //
	// ----------------------------- //
	
	// --- Altitude Measurement --- //
	altitude_delta = altitude(pressure_abs, pressure_baseline);		// altitude function below, outside void loop()
	Serial.print("Altitude: "); Serial.print(altitude_delta); Serial.print("\t");
	Serial.println("");



	// Motor PID
	Motor_Setpoint = 50; //Desired speed of motor
	analogWrite(Left_Motor_Pin, Motor_Setpoint);
	analogWrite(Right_Motor_Pin, Motor_Setpoint);
	Motor_Input = mag_orientation.heading;

	//Limit to 10 deg max deviation for safety and control

	if (abs(Motor_Input - Desired_Yaw) > 10) {
		// WRTIE CODE TO PRINT ERROR MESSAGE TO SD AND ILLUMINATE ERROR LIGHT
		analogWrite(Left_Motor_Pin, 0); 
		analogWrite(Right_Motor_Pin, 0);
		while (1);
	}

	else if (Motor_Input < 10) { Motor_Input = Motor_Input + 10; Desired_Yaw = Desired_Yaw + 10;}
	else if (Motor_Input > 350) { Motor_Input = Motor_Input - 10; Desired_Yaw = Desired_Yaw - 10;}

	if (Motor_Input > Desired_Yaw) {

		motorPID.SetOutputLimits(Motor_Setpoint, Motor_Setpoint + 10); //adjust pwm output
		motorPID.Compute();
		analogWrite(Right_Motor_Pin, Motor_Output);

	}

	else if (Motor_Input < Desired_Yaw) {

		motorPID.SetOutputLimits(Motor_Setpoint, Motor_Setpoint + 10); //adjust pwm output
		motorPID.Compute();
		analogWrite(Left_Motor_Pin, Motor_Output);

	}
	
	
}

double altitude(double P, double P0)								// found in pressure transducer example code in github
{
	return(44330.0*(1 - pow(P / P0, 1 / 5.255)));					// not sure where this formula comes from, but whatever
}