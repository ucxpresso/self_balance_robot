/*
 ===============================================================================
 Name        : main.cpp
 Author      :
 Version     : v1.0.0
 Date		 :
 Copyright   :
 Description :
 ===============================================================================
 History
 ---------+---------+--------------------------------------------+-------------
 DATE     |	VERSION |	DESCRIPTIONS							 |	By
 ---------+---------+--------------------------------------------+-------------

 ===============================================================================
 */

#include "uCXpresso.h"
#include "class/serial.h"
#include "class/usb_cdc.h"
#include "debug.h"

#ifdef DEBUG
#define DBG		dbg_printf
#else
#define DBG(...)
#endif

//
// TODO: insert other include files here
//
#include <stdlib.h>
#include <math.h>
#include <class/bus.h>
#include <class/timer.h>
#include <class/eeprom.h>
#include <class/mutex.h>
#include "h_bridge.h"
#include "PID_v1.h"
#include "sensor/mpu6050/MPU6050.h"

//
// Configuration
//
struct _config_{
	uint32_t length;
	float  roll_offset;
	double kp;
	double ki;
	double kd;
	float  pwm_min;
	float  pwm_max;
	float  pid_offset;

}PACK_STRUCT;
typedef struct _config_ CONFIG_T;

static CONFIG_T	config;

void setDefault() {
	config.length = sizeof(config);
	config.roll_offset = 0.0;
	config.kp = 7.5f;
	config.ki = 6.0f;
	config.kd = 2.0f;
	config.pwm_min = 0.00f;
	config.pwm_max = 0.72f;
	config.pid_offset = 1.0f;
}

//
//
//
CBus LEDs(LED1, LED2, LED3, LED4, END);

//
// TODO: insert other definitions and declarations here
//
//double Kp = 35.0f;		//proportional action
//double Ki = 1.0f;		//integral action
//double Kd = 2.5f;		//derivative action
//#define MAX_PWM	0.65f
//
//double Kp = 7.5f;		//proportional action
//double Ki = 12.5f;		//integral action
//double Kd = 0.5f;		//derivative action
//#define MAX_PWM	0.75f

#define ACCELEROMETER_SENSITIVITY 	8192.0
#define GYROSCOPE_SENSITIVITY 		65.536
#define M_PI 						3.14159265359f
#define RAD_TO_DEG 					57.295779513082320876798154814105f
#define	PID_SAMPLE_RATE				2		// 5ms#define DT							((float)PID_SAMPLE_RATE/1000.0)

class BalanceRobot: public CThread {
public:
	BalanceRobot(MPU6050 &sensor, HBridge &left, HBridge &right) :
	myPID(&m_input, &m_output, &m_setpoint, config.kp, config.ki, config.kd, DIRECT)
	{
		m_mpu = &sensor;
		m_left = &left;
		m_right = &right;
		m_setpoint = 0;
	}

	void stop() {
		m_left->dutyCycle(0);
		m_right->dutyCycle(0);
		m_left->direct(DIR_STOP);
		m_right->direct(DIR_STOP);
	}

	void tuings(double kp, double ki, double kd) {
		myPID.SetTunings(kp, ki, kd);
	}

	void setOutputLimits(float min, float max) {
		myPID.SetOutputLimits(min, max);
	}

	void setAccGryOffset(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy) {
		m_mxMPU.lock();
		m_mpu->setXAccelOffset(ax);
		m_mpu->setYAccelOffset(ay);
		m_mpu->setZAccelOffset(az);
		m_mpu->setYGyroOffset(gy);
		m_mxMPU.unlock();
	}


protected:
	PID myPID;
	MPU6050 *m_mpu;
	HBridge *m_left;
	HBridge *m_right;

	double m_setpoint;
	double m_input;
	double m_output;

	CMutex	m_mxMPU;

	void ComplementaryFilter(int16_t accData[3], int16_t gyrData[3],
			float *pitch, float *roll) {
		float pitchAcc, rollAcc;

		// Integrate the gyroscope data -> int(angularSpeed) = angle
		*pitch += ((float) gyrData[0] / GYROSCOPE_SENSITIVITY) * DT; // Angle around the X-axis
		*roll -= ((float) gyrData[1] / GYROSCOPE_SENSITIVITY) * DT; // Angle around the Y-axis

		// Compensate for drift with accelerometer data if !bullshit
		// Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
		int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1])
				+ abs(accData[2]);
		if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768) {
			// Turning around the X axis results in a vector on the Y-axis
			pitchAcc = atan2f((float) accData[1], (float) accData[2])
					* 180/ M_PI;
			*pitch = *pitch * 0.98 + pitchAcc * 0.02;

			// Turning around the Y axis results in a vector on the X-axis
			rollAcc = atan2f((float) accData[0], (float) accData[2])
					* 180/ M_PI;
			*roll = *roll * 0.98 + rollAcc * 0.02;
		}
	}

	virtual void run() {
		int16_t accData[3];
		int16_t gyrData[3];

		CTimer tm(TIMER0);
		tm.second(DT);
		tm.enable();

		myPID.SetMode(AUTOMATIC);
		myPID.SetOutputLimits(config.pwm_min, config.pwm_max);
		myPID.SetSampleTime(PID_SAMPLE_RATE);

		float roll, pitch;
		roll = 0.0f;
		pitch = 0.0f;

		//
		// loop
		//
		while (isAlive()) {
			if (tm.wait()) {
				//
				// read sensors
				//
				m_mxMPU.lock();
				m_mpu->getMotion6(&accData[0], &accData[1], &accData[2],
						&gyrData[0], &gyrData[1], &gyrData[2]);
				m_mxMPU.unlock();

				//
				// filter
				//
				ComplementaryFilter(accData, gyrData, &pitch, &roll);

				roll -= config.roll_offset;

				//
				// PID Control
				//
				if (roll < -(config.pid_offset) ) {
					LEDs[1] = LED_ON;
					LEDs[2] = LED_OFF;

					m_input = roll;
					myPID.SetControllerDirection(DIRECT);
					myPID.Compute();

					m_left->direct(DIR_FORWARD);
					m_right->direct(DIR_FORWARD);

				} else if ( roll> config.pid_offset) {
					LEDs[1] = LED_OFF;
					LEDs[2] = LED_ON;

					m_input = roll;
					myPID.SetControllerDirection(REVERSE);
					myPID.Compute();

					m_left->direct(DIR_REVERSE);
					m_right->direct(DIR_REVERSE);

				} else {
					LEDs[1] = LED_OFF;
					LEDs[2] = LED_OFF;
					m_input = 0;
					myPID.Compute();
					m_output = 0;
				}
				m_left->dutyCycle(m_output);
				m_right->dutyCycle(m_output);
			}
		}
	}
};

//
// Menu for Configuration
//
#ifndef DEBUG
#include <class/console.h>
class myMenu: public CThread {
public:
	myMenu(MPU6050 &mpu, BalanceRobot &robot) {
		m_mpu = &mpu;
		m_robot = &robot;
	}

	virtual bool start() {
		return CThread::start("menu", 256);
	}

protected:
	usbCDC	m_usb;
	Console m_con;
	MPU6050 *m_mpu;
	BalanceRobot *m_robot;

	//
	// Implement the CThread::run() virtual function
	//
	virtual void run() {
		m_con.assign(m_usb, m_usb);
		m_usb.enable();

		while(1) {
			if ( m_usb.isConnected() ) {
				show_welcome();
			}
			LEDs[3] = !LEDs[3];
			sleep(200);
		}
	}

	void show_welcome() {
		while(m_usb.isConnected()) {
			m_con.clear();
			m_con << "****************************************" << endl;
			m_con << "*     Welcome to Self-Balance Robot    *" << endl;
			m_con << "*               ver 1.0.0              *" << endl;
			m_con << "****************************************" << endl;
			m_con << "[1] Gyroscope & Accelerometer calibration" << endl;
			m_con << "[2] PID Control tuning" << endl;
			m_con << "[3] Save changed" << endl;
			m_con << "[4] Load Default" << endl;
			switch(m_con.getc()) {
			case '1' :
				show_mpu6050();
				break;
			case '2' :
				show_pid();
				break;
			case '3' :
				saveConfigure();
				break;
			case '4' :
				setDefault();
				break;
			case 'H':
				m_con.printf("High-Water Mark:%d\n", getStackHighWaterMark());
				m_con.getc();
				break;
			}
		}
	}

	void show_mpu6050() {
		while( m_usb.isConnected() ) {
			m_con.clear();
			m_con << "****************************************" << endl;
			m_con << "* Gyroscope & Accelerometer calibration*" << endl;
			m_con << "****************************************" << endl;
			m_con.printf("[1] Set roll offset (%0.4f)\n", config.roll_offset);
			m_con << "[ESC] Return" << endl;

			switch( m_con.getc() ) {
			case '1':
				m_con << "Input roll offset:" << flush;
				config.roll_offset = m_usb.parseFloat(true);
				break;
			case 0x1B:
				return;
			}
		}
	}

	void show_pid() {
		while( m_usb.isConnected() ) {
			m_con.clear();
			m_con << "****************************************" << endl;
			m_con << "*          PID Control tuning          *" << endl;
			m_con << "****************************************" << endl;
			m_con.printf("[1] Kp (%0.3f)\n", config.kp);
			m_con.printf("[2] Ki (%0.3f)\n", config.ki);
			m_con.printf("[3] Kd (%0.3f)\n", config.kd);
			m_con.printf("[4] Min. PWM (%0.2f)\n", config.pwm_min);
			m_con.printf("[5] Max. PWM (%0.2f)\n", config.pwm_max);
			m_con.printf("[6] Filter Offset (%0.4f)\n", config.pid_offset);
			m_con << "[ESC] Return" << endl;
			switch(m_con.getc()) {
			case '1':
				m_con << "Input Kp:" << flush;
				config.kp = m_usb.parseFloat(true);
				m_robot->tuings(config.kp, config.ki, config.kd);
				break;
			case '2':
				m_con << "Input Ki:" << flush;
				config.ki = m_usb.parseFloat(true);
				m_robot->tuings(config.kp, config.ki, config.kd);
				break;
			case '3':
				m_con << "Input Kd:" << flush;
				config.kd = m_usb.parseFloat(true);
				m_robot->tuings(config.kp, config.ki, config.kd);
				break;
			case '4':
				m_con << "Input Min. PWM:" << flush;
				config.pwm_min = m_usb.parseFloat(true);
				m_robot->setOutputLimits(config.pwm_min, config.pwm_max);
				break;
			case '5':
				m_con << "Input Max. PWM:" << flush;
				config.pwm_max = m_usb.parseFloat(true);
				m_robot->setOutputLimits(config.pwm_min, config.pwm_max);
				break;
			case '6':
				m_con << "Input Pid Offset:" << flush;
				config.pid_offset = m_usb.parseFloat(true);
				break;
			case 0x1B:
				return;
			}
		}
	}

	void saveConfigure() {
		EEPROM::write(0, &config, sizeof(config));
		m_con << "Save finished." << endl;
		m_con.getc();
	}

};

#endif


//
// main task
//
int main(void) {

#ifdef DEBUG
#if __USE_USB
	usbCDC ser;
	ser.connect();
#else
	CSerial ser;
	ser.settings(115200);
#endif
	CDebug dbg(ser);
	dbg.start();
#endif

	/*************************************************************************
	 *
	 *                         your setup code here
	 *
	 **************************************************************************/
	//
	// Load Configuration
	//
	EEPROM::read(0, &config, sizeof(config));
	if ( config.length!=sizeof(config) ) {
		setDefault();
	}

	// class default I2C address is 0x68
	// specific I2C addresses may be passed as a parameter here
	// AD0 low = 0x68 (default for InvenSense evaluation board)
	// AD0 high = 0x69
	MPU6050 accelgyro;

	// initialize device
	accelgyro.initialize();
	accelgyro.setRate(7);
	accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
	accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

	//
	// check device
	//
	if (accelgyro.testConnection()) {
	}

	//
	// H-Bridge
	//
	CPwm::frequency(KHZ(20));

	HBridge left(PWM1, P18, P19);
	HBridge right(PWM2, P22, P23);

	left.enable();
	right.enable();

	BalanceRobot robot(accelgyro, left, right);
	robot.start("Robot", 256, PRI_HIGH);

#ifndef DEBUG
	myMenu menu(accelgyro, robot);
	menu.start();
#endif

	while (1) {
		/**********************************************************************
		 *
		 *                         your loop code here
		 *
		 **********************************************************************/
		LEDs[0] = !LEDs[0];
		sleep(200);
	}
	return 0;
}

//
// default memory pool
//
static uint8_t mem_pool[DEFAULT_POOL_SIZE];

//
// setup before the system startup
//
extern "C" void sys_setup(void) {
	pool_memadd((uint32_t) mem_pool, sizeof(mem_pool));
#if __USE_USB==0
	pool_memadd(USB_MEM_BASE, USB_MEM_SIZE);
#endif
}

