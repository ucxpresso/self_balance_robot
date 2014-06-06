/*
 * h_bridge.cpp
 *
 *  Created on: Jun 1, 2014
 *      Author: Jason
 */

#include "h_bridge.h"

HBridge::HBridge(PWM_CH_T pwm, PIN_NAME_T a, PIN_NAME_T b) :
CPwm(pwm),
m_pinA(a),
m_pinB(b)
{
	m_pinA.output();
	m_pinB.output();
	dutyCycle(0);
	direct(DIR_STOP);
}

void HBridge::direct(DIR_T dir) {
	switch(dir) {
	case DIR_STOP:
		m_pinA = LOW;
		m_pinB = LOW;
		break;
	case DIR_FORWARD:
		m_pinB = LOW;
		m_pinA = HIGH;
		break;
	case DIR_REVERSE:
		m_pinA = LOW;
		m_pinB = HIGH;
		break;
	}
}
