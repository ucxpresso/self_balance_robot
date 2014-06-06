/*
 * h_bridge.h
 *
 *  Created on: Jun 1, 2014
 *      Author: Jason
 */

#ifndef H_BRIDGE_H_
#define H_BRIDGE_H_

#include <class/pin.h>
#include <class/pwm.h>

typedef enum{
	DIR_STOP = 0,
	DIR_FORWARD = 1,
	DIR_REVERSE = 2
}DIR_T;

class HBridge: public CPwm {
public:
	HBridge(PWM_CH_T pwm, PIN_NAME_T a, PIN_NAME_T b);
	void direct(DIR_T dir);

	inline void operator = (DIR_T dir) {
		direct(dir);
	}

protected:
	CPin m_pinA;
	CPin m_pinB;
};

#endif /* H_BRIDGE_H_ */
