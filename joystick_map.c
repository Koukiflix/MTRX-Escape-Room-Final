#include "joystick_map.h"

int updateValues(int adc_input, int axis_input) {
	// If the ADC input is +ve then:
	if (adc_input > 4000 && axis_input <= 3000) {
		if (axis_input + INCREMENT_SERVO > 3000) {
			return 3000;
		}
		else {
			return axis_input + INCREMENT_SERVO;
		}

	}

	else if (adc_input < 2000  && axis_input >= 0) {
		if (axis_input - INCREMENT_SERVO < 0) {
			return 0;
		}
		else {
			return axis_input - INCREMENT_SERVO;
		}

	}

	else {
		return axis_input;
	}
}
