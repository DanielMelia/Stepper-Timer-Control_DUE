#include "Arduino.h"
#include "TimerStepperDue.h"
#include "limits.h"

// ------------------------------------------- RESOURCES -------------------------------------------------------------- //

// - Blog : Arduino Due Timers (Part 1) : http://ko7m.blogspot.com/2015/01/arduino-due-timers-part-1.html
// - Application Note : AVR446: Linear speed control of stepper motor
//		- Doc : https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf
//		- Code : https://github.com/taitpthomas/AVR446


//------------------------------------------ CONTROL MODES -------------------------------------------------------------//
#define STP 0  //STOP mode. Motion not allowed
#define HOM 1
#define POS 2  //POSITION mode: the user inputs the amount of rotation and trajectory parameters (vel, acc, dec...)
#define VEL 3  //VELOCITY mode: or speed control mode. Useful for Joystick control for example. Inputs are velocity and direction

TimerStepperDue timerSteppers[9];

TimerStepperDue Timer1_Stepper;
TimerStepperDue Timer2_Stepper;
TimerStepperDue Timer3_Stepper;
TimerStepperDue Timer4_Stepper;
TimerStepperDue Timer5_Stepper;
TimerStepperDue Timer6_Stepper;
TimerStepperDue Timer7_Stepper;
TimerStepperDue Timer8_Stepper;
TimerStepperDue Timer9_Stepper;

void TimerStepperDue::InitialiseTimers() {

	int st_count = sizeof(timerSteppers) / sizeof(timerSteppers[0]);  // sizeof operator returns the size of the object in bytes
	//Serial.println(st_count);
	for (int i = 0; i < st_count; i++) {
		timerSteppers[i].motor_id = i;
		if (i == 0) {
			timerSteppers[i].ConfigureTimer(TC0, 0, TC0_IRQn);			
		}
		else if (i == 1) {
			timerSteppers[i].ConfigureTimer(TC0, 1, TC1_IRQn);
		}
		else if (i == 2) {
			timerSteppers[i].ConfigureTimer(TC0, 2, TC2_IRQn);
		}
		else if (i == 3) {
			timerSteppers[i].ConfigureTimer(TC1, 0, TC3_IRQn);
		}
		else if (i == 4) {
			timerSteppers[i].ConfigureTimer(TC1, 1, TC4_IRQn);
		}
		else if (i == 5) {
			timerSteppers[i].ConfigureTimer(TC1, 2, TC5_IRQn);
		}
		else if (i == 6) {
			timerSteppers[i].ConfigureTimer(TC2, 0, TC6_IRQn);
		}
		else if (i == 7) {
			timerSteppers[i].ConfigureTimer(TC2, 1, TC7_IRQn);
		}
		else if (i == 8) {
			timerSteppers[i].ConfigureTimer(TC2, 2, TC8_IRQn);
		}
	}

	//timerSteppers[0].ConfigureTimer(TC0, 0, TC0_IRQn);
	//timerSteppers[1].ConfigureTimer(TC0, 1, TC1_IRQn);
	//timerSteppers[2].ConfigureTimer(TC0, 2, TC2_IRQn);

	//timerSteppers[3].ConfigureTimer(TC1, 0, TC3_IRQn);
	//timerSteppers[4].ConfigureTimer(TC1, 1, TC4_IRQn);
	//timerSteppers[5].ConfigureTimer(TC1, 2, TC5_IRQn);

	//timerSteppers[6].ConfigureTimer(TC2, 0, TC6_IRQn);
	//timerSteppers[7].ConfigureTimer(TC2, 1, TC7_IRQn);
	//timerSteppers[8].ConfigureTimer(TC2, 2, TC8_IRQn);

}

// Timer Start / Update / Stop Generic Functions 
// C:\...\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\system\libsam\include\tc.h
#pragma region Timer Start / Update / Stop Generic Functions
void TimerStepperDue::TimerStart(Tc* tc, uint32_t channel, IRQn_Type irq, uint32_t rc) {
	// Tell the Power Management Controller to disable the write protection of the (Timer/Counter) registers:
	pmc_set_writeprotect(false);
	// Enable clock for the timer
	pmc_enable_periph_clk(irq);

	// Set up the Timer in waveform mode which creates a PWM in UP mode with automatic trigger on RC Compare
	// and sets it up with the determined internal clock as clock input.

	// A 32-bit unsigned integer has a range of 0 - 4294967295
	//PRESCALERS:
	// - TIMER_CLOCK1 - MCK/2   - 42MHz
	// - TIMER_CLOCK2 - MCK/8   - 10.5MHz		--> 0.09us to 409 seconds timer period
	// - TIMER_CLOCK3 - MCK/32  - 2.652MHz
	// - TIMER_CLOCK4 - MCK/128 - 656.25KHz
	// - TIMER_CLOCK5 - SLCK    - 32KHz
	// *MCK - Master Clock (84MHz)
	TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
	//freq --> desired blinking frequency (in Hz)
	//uint32_t rc = VARIANT_MCK / _prescaler / freq;
	//Creates a wafeform that goes high at RA and low at RC
	//TC_SetRA(tc, channel, rc >> 1); // 50% duty cycle square wave
	TC_SetRC(tc, channel, rc);
	TC_Start(tc, channel);
	//interrups occurs only when counter reaches RC:
	tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
	tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;

	NVIC_EnableIRQ(irq);
}
void TimerStepperDue::TimerUpdate(Tc* tc, uint32_t channel, IRQn_Type irq, uint32_t rc) {
	//uint32_t rc = VARIANT_MCK / _prescaler / freq;
	TC_SetRC(tc, channel, rc);
	TC_Start(tc, channel);
	NVIC_EnableIRQ(irq);
}
void TimerStepperDue::TimerStop(Tc* tc, uint32_t channel, IRQn_Type irq) {
	NVIC_DisableIRQ(irq);
	TC_Stop(tc, channel);
}

#pragma endregion


// SETUP FUNCTIONS
#pragma region SETUP FUNCTIONS
void TimerStepperDue::MotorSetup(int pulse_pin, int dir_pin, int ena_pin, int ppr, bool linear_stage, float pitch) {
	// Configure Microcontroller Pins
	pinMode(pulse_pin, OUTPUT);
	pinMode(dir_pin, OUTPUT);
	pinMode(ena_pin, OUTPUT);
	_pulse_pin = pulse_pin;
	_dir_pin = dir_pin;
	_ena_pin = ena_pin;

	DisableMotor(false);

	// Configure Axis
	_axis_isLinear = linear_stage;
	_ppr = ppr;
	_step_size_rad = (2 * PI) / ppr;
	if (_axis_isLinear) {
		_pitch = pitch;
		_step_size_mm = _pitch / ppr;
	}

	// By default set limit switches pins to -1 to indicate they are not configured
	_home_limit_sw_pin = -1;
	_far_limit_sw_pin = -1;
	
	// Initialise Position Variables
	_step_position = 0;
	_min_step_position = LONG_MIN;
	_max_step_position = LONG_MAX;

}
void TimerStepperDue::LimitSwtichesSetup(int home_ls_pin, int far_ls_pin, float min_pos_value, float max_pos_value) {
	// Configure Microcontroller Pins
	pinMode(home_ls_pin, INPUT_PULLUP);
	pinMode(far_ls_pin, INPUT_PULLUP);
	_home_limit_sw_pin = home_ls_pin;
	_far_limit_sw_pin = far_ls_pin;

	_min_position = min_pos_value;
	_max_position = max_pos_value;

	if (!_axis_isLinear) {	// Rotational axis
		_max_step_number = ((_max_position - _min_position) / _step_size_rad);
	}
	else {	// Linear axis
		_max_step_number = ((_max_position - _min_position) / _step_size_mm);
	}

}
void TimerStepperDue::DisableMotor(bool disable) {
	if (disable) {
		digitalWrite(_ena_pin, HIGH);
	}
	else {
		digitalWrite(_ena_pin, LOW);
	}
}
void TimerStepperDue::TimerSetup(int timerNumber, int ps)
{

	if (timerNumber == 1) {
		_timer = TC0;
		_channel = 0;
		_irq = TC0_IRQn;
	}
	else if (timerNumber == 2) {
		_timer = TC0;
		_channel = 1;
		_irq = TC1_IRQn;
	}
	else if (timerNumber == 3) {
		_timer = TC0;
		_channel = 2;
		_irq = TC2_IRQn;
	}
	else if (timerNumber == 4) {
		_timer = TC1;
		_channel = 0;
		_irq = TC3_IRQn;
	}
	else if (timerNumber == 5) {
		_timer = TC1;
		_channel = 1;
		_irq = TC4_IRQn;
	}
	else if (timerNumber == 6) {
		_timer = TC1;
		_channel = 2;
		_irq = TC5_IRQn;
	}
	else if (timerNumber == 7) {
		_timer = TC2;
		_channel = 0;
		_irq = TC6_IRQn;
	}
	else if (timerNumber == 8) {
		_timer = TC2;
		_channel = 1;
		_irq = TC7_IRQn;
	}
	else if (timerNumber == 9) {
		_timer = TC2;
		_channel = 2;
		_irq = TC8_IRQn;
	}

	_prescaler = 8;
	_max_wf_freq = VARIANT_MCK / (2 * _prescaler);

}
void TimerStepperDue::ConfigureTimer(Tc* tc, uint32_t channel, IRQn_Type irq) {
	_timer = tc;
	_channel = channel;
	_irq = irq;

	_prescaler = 8;
	_max_wf_freq = VARIANT_MCK / (2 * _prescaler);

}
void TimerStepperDue::TimerTrigger(uint32_t rc)
{

	_count = 0;
	_moving = true;  // Indicate that motion started

	TimerStart(_timer, _channel, _irq, rc);

}
void TimerStepperDue::SetMicrostepping(int ms) {
	switch (ms) {
	case 256: _ppr = 200 * ms; break;
	case 128: _ppr = 200 * ms; break;
	case  64: _ppr = 200 * ms; break;
	case  32: _ppr = 200 * ms; break;
	case  16: _ppr = 200 * ms; break;
	case   8: _ppr = 200 * ms; break;
	case   4: _ppr = 200 * ms; break;
	case   2: _ppr = 200 * ms; break;
	case   0: _ppr = 200; break;
	default: break;
	}
}
#pragma endregion

// RUN MOTOR FUNCTIONS
#pragma region RUN MOTOR FUNCTIONS
void TimerStepperDue::Home(int freq, bool dir) {

	if (_far_limit_sw_pin == -1 && _home_limit_sw_pin == -1){ return; }

	set_direction(dir);
	_f_trg = freq;
	
	// Initialise the Timer
	_control_mode = HOM;
	unsigned int compare_value = (unsigned int)(_max_wf_freq / (long)_f_trg);

	_motor_homed = false;
	_motor_homed_phase_1 = false;
	_H_homing_counter = 0;
	_F_homing_counter = 0;
	_H_F_sensor_triggered = false;

	TimerTrigger(compare_value);

}
void TimerStepperDue::Home_ovr(float position) {
	if (position > _max_position) { position = _max_position; }
	if (position < _min_position) { position = _min_position; }
	_position = position;
	_step_position = long((_position / (_max_position - _min_position)) * (float)_max_step_number);
	_min_step_position = 0;
	_max_step_position = _max_step_number;
	_motor_homed = true;
}
void TimerStepperDue::RotateSteps(int steps, bool dir, int freq, int acc_steps, int dec_steps) {

	if (acc_steps < 0) {
		acc_steps = 0;
	}
	else if (acc_steps > steps) {
		acc_steps = steps;
	}

	if (dec_steps < 0) {
		dec_steps = 0;
	}
	else if (dec_steps > steps) {
		dec_steps = steps;
	}

	if (acc_steps + dec_steps > steps) {
		acc_steps = int(steps / 2);
		dec_steps = int(steps / 2);
	}

	// Update trapezoidal profile parameters
	_total_steps = steps;
	_acc_end = acc_steps;
	_dec_start = steps - dec_steps;
	_f_trg = freq;

	// Set motor direction
	set_direction(dir);

	// Calculate first timer/counter value

	if (acc_steps != 0) {
		float acc = convert_AccSteps_2_AccRads2(freq, acc_steps);
		_c0 = (int)(0.676 * _max_wf_freq * sqrt(2 * _step_size_rad / acc));
	}
	else {
		_c0 = _max_wf_freq / _f_trg;
	}

	// Initialise the Timer
	_control_mode = POS;
	TimerTrigger(_c0);
}
void TimerStepperDue::RotateAngle(float angle, float vel, float acc, float dec) {

	if (vel <= 0 || acc < 0 || dec < 0) {
		return;
	}

	// Calculate trapezoidal profile parameters
	_total_steps = round(abs(angle) / _step_size_rad);  // Calculate number of steps to reach target angle
	_f_trg = (int)(vel / _step_size_rad);                    // Calculate targer step frequency

	long acc_steps = (vel * vel) / (2 * _step_size_rad * acc);  // Number of steps in acceleration mode
	long acc_lim = (_total_steps * dec) / (acc + dec);      // Maximum number of steps allowed

	// If acceleration limit is reached before maximum speed
	if (acc_lim <= acc_steps) {
		_acc_end = acc_lim;          // Calculate steps to acceleration end
		_dec_start = acc_lim + 1;    // Calculate steps to deceleration start
	}
	// If maximum speed is reached before acceleration limit
	else {
		long decel_val = -acc_steps * acc / dec;
		_acc_end = acc_steps;                    // Calculate steps to acceleration end
		_dec_start = _total_steps + decel_val;    // Calculate steps to deceleration start
	}

	if (acc == 0) { _acc_end = 0; }
	if (dec == 0) { _dec_start = _total_steps; }

	// Calculate first timer/counter 	
	if (acc != 0) {
		_c0 = (int)(0.676 * _max_wf_freq * sqrt(2 * _step_size_rad / acc));
	}
	else {
		_c0 = _max_wf_freq / _f_trg;

	}

	// Set motor direction
	bool dir;
	if (angle >= 0) {
		dir = true;
	}
	else {
		dir = false;
	}
	set_direction(dir);
	// Initialise Timer
	_control_mode = POS;
	TimerTrigger(_c0);
}
void TimerStepperDue::RunLinearStage_abs(float abs_trgPos_mm, float vel_mms, float acc_mms2, float dec_mms2) {

	float _curr_pos = GetPosition_mm_rad();

	if (abs_trgPos_mm > _max_position || abs_trgPos_mm < _min_position) { return; }
	if (_curr_pos > _max_position || _curr_pos < _min_position) { return; }
	
	float _dist = abs_trgPos_mm - _curr_pos;

	RunLinearStage(_dist, vel_mms, acc_mms2, dec_mms2);

}
void TimerStepperDue::RunLinearStage(float distance_mm, float vel_mms, float acc_mms2, float dec_mms2) {
	if (_pitch == 0) { return; }
	// Calculate the amount of angular rotation in radians to cover the linear distance
	float angle = 2 * PI * (distance_mm / _pitch);
	// Run motor
	float vel_rads = vel_mms * 2 * PI / _pitch;
	float acc_rads2 = acc_mms2 * 2 * PI / _pitch;
	float dec_rads2 = dec_mms2 * 2 * PI / _pitch;

	RotateAngle(angle, vel_rads, acc_rads2, dec_rads2);
}
void TimerStepperDue::RunLinearStageWithVelocity(bool direction, float vel_mms, float acc_mms2) {
	if (_pitch == 0) { return; }
	//Calculate step frequency for target velocity
	float vel_rads = vel_mms * 2 * PI / _pitch;
	_f_trg = _ppr * vel_rads / (2 * PI);
	int acc_steps;
	if (acc_mms2 != 0) {
		float acc_rads2 = acc_mms2 * 2 * PI / _pitch;
		acc_steps = convert_AccRads2_2_AccSteps(acc_rads2, vel_rads);
	}
	else {
		acc_steps = 0;
	}
	RunSpeed(_f_trg, acc_steps, direction);

}
void TimerStepperDue::RunSpeed(int freq, int acc_steps, bool dir) {
	// Update trapezoidal profile parameters
	_f_trg = freq;
	_acc_end = acc_steps;

	// Set motor direction
	set_direction(dir);

	// Calculate first timer/counter value
	if (acc_steps != 0) {
		float acc = convert_AccSteps_2_AccRads2(freq, acc_steps);
		_c0 = (int)(0.676 * _max_wf_freq * sqrt(2 * _step_size_rad / acc));
	}
	else {
		_c0 = _max_wf_freq / _f_trg;
	}
	_control_mode = VEL;
	_vm_stop = false;
	TimerTrigger(_c0);
}
void TimerStepperDue::StopSpeedMode(int steps) {
	if (_control_mode == VEL){
		_vm_stop = true;
	}
	else if (_control_mode == POS) {
		_dec_start = _count;
	}

	_total_steps = _count + steps;
	
}
void TimerStepperDue::StopMotor() {

	if (_pulse_pin_state == HIGH) {
		Step();
		_pulse_pin_state = LOW;
	}
	_control_mode = STP;
	_moving = false;
	_finished = true;
	TimerStop(_timer, _channel, _irq);
}
void TimerStepperDue::ResetPosition() {
	_motor_homed = false;
	_position = -999;
}
void TimerStepperDue::StopWithDeceleration(float dec_mms2, float init_vel_mms) {

	int dec_steps;
	if (dec_mms2 != 0) { 
		float init_vel_rads = init_vel_mms * 2 * PI / _pitch;
		float dec_rads2 = dec_mms2 * 2 * PI / _pitch;
		dec_steps = convert_AccRads2_2_AccSteps(dec_rads2, init_vel_rads);
	}
	else { dec_steps = 0; }
	StopSpeedMode(dec_steps);
}

#pragma endregion

// FEEDBACK FUNCTIONS
#pragma region FEEDBACK FUNCTIONS
bool TimerStepperDue::IsMoving() {
	return _moving;
}
bool TimerStepperDue::IsHomed() {
	return _motor_homed;
}
byte TimerStepperDue::GetCurrentControlMode() {
	return _control_mode;
}
long TimerStepperDue::GetStepPosition() {
	return _step_position;
}
float TimerStepperDue::GetPosition_mm_rad() {
	if (!_motor_homed) { return -999; }
	if (!_axis_isLinear) {
		_position = (float)(_step_position * _step_size_rad);
	}
	else {
		_position = (float)(_step_position * _step_size_mm);
	}
	
	return _position;

}
byte TimerStepperDue::CheckLimitSwitches() {
	//NOTE : "homing_counter" to deal with false LimSw positives - switch must read 1 for three consecutive times to stop motor
	if (_far_limit_sw_pin == -1 && _home_limit_sw_pin == -1) { return 0; }

	_far_limit_sw_value = digitalRead(_far_limit_sw_pin);
	_home_limit_sw_value = digitalRead(_home_limit_sw_pin);

	if (_home_limit_sw_value && _far_limit_sw_value) {	//BOTH			
		return 1;
	}
	else if(_home_limit_sw_value) {	//HOME 
		if (_control_mode != HOM) {
			_H_homing_counter += 1;
			//if (_dir == -1 && _H_homing_counter > 2) { StopMotor(); }
		}	
		return 2;
	}
	else if (_far_limit_sw_value) {	//FAR
		if (_control_mode != HOM) {
			_F_homing_counter += 1;
			//if (_dir == 1 && _F_homing_counter > 2) { StopMotor(); }
		}	
		return 3;
	}
	else {	//NONE
		if (_control_mode != HOM) {
			_H_homing_counter = 0;
			_F_homing_counter = 0;
		}
		return 0;
	}
}
String TimerStepperDue::PassMotorParametersToString() {
	String tempStr = String(_pulse_pin) + ";" + String(_dir_pin) + ";" + String(_ena_pin) + ";" + String(_home_limit_sw_pin) + ";" + String(_far_limit_sw_pin) + ";" +
		String(_ppr) + ";" + String(_pitch, 1) + ";" + String(_max_position, 1) + ";";
	return tempStr;

}
String TimerStepperDue::PassMotorParametersAndIdToString() {
	String tempStr = String(motor_id) + ";" + String(_pulse_pin) + ";" + String(_dir_pin) + ";" + String(_ena_pin) + ";" + String(_home_limit_sw_pin) + ";" + 
		String(_far_limit_sw_pin) + ";" + String(_ppr) + ";" + String(_pitch, 1) + ";" + String(_max_position, 1) + ";";		
	return tempStr;

}
#pragma endregion

// LOW-LEVEL FUNCTIONS
#pragma region LOW-LEVEL FUNCTIONS
void TimerStepperDue::set_direction(bool dir) {
	if (dir) {
		digitalWrite(_dir_pin, HIGH);
		//_dir_pin_state = true;
		_dir = 1;
	}
	else {
		digitalWrite(_dir_pin, LOW);
		//_dir_pin_state = false;
		_dir = -1;
	}
}
float TimerStepperDue::convert_AccSteps_2_AccRads2(int freq, int acc_steps) {
	// NOTE: try setting v_0 = 0 and see the difference in results
	float v_0 = (float(freq) / 10) * _step_size_rad; // Calculate initial, final and average velocities
	float v_f = freq * _step_size_rad;
	float v_avg = (v_f + v_0) / 2;
	float travel = acc_steps * _step_size_rad;       // Travel (rad) at the end of acceleration phase
	float time = travel / v_avg;                // Time taken for the motor to reach the end of the acceleration phase
	float acc = (v_f - v_0) / time;             // Acceleration value (rad/s2)

	return acc;

}
int TimerStepperDue::convert_AccRads2_2_AccSteps(float acc, float vel) {
	// NOTE: try setting v_0 = 0 and see the difference in results
	float v_0 = vel / 10;
	float v_avg = (vel + v_0) / 2;
	float time = (vel - v_0) / acc;
	float travel = time * v_avg;
	int acc_steps = travel / _step_size_rad;

	return acc_steps;
}
void TimerStepperDue::Step() {
	if (_pulse_pin_state) {
		digitalWrite(_pulse_pin, LOW);
		_pulse_pin_state = LOW;
	}
	else {
		digitalWrite(_pulse_pin, HIGH);
		_count++;
		_step_position += _dir;
		if(_motor_homed){
			//if (!_axis_isLinear) {
			//	_position += _dir * _step_size_rad;
			//}
			//else {
			//	_position += _dir * _step_size_mm;
			//}			
		}
		_pulse_pin_state = HIGH;
	}
}
#pragma endregion

// INTERRUPT SERVICE ROUTINE FUNCTION
#pragma region INTERRUPT SERVICE ROUTINE FUNCTION
void TimerStepperDue::TimerMotor_Run() {

	if (_motor_homed) {
		if ((_step_position > _max_step_position && _dir == 1) || (_step_position < _min_step_position && _dir == -1)) {
			if (!_vm_stop) {
				StopSpeedMode(_ppr/5);
			}
		}
	}

	switch (_control_mode) {
		
		// If Motor in STOP MODE:
	case STP:
		if (_pulse_pin_state == HIGH) {
			Step();
		}
		_moving = false;
		_finished = true;
		_control_mode = STP;
		TimerStop(_timer, _channel, _irq);
		break;
	case HOM:
		_far_limit_sw_value = digitalRead(_far_limit_sw_pin);
		_home_limit_sw_value = digitalRead(_home_limit_sw_pin);
		if (!_motor_homed_phase_1) {
			Step();
			if (_far_limit_sw_value == HIGH && _home_limit_sw_value == HIGH) {	// KEEP MOVING - None of the Limit Switches have been triggered
				_H_homing_counter = 0;
				_F_homing_counter = 0;
			}
			else {
				if (_home_limit_sw_value == LOW && _far_limit_sw_value == LOW) {	// ERROR - Both Limit Switches Activated
					_H_homing_counter += 1;
					_F_homing_counter += 1;
				}
				else if (_home_limit_sw_value == LOW) {	// HOMED - Home Limit Switch Activated. Count 10 consicutive times to avoid false positives
					_H_homing_counter += 1;
				}
				else if (_far_limit_sw_value == LOW) {	// HOMED - Far Limit Switch Activated. Count 10 consicutive times to avoid false positives
					_F_homing_counter += 1;
				}

				if (_H_homing_counter > 10) {
					_H_F_sensor_triggered = false;
				}
				else if (_F_homing_counter > 10) {
					_H_F_sensor_triggered = true;
				}
				if (_H_homing_counter > 10 || _F_homing_counter > 10) {
					_motor_homed_phase_1 = true;
					// If a limit switch has been hit, reverse direction and move slowly until the switch is not pressed
					
					int slow_freq = 100;	//Set final frequency
					unsigned int compare_value = (unsigned int)(_max_wf_freq / slow_freq);
					
					bool tempDir;	// Reverse direction
					if (_dir == 1) { tempDir = false; }
					else { tempDir = true; }				
					set_direction(tempDir);
					
					TimerUpdate(_timer, _channel, _irq, compare_value);	// Update Timer
					_H_homing_counter = 0;
					_F_homing_counter = 0;
				}
				// STOP

			}
		}
		else {
			Step();
			if (!_H_F_sensor_triggered && _home_limit_sw_value == HIGH) {
				_H_homing_counter += 1;
			}
			else if (_H_F_sensor_triggered && _far_limit_sw_value == HIGH) {
				_F_homing_counter += 1;
			}
			else {
				_H_homing_counter = 0;
				_F_homing_counter = 0;
			}
			if (_H_homing_counter > 10 || _F_homing_counter > 10) {
				if (_H_F_sensor_triggered) {
					_position = _max_position;
					_step_position = 0;
					_max_step_position = _step_position;
					_min_step_position = -_max_step_number;
				}
				else {
					_position = _min_position;
					_step_position = 0;
					_min_step_position = _step_position;
					_max_step_position = _max_step_number;
				}
				// Motor homed!
				_motor_homed = true;
				_control_mode = STP;
				_moving = false;
				_finished = true;
				TimerStop(_timer, _channel, _irq);
			}
		}

		break;
		// If Motor in POSITION MODE:
	case POS:
		if (_count < _total_steps) {

			if (_count == 0 && _pulse_pin_state == LOW) {
				unsigned int compare_value = (int)_c0;
				TimerUpdate(_timer, _channel, _irq, compare_value);
				_prev_counter_value = compare_value;
				_rest = 0;				
			}
			//ACCELERATION PHASE
			else if (_count < _acc_end && _pulse_pin_state == LOW) {
				unsigned int compare_value = _prev_counter_value - ((2 * _prev_counter_value + _rest) / (4 * _count + 1));
				TimerUpdate(_timer, _channel, _irq, compare_value);
				_prev_counter_value = compare_value;
				//% = compound remainder: calculate the remainder when one integer is divided by another
				if (_count != (_acc_end - 1)) {
					_rest = (2 * _prev_counter_value + _rest) % (4 * _count + 1);
				}
				else {
					_rest = 0;
				}				
			}
			// CONSTANT VELOCITY PHASE
			else if (_count < _dec_start && _dec_start != _acc_end + 1 && _pulse_pin_state == LOW) {
				unsigned int compare_value = _max_wf_freq / _f_trg;
				TimerUpdate(_timer, _channel, _irq, compare_value);
				//UpdateTimer(compare_value);
				_prev_counter_value = compare_value;				
			}
			// DECELERATION PHASE
			else {
				if (_pulse_pin_state == LOW) {
					int num = 2 * _prev_counter_value + _rest;
					int den = 4 * (_count - _total_steps) + 1;
					unsigned int compare_value = _prev_counter_value - (num / den);
					TimerUpdate(_timer, _channel, _irq, compare_value);
					_prev_counter_value = compare_value;
					if (_count != (_total_steps - 1)) {
						_rest = num % den;
					}
					else {
						_rest = 0;
					}
				}
			}
			Step();  // Toggle step pin
		}
		else {
			// STOP
			if (_pulse_pin_state == HIGH) { Step(); }
			_control_mode = STP;
			_moving = false;
			_finished = true;
			TimerStop(_timer, _channel, _irq);
		}
		break;
		// If Motor in VELOCITY MODE:
	case VEL:
		// IF STOP WITH DECELERATION STARTED: 
		if (_vm_stop) {
			if (_count < _total_steps) {

				if (_pulse_pin_state == LOW) {
					int cnt = _total_steps - _count;
					int num = 2 * _prev_counter_value + _rest;
					int den = 4 * (_count - _total_steps) + 1;
					unsigned int compare_value = _prev_counter_value - (num / den);
					TimerUpdate(_timer, _channel, _irq, compare_value);
					if (_count != (_total_steps - 1)) {
						_rest = num % den;
					}
					else {
						_rest = 0;
					}
					_prev_counter_value = compare_value;
				}
				Step();  // Toggle step pin
			}
			else {
				if (_pulse_pin_state == HIGH) {
					Step();
				}
				_moving = false;
				_finished = true;
				_control_mode = STP;
				TimerStop(_timer, _channel, _irq);
			}
		}
		// ELSE:
		else {
			// FIRST STEP:
			if (_count == 0 && _pulse_pin_state == LOW) {
				unsigned int compare_value = (int)_c0;
				TimerUpdate(_timer, _channel, _irq, compare_value);
				_prev_counter_value = compare_value;
				_rest = 0;
			}
			//ACCELERATION PHASE:
			else if (_count < _acc_end && _pulse_pin_state == LOW) {
				unsigned int compare_value = _prev_counter_value - ((2 * _prev_counter_value + _rest) / (4 * _count + 1));
				TimerUpdate(_timer, _channel, _irq, compare_value);
				_prev_counter_value = compare_value;
				if (_count != (_acc_end - 1)) {
					_rest = (2 * _prev_counter_value + _rest) % (4 * _count + 1);
				}
				else {
					_rest = 0;
				}
			}
			//ACCELERATION PHASE END :
			else if (_count == _acc_end && _pulse_pin_state == LOW) {
				unsigned int compare_value = _max_wf_freq / _f_trg;
				TimerUpdate(_timer, _channel, _irq, compare_value);
				_prev_counter_value = compare_value;
				_count++;
				_rest = 0;
			}
			Step();  // Toggle step pin
		}

		break;
	}
}
#pragma endregion

// ISR Handler Functions
#pragma region ISR Handler Functions
void TC0_Handler()
{
	TC_GetStatus(TC0, 0);
	timerSteppers[0].TimerMotor_Run();
}
void TC1_Handler()
{
	TC_GetStatus(TC0, 1);
	timerSteppers[1].TimerMotor_Run();
}
void TC2_Handler()
{
	TC_GetStatus(TC0, 2);
	timerSteppers[2].TimerMotor_Run();
}
void TC3_Handler()
{
	TC_GetStatus(TC1, 0);
	timerSteppers[3].TimerMotor_Run();
}
void TC4_Handler()
{
	TC_GetStatus(TC1, 1);
	timerSteppers[4].TimerMotor_Run();
}
void TC5_Handler()
{
	TC_GetStatus(TC1, 2);
	timerSteppers[5].TimerMotor_Run();
}
void TC6_Handler()
{
	TC_GetStatus(TC2, 0);
	timerSteppers[6].TimerMotor_Run();
}
void TC7_Handler()
{
	TC_GetStatus(TC2, 1);
	timerSteppers[7].TimerMotor_Run();
}
void TC8_Handler()
{
	TC_GetStatus(TC2, 2);
	timerSteppers[8].TimerMotor_Run();
}
#pragma endregion

