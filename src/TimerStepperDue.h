#ifndef TimerStepperDue_h
#define TimerStepperDue_h

#include "Arduino.h"

class TimerStepperDue
{
public:
	// Initialise Timers
	static void InitialiseTimers();
	// SETUP FUNCTIONS
	void ConfigureTimer(Tc* tc, uint32_t channel, IRQn_Type irq);
	void TimerSetup(int timerNumber, int ps);
	void MotorSetup(int pulse_pin, int dir_pin, int ena_pin, int ppr, bool linear_stage = false, float pitch = 0);
	void LimitSwtichesSetup(int home_ls_pin, int far_ls_pin, float min_pos_value, float max_pos_value);
	void SetMicrostepping(int ms);
	// RUN MOTOR FUNCTIONS
	void Home(int freq, bool dir);
	void Home_ovr(float position);
	void RotateSteps(int steps, bool dir, int freq, int acc_steps, int dec_steps);
	void RotateAngle(float angle, float vel, float acc, float dec);
	void RunLinearStage(float distance_mm, float vel_mms, float acc_mms2, float dec_mms2);
	void RunLinearStage_abs(float abs_trgPos_mm, float vel_mms, float acc_mms2, float dec_mms2);
	void RunLinearStageWithVelocity(bool direction, float vel_mms, float acc_mms2);
	void RunSpeed(int freq, int acc_steps, bool dir);
	void StopWithDeceleration(float dec_mms2, float init_vel_mms);
	void StopSpeedMode(int steps);
	void StopMotor();
	void ResetPosition();
	void DisableMotor(bool disable);
	// FEEDBACK FUNCTIONS
	bool IsMoving();
	bool IsHomed();
	long GetStepPosition();
	float GetPosition_mm_rad();
	byte CheckLimitSwitches();
	byte GetCurrentControlMode();
	String PassMotorParametersToString();
	String PassMotorParametersAndIdToString();
	// INTERRUP SERVICE ROUTINE
	void TimerMotor_Run();
	
private:
	// Timer Start / Update / Stop / Trigger Functions
	void TimerStart(Tc* tc, uint32_t channel, IRQn_Type irq, uint32_t rc);
	void TimerUpdate(Tc* tc, uint32_t channel, IRQn_Type irq, uint32_t rc);
	void TimerStop(Tc* tc, uint32_t channel, IRQn_Type irq);
	void TimerTrigger(uint32_t rc);
	// Other functions
	void set_direction(bool dir);
	float convert_AccSteps_2_AccRads2(int freq, int acc_steps);
	int convert_AccRads2_2_AccSteps(float acc, float vel);
	void Step();
	// Timer Setup Variables ----------------------------------
	Tc *_timer;
	uint32_t _channel;
	IRQn_Type _irq;
	int _prescaler;
	// --------------------------------------------------------

	// Hardware Settings --------------------------------------
	int _pulse_pin, _dir_pin, _ena_pin;		// Stepper Driver Digital Pins: pulse / direction / enabled
	int _ppr;		// Stepper Driver pulses per revolution setting
	double _step_size_rad, _step_size_mm, _pitch;	// Stepper motor step size (rads and/or mm) and screw pitch
	int _home_limit_sw_pin, _far_limit_sw_pin, _home_limit_sw_value, _far_limit_sw_value;	// Limit Switches pin numbers and pin output values
	bool _axis_isLinear;	// FALSE: rotational ; TRUE: linear stage
	int motor_id;

	// Motor State --------------------------------------------
	bool _motor_homed, _motor_homed_phase_1;
	int _H_homing_counter, _F_homing_counter; // TEST - created to ignore false Limit Switch Triggerings
	bool _H_F_sensor_triggered; // FALSE : home sansor triggered, TRUE : far sensor triggered

	volatile bool _moving, _finished;  // Indicates if the motor is moving, and if motion has finished
	volatile bool _pulse_pin_state;
	int _dir;
	// --------------------------------------------------------
	byte _control_mode;
	// Trajectory Parameters ----------------------------------
	volatile unsigned int _count;
	int _total_steps;
	int _acc_end;
	int _dec_start;
	unsigned int _c0;                                  // Initial counter value for the acceleration phase
	int _f_trg;                       // Target frequency of the pulse train (motor rotation speed)
	long _max_wf_freq; // = clock_freq / (2 * prescaler);  // the maximum achievable waveform frequency is given by switching twice at clock_freq/prescaler (Hz)
	bool _vm_stop;                    // variable to stop the motor running in velocity mode
	volatile unsigned int _rest = 0;
	volatile unsigned int _prev_counter_value;  // Keeps track of the previous counter value		
	// --------------------------------------------------------
	
	long _step_position, _min_step_position, _max_step_position, _max_step_number;
	float _position, _min_position, _max_position;
	


};

extern TimerStepperDue timerSteppers[9];

extern TimerStepperDue Timer1_Stepper;
extern TimerStepperDue Timer2_Stepper;
extern TimerStepperDue Timer3_Stepper;
extern TimerStepperDue Timer4_Stepper;
extern TimerStepperDue Timer5_Stepper;
extern TimerStepperDue Timer6_Stepper;
extern TimerStepperDue Timer7_Stepper;
extern TimerStepperDue Timer8_Stepper;
extern TimerStepperDue Timer9_Stepper;

#endif