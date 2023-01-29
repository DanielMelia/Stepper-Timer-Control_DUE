#ifndef axis_h
#define axis_h

#include "Arduino.h"
#include "TimerStepperDue.h"

class axis
{
public:
  void init(int timer_id = 0);
  void SetTimerId(int id);
  void ConfigureMotor(int pulse_pin, int dir_pin, int ena_pin, int ppr, bool linear_stage = false, float pitch = 0);
  void ConfigureAxisLimits(int home_ls_pin, int far_ls_pin, float min_pos_value, float max_pos_value);

  void SetMotionParameters(float vel, float acc, float dec);

  void Home(int freq, bool dir);
  void Home_ovr(float pos);
  void MoveRelativeDistance(float dist);
  void MoveToPosition(float trg_pos);
  void Stop(int steps = 0);
  void StopWithDeceleration(float dec = 0);
  void ResetPosition();
  void MoveWithVelocity(bool dir, float vel = 0);
  void MoveJoystickMode(bool dir, float vel);

  bool IsHomed();
  bool IsMoving();
  float GetPosition();
  byte GetCurrentControlMode();
  byte CheckLimitSwitches();
  float minPos() const {return _minPos;}
  float maxPos() const {return _maxPos;}
  String PassMotorParametersToString();
    float GetSpeed() const {return _vel;}
  float GetAcc() const {return _acc;}
  float GetDec() const {return _dec;}

private:
  int _timer_id;
  float _vel, _acc, _dec; // Default parameters
  float _curr_vel_mms;        // Current Speed Setting
  float _minPos, _maxPos;
  float _curr_pos;
  bool _isHomed, _isMoving;
};



#endif
