#ifndef XY_Table_h
#define XY_Table_h

#include "Arduino.h"
//#include "TimerStepperDue.h"
#include "axis.h"

//enum table_type{XY, XYZ, XYT, XYZT};

struct position{
  float x;
  float y;
};

class XY_Table
{
public:
  //XY_Table(int mock);
  void init(int x_timer_id = 0, int y_timer_id = 1);
  axis X;
  axis Y;

  //Stage Move Functions
  void Home(int freq = 1000, int x_dir = 0, int y_dir = 0);
  void MoveDistance_mm(float x, float y, bool synchronize = true);
  void MoveToPosition_mm(float x, float y, bool synchronize = true);
  void MoveJoystickMode(float v_x, float v_y);
  void SetMotionParameters(float vel, float acc, float dec);
  void ResetAxesPosition();
  void SetSpeeds(float x_vel, float y_vel);
  void StopAll(int steps = 0);
  void StopAllWithDeceleration(float dec = 0);

  //Feedback Functions
  byte IsHomed();       // 0: not homed; 1: both axes homed; 2: x-axis homed; 3: y-axis homed
  byte IsMoving();      // 0: not moving; 1: both axes moving; 2: x-axis moving; 3: y-axis moving
  position GetPosition();
  byte GetCurrentControlMode();
  float GetSpeed() const {return _vel;}
  float GetAcc() const {return _acc;}
  float GetDec() const {return _dec;}


  //void Configure_X_axis(int pulse_pin, int dir_pin, int home_ls_pin, int far_ls_pin, int ppr, float pitch, float min_pos_value, float max_pos_value);
  //void Configure_Y_axis(int pulse_pin, int dir_pin, int home_ls_pin, int far_ls_pin, int ppr, float pitch, float min_pos_value, float max_pos_value);
private:
  bool _IsHomed, _IsXHomed, _IsYHomed;
  bool _IsMoving, _IsXMoving, _IsYMoving;
  float _vel, _acc, _dec;
  position _pos;
  //float _x_vel, _y_vel;
};

#endif