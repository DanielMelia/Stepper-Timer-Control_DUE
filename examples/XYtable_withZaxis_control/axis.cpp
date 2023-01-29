#include "Arduino.h"
#include "axis.h"

// Initialization / Setup Functions

void axis::init(int timer_id){
  int st_count = sizeof(timerSteppers) / sizeof(timerSteppers[0]);  // sizeof operator returns the size of the object in bytes
  //Serial.println(st_count);
  for (int i = 0; i < st_count; i++) {
    if (i == 0) {
      timerSteppers[i].ConfigureTimer(TC0, 0, TC0_IRQn);
    } else if (i == 1) {
      timerSteppers[i].ConfigureTimer(TC0, 1, TC1_IRQn);
    } else if (i == 2) {
      timerSteppers[i].ConfigureTimer(TC0, 2, TC2_IRQn);
    } else if (i == 3) {
      timerSteppers[i].ConfigureTimer(TC1, 0, TC3_IRQn);
    } else if (i == 4) {
      timerSteppers[i].ConfigureTimer(TC1, 1, TC4_IRQn);
    } else if (i == 5) {
      timerSteppers[i].ConfigureTimer(TC1, 2, TC5_IRQn);
    } else if (i == 6) {
      timerSteppers[i].ConfigureTimer(TC2, 0, TC6_IRQn);
    } else if (i == 7) {
      timerSteppers[i].ConfigureTimer(TC2, 1, TC7_IRQn);
    } else if (i == 8) {
      timerSteppers[i].ConfigureTimer(TC2, 2, TC8_IRQn);
    }
  } 
  SetTimerId(timer_id); 
}

void axis::SetTimerId(int id){
  _timer_id = id;
}
void axis::ConfigureMotor(int pulse_pin, int dir_pin, int ena_pin, int ppr, bool linear_stage, float pitch){
  timerSteppers[_timer_id].MotorSetup(pulse_pin, dir_pin, ena_pin, ppr, linear_stage, pitch);
}

void axis::ConfigureAxisLimits(int home_ls_pin, int far_ls_pin, float min_pos_value, float max_pos_value){
  _minPos = min_pos_value;
  _maxPos = max_pos_value;
  timerSteppers[_timer_id].LimitSwtichesSetup(home_ls_pin, far_ls_pin, min_pos_value, max_pos_value);
}

void axis::SetMotionParameters(float vel, float acc, float dec){
  _vel=vel;
  _acc=acc;
  _dec=dec;
}

// Feedback Functions

bool axis::IsHomed(){
  _isHomed = timerSteppers[_timer_id].IsHomed();
  return _isHomed;
}

bool axis::IsMoving(){
  _isMoving = timerSteppers[_timer_id].IsMoving();
  return _isMoving;
}

float axis::GetPosition(){
  //Serial.println("GP");
  _curr_pos = timerSteppers[_timer_id].GetPosition_mm_rad();
  //Serial.println(_curr_pos);
  return _curr_pos;
}

byte axis::GetCurrentControlMode(){
  return timerSteppers[_timer_id].GetCurrentControlMode();
}

byte axis::CheckLimitSwitches(){
  return timerSteppers[_timer_id].CheckLimitSwitches();
}

String axis::PassMotorParametersToString(){
  return timerSteppers[_timer_id].PassMotorParametersToString();
}

// Motion Control Functions

void axis::Stop(int steps){
  if (steps == 0){timerSteppers[_timer_id].StopMotor();}
  else{timerSteppers[_timer_id].StopSpeedMode(steps);}
  
}

void axis::ResetPosition(){
  timerSteppers[_timer_id].ResetPosition();
}

void axis::Home(int freq, bool dir){
  timerSteppers[_timer_id].Home(freq, dir);
}

void axis::Home_ovr(float pos){
  timerSteppers[_timer_id].Home_ovr(pos);
}

void axis::MoveRelativeDistance(float dist){
  if(dist == 0){return;}
  _curr_vel_mms = _vel;
  timerSteppers[_timer_id].RunLinearStage(dist, _vel, _acc, _dec);
}

void axis::MoveToPosition(float trg_pos){
  _curr_pos = GetPosition();
  //Serial.println(_curr_pos);
  if (_curr_pos ==-999){return;}
  float dist = trg_pos - _curr_pos;
  //Serial.println(dist);
  MoveRelativeDistance(dist);
}

void axis::MoveWithVelocity(bool dir, float vel){
  if (vel == 0){
    _curr_vel_mms = _vel;
    timerSteppers[_timer_id].RunLinearStageWithVelocity(dir, _vel, _acc);
  }else{
    _curr_vel_mms = vel;
    timerSteppers[_timer_id].RunLinearStageWithVelocity(dir, vel, _acc);
  }
  
}

void axis::MoveJoystickMode(bool dir, float vel){
  //Serial.print("Vel: ");Serial.println(vel);
  _curr_vel_mms = vel;
  timerSteppers[_timer_id].RunLinearStageWithVelocity(dir, vel, 0);
}

void axis::StopWithDeceleration(float dec){
  if (dec == 0){dec = _dec;}
  if (_curr_vel_mms == 0){Stop();}
  else{timerSteppers[_timer_id].StopWithDeceleration(dec, _curr_vel_mms);}
  
}

// void axis::MoveWithVelocity(bool dir, float vel){
//   if(vel == 0 && dir){
//     MoveToPosition(_maxPos);
//   }else if(vel == 0 && !dir){
//     MoveToPosition(_minPos);
//   }else{
//     float dist;
//     if(dir){

//     }else{

//     }
//     timerSteppers[_timer_id].RunLinearStage(dist, _vel, _acc, _dec);
//   }
// }

