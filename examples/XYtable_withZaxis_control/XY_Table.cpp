#include "Arduino.h"
#include "XY_Table.h"

// Initialization / Setup Functions

void XY_Table::init(int x_timer_id, int y_timer_id){
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

  X.SetTimerId(x_timer_id);
  Y.SetTimerId(y_timer_id);
  //Serial.println("Timers Set!");
}

void XY_Table::SetMotionParameters(float vel, float acc, float dec){
  _vel=vel;
  _acc=acc;
  _dec=dec;
  X.SetMotionParameters(_vel, _acc, _dec);
  Y.SetMotionParameters(_vel, _acc, _dec);
}

void XY_Table::ResetAxesPosition(){
  X.ResetPosition();
  Y.ResetPosition();
}

void XY_Table::StopAll(int steps){

  X.Stop(steps);
  Y.Stop(steps);

  // if (steps == 0){
  //   X.Stop();
  //   Y.Stop();
  // }
  // else{
  //   X.Stop(steps);
  //   Y.Stop(steps);
  // }


}

void XY_Table::StopAllWithDeceleration(float dec){
  X.StopWithDeceleration(dec);
  Y.StopWithDeceleration(dec);
}

// Feedback Functions

position XY_Table::GetPosition(){
  position tempPos;
  tempPos.x = X.GetPosition();
  tempPos.y = Y.GetPosition();
  _pos = tempPos;
  return _pos;
}

byte XY_Table::IsHomed() {
  _IsXHomed = X.IsHomed();
  _IsYHomed = Y.IsHomed();

  if (_IsXHomed && _IsYHomed) {
    _IsHomed = true;
  } else {
    _IsHomed = false;
  }

  if(_IsHomed){
    return 1;
  }else{
    if(_IsXHomed){
      return 2;
    }else if(_IsYHomed){
      return 3;
    }else{
      return 0;
    }
  }

}

byte XY_Table::IsMoving() {
  _IsXMoving = X.IsMoving();
  _IsYMoving = Y.IsMoving();

  if (_IsXMoving || _IsYMoving) {
    _IsMoving = true;    
  } else {
    _IsMoving = false;
  }

  if (!_IsMoving){
    return 0;
  }else if(_IsXMoving && _IsYMoving){
    return 1;
  }else if(_IsXMoving){
    return 2;
  }else{
    return 3;
  }



}

byte XY_Table::GetCurrentControlMode(){
  // Control Modes: STP 0 / HOM 1 / POS 2 / VEL 3 / UNDEF 4
  // Single motor has only 0-3. UNDEF added when both motors have different CM
  byte X_CM = X.GetCurrentControlMode();
  byte Y_CM = Y.GetCurrentControlMode();

  if(X_CM == Y_CM){
    return X_CM;
  }else{
    if(X_CM == 0){return Y_CM;}     
    else if(Y_CM == 0){return X_CM;}
    else{return 4;}   
  }

}

// Motion Control Functions

void XY_Table::Home(int freq, int x_dir, int y_dir){
  X.Home(freq, x_dir);
  //Y.Home(freq, y_dir);
}

void XY_Table::MoveDistance_mm(float x, float y, bool synchronize){

    //Serial.print("x dist: ");Serial.println(x);
    //Serial.print("y dist: ");Serial.println(y);
  //Serial.print("Sync: ");Serial.println(synchronize);
  if (synchronize) {

    float x_vel, y_vel;

    if (abs(x) >= abs(y)) {
      x_vel = _vel;
      y_vel = _vel * abs(y / x);
    } else {
      x_vel = _vel * abs(x / y);
      y_vel = _vel;
    }

    //Serial.print("x vel: ");Serial.println(x_vel);
    //Serial.print("y vel: ");Serial.println(y_vel);
    //Serial.println("Param1: ");Serial.println(x_vel);Serial.println(y_vel);Serial.println(_acc);Serial.println(_dec);
    X.SetMotionParameters(x_vel, _acc, _dec);
    Y.SetMotionParameters(y_vel, _acc, _dec);
  } else {
    //Serial.println("Param2: ");Serial.println(_vel);Serial.println(_acc);Serial.println(_dec);
    X.SetMotionParameters(_vel, _acc, _dec);
    Y.SetMotionParameters(_vel, _acc, _dec);
  }
  //Serial.print("R_X: ");
  X.MoveRelativeDistance(x);
  //Serial.print("R_Y: ");
  Y.MoveRelativeDistance(y);
}

void XY_Table::MoveToPosition_mm(float x, float y, bool synchronize) {
  if (synchronize) {
    GetPosition();
    float x_dist = abs(x - _pos.x);
    float y_dist = abs(y - _pos.y);

    float x_vel, y_vel;

    if (x_dist >= y_dist) {
      x_vel = _vel;
      y_vel = _vel * (y_dist / x_dist);
    } else {
      x_vel = _vel * (x_dist / y_dist);
      y_vel = _vel;
    }

    X.SetMotionParameters(x_vel, _acc, _dec);
    Y.SetMotionParameters(y_vel, _acc, _dec);
  } else {
    X.SetMotionParameters(_vel, _acc, _dec);
    Y.SetMotionParameters(_vel, _acc, _dec);
  }


  X.MoveToPosition(x);
  Y.MoveToPosition(y);
}

void XY_Table::MoveJoystickMode(float v_x, float v_y){
  float x_vel = abs(v_x);
  float y_vel = abs(v_y);
  bool x_dir, y_dir;

  if (v_x >0){
    x_dir = true;
  }else{
    x_dir = false;
  }
  if (v_y >0){
    y_dir = true;
  }else{
    y_dir = false;
  }
  if (x_vel != 0){
    //Serial.print("x_vel: ");Serial.println(x_vel);
    X.MoveJoystickMode(x_dir, x_vel);
  }else{
    //Serial.println("x_stp");
    X.Stop();
  }
  if (y_vel != 0){
    //Serial.print("y_vel: ");Serial.println(y_vel);
    Y.MoveJoystickMode(y_dir, y_vel);
  }else{
    //Serial.println("y_stp");
    Y.Stop();
  }
}

// XY_Table::XY_Table(int mock){

//   int st_count = sizeof(timerSteppers) / sizeof(timerSteppers[0]);  // sizeof operator returns the size of the object in bytes
//   Serial.println(st_count);
//   for (int i = 0; i < st_count; i++) {
//     if (i == 0) {
//       timerSteppers[i].ConfigureTimer(TC0, 0, TC0_IRQn);
//     } else if (i == 1) {
//       timerSteppers[i].ConfigureTimer(TC0, 1, TC1_IRQn);
//     } else if (i == 2) {
//       timerSteppers[i].ConfigureTimer(TC0, 2, TC2_IRQn);
//     } else if (i == 3) {
//       timerSteppers[i].ConfigureTimer(TC1, 0, TC3_IRQn);
//     } else if (i == 4) {
//       timerSteppers[i].ConfigureTimer(TC1, 1, TC4_IRQn);
//     } else if (i == 5) {
//       timerSteppers[i].ConfigureTimer(TC1, 2, TC5_IRQn);
//     } else if (i == 6) {
//       timerSteppers[i].ConfigureTimer(TC2, 0, TC6_IRQn);
//     } else if (i == 7) {
//       timerSteppers[i].ConfigureTimer(TC2, 1, TC7_IRQn);
//     } else if (i == 8) {
//       timerSteppers[i].ConfigureTimer(TC2, 2, TC8_IRQn);
//     }
//   }

//   X.SetTimerId(0);
//   Y.SetTimerId(1);
//   Serial.println("Timers Set!");

// }

// void XY_Table::Configure_X_axis(int pulse_pin, int dir_pin, int home_ls_pin, int far_ls_pin, int ppr, float pitch, float min_pos_value, float max_pos_value){
//   Timer1_Stepper.TimerSetup(1, 64);       // Timer number , prescaler
//   Timer1_Stepper.MotorSetup(pulse_pin, dir_pin, ppr, true, pitch);
//   Timer1_Stepper.LimitSwtichesSetup(home_ls_pin, far_ls_pin, min_pos_value, max_pos_value);
// }

// void XY_Table::Configure_Y_axis(int pulse_pin, int dir_pin, int home_ls_pin, int far_ls_pin, int ppr, float pitch, float min_pos_value, float max_pos_value){
//   Timer2_Stepper.TimerSetup(2, 64);       // Timer number , prescaler
//   Timer2_Stepper.MotorSetup(pulse_pin, dir_pin, ppr, true, pitch);
//   Timer2_Stepper.LimitSwtichesSetup(home_ls_pin, far_ls_pin, min_pos_value, max_pos_value);
// }