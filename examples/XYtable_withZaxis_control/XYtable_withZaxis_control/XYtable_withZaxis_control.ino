#include "XY_Table.h"
//#include "axis.h"

// INITITAL COMMANDS:
// CONNECT: I:1;2;
// RESET POSITION: U;
//SET X AXIS: X:52;53;47;22;23;1000;2.0;200.0;
//SET Y AXIS: Y:50;51;46;24;25;1000;2.0;200.0;
//SET Z AXIS: Z:48;49;45;26;27;2000;0.4;50.0;
//SET MOTION: P:10.0;50.0;50.0;


XY_Table PIC_sorter_XY_table;
axis Z_stage;

enum AxisConfiguration {XY,Z,XYZ};
//NONE, X, Y, Z, XY, XZ, YZ, XYZ; {0, 1, 2, 3, 4, 5, 6, 7}
byte system_homed;

int axisConfig;

// --------------------------------------- SERIAL PARAMETERS -------------------------------------------------------- //

String InStr;          // Variable String to receive incoming Serial commands
char first_char;       // First Character of the incoming String. Contains command code (which command to execute)
//int myArray[i+1] = {0, ... , i};
// Array example --> int myArray[3] = {a0, a1, a2} ; myArray[2] contains a2
float sub_command[8];  // Array to store the parameters passed with the command (angle, velocity, acceleration, etc...). Need to specify maximum number of expected parameters
int sub_command_size = sizeof(sub_command) / sizeof(sub_command[0]);  // sizeof operator returns the size of the object in bytes
int params_received = 0;
bool SerialOut = false;
int sample_time = 100; // Main loop cycle time
bool hasZ_Axis; //Set to TRUE to enable Z axis functions

int pow_pin = 30;
int pow_ena_pin = 31;

void setup() {
  //Initialise Serial Port:
  Serial.begin(115200);
  Serial.println("Serial init");
  PIC_sorter_XY_table.init();
  Z_stage.SetTimerId(2);
  pinMode(pow_pin, INPUT);
  pinMode(pow_ena_pin, OUTPUT);
  

  //InitialiseTimers();   // Execute at the beggining of the program. Assigns every timerStepper object to its correspondant Timer/Counter on the Microcontroller

  //PIC_sorter_XY_table.X.SetTimerId(0);
  //PIC_sorter_XY_table.Y.SetTimerId(1);

  //PIC_sorter_XY_table.X.ConfigureMotor(52, 53, 47, 1000, true, 2);
  //PIC_sorter_XY_table.Y.ConfigureMotor(50, 51, 46, 1000, true, 2);
  //Z_stage.ConfigureMotor(48, 49, 45, 2000, true, 2);  // NOTE: Z-motor is 400steps/rev. Solution: set the pitch to half of what it should be OR ppr to double

  //PIC_sorter_XY_table.X.ConfigureAxisLimits(22, 23, 0, 200);
  //PIC_sorter_XY_table.Y.ConfigureAxisLimits(24, 25, 0, 200);  
  //Z_stage.ConfigureAxisLimits(26, 27, 0, 50);

  //PIC_sorter_XY_table.SetMotionParameters(5,25,25);
  //Z_stage.SetMotionParameters(5,25,25);
  digitalWrite(pow_ena_pin, false);
}

void loop() {
  uint32_t start = millis(); // read start time
  // 1 - READ INCOMING SERIAL COMMAND
  readCommand(InStr);
  int IsPowOn = digitalRead(pow_pin);
  int IsPowEnabled = digitalRead(pow_ena_pin);
  byte stageHomed = PIC_sorter_XY_table.IsHomed();
  byte stageMoving = PIC_sorter_XY_table.IsMoving();
  bool Z_homed = Z_stage.IsHomed();
  bool Z_moving = Z_stage.IsMoving();
  
  byte stageCM = PIC_sorter_XY_table.GetCurrentControlMode();
  byte Z_CM = Z_stage.GetCurrentControlMode();

  byte XYZ_CM = GetXYZ_ControlMode(stageCM, Z_CM); 

  position stagePosition = PIC_sorter_XY_table.GetPosition();
  float Z_position = Z_stage.GetPosition();
  byte x_lim_sw = PIC_sorter_XY_table.X.CheckLimitSwitches();
  byte y_lim_sw = PIC_sorter_XY_table.Y.CheckLimitSwitches();
  byte z_lim_sw = Z_stage.CheckLimitSwitches();
  system_homed = GetSystemHomed(stageHomed, Z_homed);
  //float speed = PIC_sorter_XY_table.GetSpeed();
  //float acc = PIC_sorter_XY_table.GetAcc();
  //float dec = PIC_sorter_XY_table.GetDec();


  if(SerialOut){
    
    Serial.print(stageHomed);Serial.print(";");           //data: 0
    Serial.print(Z_homed);Serial.print(";");              //data: 1
    Serial.print(stagePosition.x, 3);Serial.print(";");   //data: 2
    Serial.print(stagePosition.y, 3);Serial.print(";");   //data: 3
    Serial.print(Z_position, 3);Serial.print(";");        //data: 4
    Serial.print(stageMoving);Serial.print(";");          //data: 5
    Serial.print(Z_moving);Serial.print(";");             //data: 6
    Serial.print(x_lim_sw);Serial.print(";");             //data: 7
    Serial.print(y_lim_sw);Serial.print(";");             //data: 8
    Serial.print(z_lim_sw);Serial.print(";");             //data: 9
    Serial.print(IsPowOn);Serial.print(";");              //data: 10   
    Serial.print(IsPowEnabled);Serial.print(";");              //data: 10  
    Serial.print(XYZ_CM);Serial.print(";");              //data: 11
    Serial.print(first_char);Serial.println(";");         //data: 12

    // Serial.print(stageHomed);Serial.print("\t");           //data: 0
    // Serial.print(Z_homed);Serial.print("\t");              //data: 1
    // Serial.print(stagePosition.x, 3);Serial.print("\t");   //data: 2
    // Serial.print(stagePosition.y, 3);Serial.print("\t");   //data: 3
    // Serial.print(Z_position, 3);Serial.print("\t");        //data: 4
    // Serial.print(stageMoving);Serial.print("\t");          //data: 5
    // Serial.print(Z_moving);Serial.print("\t");             //data: 6
    // Serial.print(x_lim_sw);Serial.print("\t");             //data: 7
    // Serial.print(y_lim_sw);Serial.print("\t");             //data: 8
    // Serial.print(z_lim_sw);Serial.print("\t");             //data: 9
    // Serial.print(IsPowOn);Serial.print("\t");              //data: 10   
    // Serial.print(XYZ_CM);Serial.print("\t");               //data: 11
    // Serial.println(first_char);                            //data: 12

    
 
  }

  //Serial.print("Motor Position: ");Serial.print(stagePosition.x, 4);Serial.print("\t");Serial.println(stagePosition.y, 4);

// ---------------------------------------------------------------------------- //  

  uint32_t finish = millis();  // read finish time
  //Calculate cycle time duration
  int duration = (int) finish - (int) start;
  //Serial.println(duration);
  /*To obtain a fixed sample time, the cycle time duration is subtracted   
  from the sample time, and the difference is added to the code as a delay*/
  int t_delay = sample_time - duration;
  if (t_delay > 0) {delay(t_delay);}

  //delay(100);
}

byte GetXYZ_ControlMode(byte XY_CM, byte Z_CM){
  if(XY_CM == Z_CM){
    return XY_CM;
  }else{
    if(XY_CM == 0){return Z_CM;}     
    else if(Z_CM == 0){return XY_CM;}
    else{return 4;}   
  }
}

//NONE, X, Y, Z, XY, XZ, YZ, XYZ; {0, 1, 2, 3, 4, 5, 6, 7}
// 0    1  2  3   4   5   6    7
byte GetSystemHomed(byte XY_home_state, bool Z_home_state){
  if (XY_home_state ==0){ // XY NONE HOMED
    if(!Z_home_state){return 0;}
    else{return 3;}
  }else if (XY_home_state == 1){ // XY BOTH HOMED
    if(!Z_home_state){return 4;}
    else{return 7;}
  }else if (XY_home_state == 2){ // XY X HOMED
    if(!Z_home_state){return 1;}
    else{return 5;}    
  }else if (XY_home_state == 3){ // XY Y HOMED
    if(!Z_home_state){return 2;}
    else{return 6;}    
  }else{
    return 9;
  }
  
}

void readCommand(String &inString) {
  //If Serial available read the input Serial line
  if (Serial.available()) {
    inString = Serial.readStringUntil('\n');

    //If the string is not empty, translate and execute the command
    if (inString.length() > 0) {
      //SplitString extracts the command code and stores it into first_char (what command to execute), then extracts the command parameters
      //and stores them into the array sub_command (information about the command)
      splitString(inString, first_char, sub_command);
      // OPTION: Copy structure from P (Get/Set) to X, Y, and Z
      switch (first_char) {
      // AXES CONFIGURATION  
        // SENT HARDWARE PARAMETERS THOUGHT SERIAL PORT
        case 'C':  //C:axis;
        if(sub_command[0] == -999){return;}
        else if(sub_command[0] == 0){Serial.println("X;" + PIC_sorter_XY_table.X.PassMotorParametersToString());}
        else if(sub_command[0] == 1){Serial.println("Y;" + PIC_sorter_XY_table.Y.PassMotorParametersToString());}
        else if(sub_command[0] == 2){Serial.println("Z;" + Z_stage.PassMotorParametersToString());}
          //Serial.println("C;" + PIC_sorter_XY_table.X.PassMotorParametersToString() + PIC_sorter_XY_table.Y.PassMotorParametersToString() + Z_stage.PassMotorParametersToString());
          //Z_stage.PassMotorParametersToString()
          //Serial.println("Y;" );
          break;
        // SET X AXIS HARDWARE PARAMETERS
        case 'X':  //X:pls_pin;dir_pin;ena_pin;h_sw_pin;f_sw_pin;ppr;pitch;travel_mm;
          for (int i = 0; i <= 7; i++) {if(sub_command[i] == -999){return;}}
        //if(){sub_command[0]==-999}
          PIC_sorter_XY_table.X.ConfigureMotor((int)sub_command[0], (int)sub_command[1], (int)sub_command[2], (int)sub_command[5], true, sub_command[6]);
          PIC_sorter_XY_table.X.ConfigureAxisLimits((int)sub_command[3], (int)sub_command[4], 0, sub_command[7]);
          break;
        // SET Y AXIS HARDWARE PARAMETERS             
        case 'Y':  //Y:pls_pin;dir_pin;ena_pin;h_sw_pin;f_sw_pin;ppr;pitch;travel_mm;
          for (int i = 0; i <= 7; i++) {if(sub_command[i] == -999){return;}}
          PIC_sorter_XY_table.Y.ConfigureMotor((int)sub_command[0], (int)sub_command[1], (int)sub_command[2], (int)sub_command[5], true, sub_command[6]);
          PIC_sorter_XY_table.Y.ConfigureAxisLimits((int)sub_command[3], (int)sub_command[4], 0, sub_command[7]);
          break;
        // SET Z AXIS HARDWARE PARAMETERS   
        case 'Z':  //Z:pls_pin;dir_pin;ena_pin;h_sw_pin;f_sw_pin;ppr;pitch;travel_mm;
          for (int i = 0; i <= 7; i++) {if(sub_command[i] == -999){return;}}
          Z_stage.ConfigureMotor((int)sub_command[0], (int)sub_command[1], (int)sub_command[2], (int)sub_command[5], true, sub_command[6]);
          Z_stage.ConfigureAxisLimits((int)sub_command[3], (int)sub_command[4], 0, sub_command[7]);
          break;         
        // SET / SEND MOTION PARAMETERS
        case 'P':  // Motion Parameters: GET: P; / SET: P:vel;acc;dec;
          if (sub_command[0] == -999){
            Serial.println("P;" + String(PIC_sorter_XY_table.GetSpeed()) + ";" + String(PIC_sorter_XY_table.GetAcc()) + ";" + String(PIC_sorter_XY_table.GetDec()) + ";" + String(Z_stage.GetSpeed()) + ";" + String(Z_stage.GetAcc()) + ";" + String(Z_stage.GetDec()) + ";");
          }else{
            for (int i = 0; i <= 2; i++) {if(sub_command[i] == -999){return;}}
            PIC_sorter_XY_table.SetMotionParameters(sub_command[0], sub_command[1], sub_command[2]);
            if (axisConfig == 1 || axisConfig == 2){Z_stage.SetMotionParameters(sub_command[3], sub_command[4], sub_command[5]);}
          }
          break;

      // INITIALISATION
        // ENABLE/DISABLE POWER
        case 'E':
          if(sub_command[0] == -999){return;}
          else if((int)sub_command[0] == 0){digitalWrite(pow_ena_pin, true);}
          else if((int)sub_command[0] == 1){digitalWrite(pow_ena_pin, false);}
          break;

        // HOMING
        case 'H':  //H:freq;dir;
          //if(sub_command[0]==-999 || sub_command[1]==-999){return;}
          for (int i = 0; i <= (sub_command_size - 1); i++) {if(sub_command[i] != -999){params_received += 1;}}

          //for (int i = 0; i <= 5; i++) {if(sub_command[i] == -999){return;}}
          if(params_received == 5){
          PIC_sorter_XY_table.X.Home((int)sub_command[0],(bool)sub_command[2]);
          PIC_sorter_XY_table.Y.Home((int)sub_command[1],(bool)sub_command[3]);
          Z_stage.Home_ovr(sub_command[4]);
          }else if(params_received == 6){
          PIC_sorter_XY_table.X.Home((int)sub_command[0],(bool)sub_command[3]);
          PIC_sorter_XY_table.Y.Home((int)sub_command[1],(bool)sub_command[4]);
          Z_stage.Home((int)sub_command[2],(bool)sub_command[5]);
          }else if(params_received == 3){
            if ((int)sub_command[0]==1){
              PIC_sorter_XY_table.X.Home((int)sub_command[1],(bool)sub_command[2]);
              PIC_sorter_XY_table.Y.Home_ovr(0);
              Z_stage.Home_ovr(0);
              }
            else if ((int)sub_command[0]==2){
              PIC_sorter_XY_table.Y.Home((int)sub_command[1],(bool)sub_command[2]);
              PIC_sorter_XY_table.X.Home_ovr(0);
              Z_stage.Home_ovr(0);              
              }
            else if ((int)sub_command[0]==3){
              PIC_sorter_XY_table.X.Home_ovr(0);
              PIC_sorter_XY_table.Y.Home_ovr(0);
              Z_stage.Home((int)sub_command[1],(bool)sub_command[2]);
              }
          }

          break;
        
        case 'K':
          for (int i = 0; i <= (sub_command_size - 1); i++) {if(sub_command[i] != -999){params_received += 1;}}
          if(params_received == 3){
            PIC_sorter_XY_table.X.Home_ovr(sub_command[0]);
            PIC_sorter_XY_table.Y.Home_ovr(sub_command[1]);
            Z_stage.Home_ovr(sub_command[2]);
          }
          break;
          
        // BEGIN SERIAL TRANSMISSION  
        case 'I':  //I:on_off;
          if(sub_command[0]!=0 && sub_command[0]!=1){return;}
          if((bool)sub_command[0]){  
            if(sub_command[1]!=0 && sub_command[1]!=1 && sub_command[1]!=2){return;}  
            SerialOut = true;      
            axisConfig = (int)sub_command[1];
            //String str = String(axisConfig);
            //axisConfig = XYZ;
            Serial.println("I;" + String(axisConfig) + ";");
            //Serial.println("I;2;");
            //Serial.println(axisConfig);
            delay(100);
            
          }else{
            SerialOut = false;
          }          
          break;
        // RESET AXES POSITIONS (UN-HOME)  
        case 'U':  //S;
          PIC_sorter_XY_table.ResetAxesPosition();
          Z_stage.ResetPosition();
          break; 

      // MOTION COMMANDS
        // NOTE: Include additional parameter to Move Functions: 0 -> XY Only; 1 -> Z Only; 2 -> XYZ
        // JOYSTICK MODE       
        case 'J':  //J:x_vel;y_vel;z_vel;
          for (int i = 0; i <= 2; i++) {if(sub_command[i] == -999){return;}}
          PIC_sorter_XY_table.MoveJoystickMode(sub_command[0], sub_command[1]);
          if (axisConfig == 1 || axisConfig == 2){
            if (sub_command[2] >= 0){
              Z_stage.MoveJoystickMode(true,sub_command[2]);
            }else{
              Z_stage.MoveJoystickMode(false,abs(sub_command[2]));
            }            
          }
          //PIC_sorter_XY_table.X.MoveToPosition((int)sub_command[0]);
          break;  
        // MOVE TO ABSOLUTE POSITION                           
        case 'M':  //M:x_pos;y_pos;z_pos;
          for (int i = 0; i <= 2; i++) {if(sub_command[i] == -999){return;}}
          PIC_sorter_XY_table.MoveToPosition_mm(sub_command[0], sub_command[1], true);
          if (axisConfig == 1 || axisConfig == 2){Z_stage.MoveToPosition(sub_command[2]);}
          break;
        // MOVE RELATIVE DISTANCE                   
        case 'R':  //R:x_dist;y_dist;z_dist;
          for (int i = 0; i <= 2; i++) {if(sub_command[i] == -999){return;}}
          PIC_sorter_XY_table.MoveDistance_mm(sub_command[0], sub_command[1]);
          if (axisConfig == 1 || axisConfig == 2){
            Z_stage.MoveRelativeDistance(sub_command[2]);
            }
          break;
        // MOVE IN VELOCITY MODE
        case 'V':  //V:axis;dir;vel;
          for (int i = 0; i <= 2; i++) {if(sub_command[i] == -999){return;}}
          if((int)sub_command[0] == 0){
            PIC_sorter_XY_table.X.MoveWithVelocity((bool)sub_command[1],sub_command[2]);
          }else if ((int)sub_command[0] == 1){
            PIC_sorter_XY_table.Y.MoveWithVelocity((bool)sub_command[1],sub_command[2]);
          }else if ((int)sub_command[0] == 2){
            if (axisConfig == 1 || axisConfig == 2){Z_stage.MoveWithVelocity((bool)sub_command[1],sub_command[2]);}
          }         
          break; 
        // STOP ALL MOTION      
        case 'S':  //S:mode;
          if((bool)sub_command[0] == 0){
            PIC_sorter_XY_table.StopAll((int)sub_command[1]);
            Z_stage.Stop((int)sub_command[1]);
            //PIC_sorter_XY_table.StopAllWithDeceleration((int)sub_command[1]);
            //Z_stage.StopWithDeceleration((int)sub_command[1]);
          }
          else{
            PIC_sorter_XY_table.StopAllWithDeceleration();
            Z_stage.StopWithDeceleration();
          }

          //PIC_sorter_XY_table.X.Stop();
          //PIC_sorter_XY_table.Y.Stop();
          break;           
      }
    }

    // Reset variables
    inString = "";
    //sub_command_size
    int array_size = sizeof(sub_command) / sizeof(sub_command[0]);  // sizeof operator returns the size of the object in bytes
    for (int i = 0; i <= (sub_command_size - 1); i++) {
    //for (int i = 0; i <= (array_size - 1); i++) {
      sub_command[i] = -999;  // Initialise array with -999. Check inside each case, that the split values are not equal to -999.
    } 
    params_received = 0;

  }
}

void splitString(String &inString, char &fst_char, float c[]) {
  //Function to split the entering command into its respective values.
  //The command should have a first character indicating the action code (home, move, ...) followed by a serie of parameters (position, velocity, time,...)
  // The action code is followed by ":", and every parameter is separated by ";"
  // The function takes the inStr command and updates by reference the firstChar parameter and the sub_command array of values

  fst_char = inString.charAt(0);
  int ind1, ind2, ind3, ind4, ind5, ind6, ind7, ind8, ind9;
  ind1 = inString.indexOf(':');  //Find the index of the character ':'
  if (ind1 == -1) { //if not found
    return;
  }
  ind2 = inString.indexOf(';', ind1 + 1);  //Find the first ';' after the colon
  if (ind2 == -1) {
    return;
  }
  c[0] = inString.substring(ind1 + 1, ind2).toFloat();  //Extract substring from ':' to the first ';' and convert it to float
  ind3 = inString.indexOf(';', ind2 + 1);  //Find the next ';'
  if (ind3 == -1) {
    return;
  }
  c[1] = inString.substring(ind2 + 1, ind3).toFloat();
  ind4 = inString.indexOf(';', ind3 + 1);
  if (ind4 == -1) {
    return;
  }
  c[2] = inString.substring(ind3 + 1, ind4).toFloat();
  ind5 = inString.indexOf(';', ind4 + 1);
  if (ind5 == -1) {
    return;
  }
  c[3] = inString.substring(ind4 + 1, ind5).toFloat();
  ind6 = inString.indexOf(';', ind5 + 1);
  if (ind6 == -1) {
    return;
  }
  c[4] = inString.substring(ind5 + 1, ind6).toFloat();
  ind7 = inString.indexOf(';', ind6 + 1);
  if (ind7 == -1) {
    return;
  }
  c[5] = inString.substring(ind6 + 1, ind7).toFloat();  
  ind8 = inString.indexOf(';', ind7 + 1);
  if (ind8 == -1) {
    return;
  }
  c[6] = inString.substring(ind7 + 1, ind8).toFloat(); 
  ind9 = inString.indexOf(';', ind8 + 1);
  if (ind9 == -1) {
    return;
  }
  c[7] = inString.substring(ind8 + 1, ind9).toFloat(); 

}

void InitialiseTimers() {
  int st_count = sizeof(timerSteppers) / sizeof(timerSteppers[0]);  // sizeof operator returns the size of the object in bytes
  Serial.println(st_count);
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
}
