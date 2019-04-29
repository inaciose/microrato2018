//
//  Robot 1 - Robot based in a State machine that deviate from obstacles.
//  Working??
/*  Function: It deviates from obstacles.
    Change log: 
    Features:
    motor_direction() (0; -SPEED_HIGH); (-SPEED_HIGH; 0)
    IR tower fixed position. Robot rotate to find IR tower
    Two speeds! (NORMAL and SPEEDY)       
*/
#include "SoundRat2018.h"

void setup() {
  //Inicialize VAR
  for (int i = 0; i < SPEED_BUF_SIZE; i++) {
    speed_bufL[i] = 0;
    speed_bufR[i] = 0;
  }
  //Inicialize VAR END

  // BUTTONS
  pinMode(BUTTON_START_PIN, INPUT_PULLUP); pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);
  // Interrupções
  attachInterrupt(digitalPinToInterrupt(BUTTON_STOP_PIN), int_stop_pressed, FALLING); // RISING - to trigger when the pin goes from low to high; FALLING - for when the pin goes from high to low;

  // SERIAL
  Serial.begin (230400); Serial.println ("Setup ");

  // SONAR SENSORS
  SonarSetup(trigPin_FL,  echoPin_FL);
  SonarSetup(trigPin_L,   echoPin_L);
  SonarSetup(trigPin_FR,  echoPin_FR);
  SonarSetup(trigPin_R,   echoPin_R);
  //new
  SonarSetup(trigPin_F,  echoPin_F);
  SonarSetup(trigPin_F,   echoPin_F);
  
  // MOTOR's - Channel A, B
  pinMode(motorA_1A, OUTPUT);
  pinMode(motorA_2A, OUTPUT);
  pinMode(motorB_1A, OUTPUT);
  pinMode(motorB_2A, OUTPUT);

  // LED
  pinMode(LED_PIN, OUTPUT);
  turn_off_led_red();

  // IR Tower
  pinMode(servo_pin, OUTPUT);
  pinMode(receiver_pin, INPUT);
  ir_servo.attach(servo_pin);
  ir_servo.write(servo_current_pos);
  delay(500); // testing
  servo_last_millis = millis();

  // IR Floor sensors
  pinMode(IR_FLOOR_PIN_S0,INPUT);
  pinMode(IR_FLOOR_PIN_S1,INPUT);
  pinMode(IR_FLOOR_PIN_S2,INPUT);

  Serial.println("Waiting for start (Push Start button)");
  //Wait for Start button. Debounce
  button_start=0;
  while (button_start<5){
      button_start = 0;
      for(int i=0; i<20; i++){
        if(!digitalRead(BUTTON_START_PIN))
          button_start += 1;
        delay(1);
      }
  }

  // TEST 
  ///mouse_state = STATE_ROTATE_IR_SCAN;
}// SETUP() END

long tran2enc_pulse(long x) {
  return(x * WHEEL_IMP_ROT / WHEEL_PERIMETER);
}

#define if_test_mode 0
void loop() {
  unsigned long currentMillis = millis();
  unsigned long timeout = TIMEOUT_RACE;

  // 1º | Decide mouse_state
  if (currentMillis - prevMillis_tick_mouse_state_machine >= TICK_TIME_STATE_MACHINE) {
    prevMillis_tick_mouse_state_machine = currentMillis;

    // 1º | Verify timeout of race
    //PRINT("   currentMillis", currentMillis); PRINT("   race_time", race_time); PRINT("   timeout", timeout); 
    if ((currentMillis - race_time) >= timeout && mouse_state != STATE_ABORT) {
      mouse_state = STATE_TIMEOUT_RACE;
      PRINTSLN("   STATE_TIMEOUT_RACE"); 
      PRINTSLN("   STATE_TIMEOUT_RACE"); 
      PRINTSLN("   STATE_TIMEOUT_RACE"); 
      PRINTSLN("   STATE_TIMEOUT_RACE"); 
    }

    
    // 2º | Floor sensor
    ///while (millis() - prevMillis_tick_mouse_state_machine < 500);
    if(journey_state == STATEJ_FIND_IR_TOWER){
      ///prevMillis_tick_mouse_state_machine = millis();
      i_tmp = IR_floor_scan();
      
      if (i_tmp){
        PRINTLN("Floor Sensor ", i_tmp); 
        mouse_state = STATE_TOWER_ARRIVE;
        journey_state = STATEJ_FIND_BEGINING; 
        ///timeout = 2000; //The new time to go back
      }
    }

    // 3º | STATE MACHINE and SONAR SCAN
    mouse_state_machine_run(SonarScan());
    ///updatePosition();
  }
  
  // 4º | Read IR sensor | Stoping and rotating until complete rotation or finding IR tower
  if (journey_state == STATEJ_FIND_IR_TOWER){
    if(currentMillis - prevMillis_tick_IR_poling >= TICK_TIME_IR_POLLING[sequence]){
    
      if(sequence++ >= TICK_TIME_IR_POLLING) sequence=0;   
      
      mouse_state = STATE_ROTATE_IR_SCAN; 
      motors_stop();
      delay(10);
      ///myEnc_L.write(0);   // Reset encoders
      ///myEnc_R.write(0);
      myEnc_L_oldpos = myEnc_L.read();
      myEnc_R_oldpos = myEnc_R.read();
      prevMillis_tick_IR_poling = currentMillis;    // Only start counte rfor next poling after a succeful detection
      IR_read = 0;
    } // WHILE END
  }
}// LOOP() END

void mouse_state_machine_run(uint8_t sensors) {
  // Print changed mouse_state
  printState();


/* DICONNECTED INVERTION
  // Inversion on turning decision, with hystheres
  if(cnt_motors_right > 5){  
    turning = turn_RIGHT;
    PRINTLN("  turn_LEFT <<<", cnt_motors_right);
  }
  if(cnt_motors_right < -5){  
    turning = turn_LEFT;
    PRINTLN("  turn_RIGHT >>>", cnt_motors_right);
  }
  */
  
  switch (mouse_state) {
      case (STATE_ROTATE_IR_SCAN):
      //IR read
      #define IR_TOWER_SAMPLES 100
      IR_read=0;
        for(int i=0; i<IR_TOWER_SAMPLES; i++){
          IR_read += !digitalRead(receiver_pin);  //Negative logic
          delayMicroseconds(10);
        }
        myEnc_L_currpos = (myEnc_L.read()-myEnc_L_oldpos);
      ///  PRINTLN("(myEnc_L.read()-myEnc_L_oldpos)", myEnc_L_currpos ); 
      if(IR_read > IR_TOWER_SAMPLES/4 || (myEnc_L.read()-myEnc_L_oldpos) > tran2enc_pulse( ROBOT_CURVING_PERIMETER)){
        // Return if find IR tower

        if(IR_read > IR_TOWER_SAMPLES/4){         // IR Tower found. Reset robot_theta
          robot_theta = 0;
          PRINTLN2("IR_read", IR_read);
        }
        
        mouse_state = STATE_FORWARD;
      }else{ 
        if(sequence & 1){
          motors_right(SPEED_NORMAL, -SPEED_NORMAL);
          ///PRINTSLN("   ROTATE right");
        }else{
          ///PRINTSLN("   ROTATE left");
          motors_left(-SPEED_NORMAL, SPEED_NORMAL);
        }
      }
      break;

    case STATE_ABORT:
      PRINTSLN2("STATE_ABORT");
      motors_stop();
      servo_off();
      //Wait for Start button. Debounce
      button_start=0;

      //TIMEOUT 
      if(journey_state == STATEJ_TIMEOUT_RACE){
        while(1){
          turn_on_led_red(); delay(200); turn_off_led_red(); delay(200);
        }// WAIT FOR RESET!!!!
      }
      
      while (button_start<5){
          button_start = 0;
          for(int i=0; i<20; i++){
            if(!digitalRead(BUTTON_START_PIN))
              button_start += 1;
            delay(1);
          }
      }
      
      PRINTLN2("button_start = ", button_start);
      // END //Wait for Start button. Debounce
  
      mouse_state = STATE_FORWARD;
      race_time = millis();   //Start race timer
      break;

    case STATE_STOP:
        if (sensors == NONE) {
          motors_forward(SPEED_HIGH, SPEED_HIGH);
          mouse_state = STATE_FORWARD;
        } else if (sensors == SENSOR_RIGHT) {
          motors_left(-SPEED_NORMAL, SPEED_NORMAL);
          mouse_state = STATE_ROTATE_LEFT;
        }
        else  if (sensors == SENSOR_LEFT) {
          motors_right(SPEED_NORMAL, -SPEED_NORMAL);
          mouse_state = STATE_ROTATE_RIGHT;
        } else  if (sensors == BOTH) {
            if(sequence & 1){
                PRINTSLN("   SCRUMBLE ROTATE right");
                motors_right(SPEED_NORMAL, -SPEED_NORMAL);
                mouse_state = STATE_ROTATE_RIGHT;
            }else{
                PRINTSLN("   Scrumble ROTATE left");
                motors_left(-SPEED_NORMAL, SPEED_NORMAL);
                mouse_state = STATE_ROTATE_LEFT;
            }
          }
      break;

    case STATE_FORWARD:
        if (sensors == SENSOR_RIGHT) {
          motors_left(-SPEED_NORMAL, SPEED_NORMAL);
          mouse_state = STATE_ROTATE_LEFT;
        }
        else  if (sensors == SENSOR_LEFT) {
          motors_right(SPEED_NORMAL, -SPEED_NORMAL);
          mouse_state = STATE_ROTATE_RIGHT;
        } else  if (sensors == BOTH) {
          if(sequence & 1){
                PRINTSLN("   SCRUMBLE ROTATE right");
                motors_right(SPEED_NORMAL, -SPEED_NORMAL);
                mouse_state = STATE_ROTATE_RIGHT;
            }else{
                PRINTSLN("   Scrumble ROTATE left");
                motors_left(-SPEED_NORMAL, SPEED_NORMAL);
                mouse_state = STATE_ROTATE_LEFT;
            }
        } else {
          motors_forward(SPEED_HIGH, SPEED_HIGH);
        }
      break;

    case STATE_ROTATE_RIGHT:
          if (sensors == NONE ) {
            motors_forward(SPEED_HIGH, SPEED_HIGH);
            mouse_state = STATE_FORWARD;
          } else
            motors_right(SPEED_NORMAL, -SPEED_NORMAL);
      break;

    case STATE_ROTATE_LEFT:
        if (sensors == NONE)
        {
          motors_forward(SPEED_HIGH, SPEED_HIGH);
          mouse_state = STATE_FORWARD;
        } else
          motors_left(-SPEED_NORMAL, SPEED_NORMAL);
      break;

    case STATE_TOWER_ARRIVE:
      journey_state = STATEJ_FIND_BEGINING;
      
      servo_off();
      turn_on_led_red();        //LED on until end of race
      motors_stop();
      delay(500);
      mouse_state = STATE_ROTATE_180;
      //n_pulse_rotate = tran2enc_pulse(ROBOT_CURVING_PERIMETER/2); //180º rotation    
      n_pulse_rotate = tran2enc_pulse(300); //180º rotation    
      ///myEnc_L.write(0);
      ///myEnc_R.write(0);
      int speedL, speedR;
      
          ///myEnc_L_currpos = myEnc_L.read() - myEnc_L_oldpos;
          ///myEnc_R_currpos = myEnc_R.read() - myEnc_R_oldpos;
          myEnc_L_oldpos = myEnc_L.read();
          myEnc_R_oldpos = myEnc_R.read();
          
      break;

    case STATE_TIMEOUT_RACE:
      PRINTLN("race_time ", race_time);
      journey_state = STATEJ_TIMEOUT_RACE;
      mouse_state = STATE_ABORT;
      
      myEnc_L_oldpos = myEnc_L.read();   //Save current encoder position
      myEnc_R_oldpos = myEnc_R.read();
      break;

    case STATE_BEGIN_RETURN:
      servo_off();
      turn_on_led_blink();        //LED blink on end of race
      motors_stop();
      break;

    case STATE_ROTATE_180:    //Rotate right
           myEnc_L_currpos = myEnc_L.read();
           myEnc_R_currpos = myEnc_R.read();
           
          if(myEnc_L_currpos-myEnc_L_oldpos < n_pulse_rotate)  {
            speedL = 60;
          }else{
            speedL = 0;
            mouse_state = STATE_FORWARD;
            PRINTSLN(" STATE_ROTATE_180 END 1");   
          }
          
          if (myEnc_R_currpos-myEnc_R_oldpos < n_pulse_rotate)  {
            speedR = 60;
          }else{
            speedR = 0;
            mouse_state = STATE_FORWARD;
            PRINTSLN(" STATE_ROTATE_180 END 2");
          }

          motors_forward(speedL, -speedR);
      break;
  }
}

void motors_stop() {
  PRINTS5("MStop:\t");
  PRINTS5("SSet 0|0");
  speed_L = pwmOut(0, ML, MOTOR_BREAK);
  speed_R = pwmOut(0, MR, MOTOR_BREAK);
  
  PRINT5(" S ", speed_L); PRINTLN5(" | ", speed_R);
}

void motors_forward(int speedL, int speedR) {   
  PRINTS5("MFwd\t");
  PRINT5(" Sset ", speed_L); PRINTLN5(" | ", speed_R);
  if(speedL == 0)
    pwmOut(speedL, ML);
  speed_L = pwmOut(speedL, ML);
  speed_R = pwmOut(speedR, MR);
  PRINT5(" S ", speed_L); PRINTLN5(" | ", speed_R);
}

void motors_right(int speedL, int speedR) {
  cnt_motors_right++;   //Counter of turns right
 
  PRINTS5("MRight\t");
  PRINTS5("SSet 125|-125");
  speed_L = pwmOut( speedL, ML);
  speed_R = pwmOut( speedR, MR);
  
  PRINT5(" S ", speed_L); PRINTLN5(" | ", speed_R);
}

void motors_left(int speedL, int speedR) {
  cnt_motors_right--;   //Counter of turns right
  
  speed_L = pwmOut(speedL, ML);
  speed_R = pwmOut(speedR, MR);
 
  PRINT5(" S ", speed_L); PRINTLN5(" | ", speed_R);
}

void SonarSetup(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

int SonarPing(int triggerPin, int echoPin, int timeout) {
  float duration;
  int distance;
  digitalWrite(triggerPin, LOW);      // clean line to trigger
  delayMicroseconds(2);               // Added this line
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);              // trigger duration
  digitalWrite(triggerPin, LOW);      //stop trigger
  duration = pulseIn(echoPin, HIGH, timeout);  //receive duration of pulse
  ///Serial.print(" duration "); Serial.print(duration, 0);
  distance = (duration / 2) / 29 * 10; // compute distance in mm
  if (distance == 0) distance = 11111; // If timeout, reading is 0. Convert to infinite

  return distance;
}

uint8_t SonarScan() {
  // uint8_t sensors_mouse_state;
  distance_L  = SonarPing(trigPin_L,  echoPin_L,  timeout_L);
  distance_FL = SonarPing(trigPin_FL, echoPin_FL, timeout_FL);
  distance_R  = SonarPing(trigPin_R,  echoPin_R,  timeout_R);
  distance_FR = SonarPing(trigPin_FR, echoPin_FR, timeout_FR);
  distance_F  = SonarPing(trigPin_F,  echoPin_F,  timeout_F);
  distance_F  = SonarPing(trigPin_F,  echoPin_F, timeout_F);
  
  PRINT4("Sensors: ", distance_L); PRINT3("\t", distance_FL); PRINT3("\t", distance_FR); PRINT3("\t", distance_R);

  mouse_state_table_entry = 0;
  if (distance_L < distance_L_max)
    mouse_state_table_entry |= 0b01000;
  if (distance_FL < distance_FL_max)
    mouse_state_table_entry |= 0b00100;
  if (distance_FR < distance_FR_max)
    mouse_state_table_entry |= 0b00010;
  if (distance_R < distance_R_max)
    mouse_state_table_entry |= 0b00001;
  if (distance_F < distance_F_max)
    mouse_state_table_entry |= 0b10000;
    
  return (mouse_state_table[mouse_state_table_entry]);
}

// Motor Driver
int pwmOut(int speed_in, int motor, int motor_stop) {
  int average = 0;

  // Limite speed to max
  if (speed_in > SPEED_HIGH)
    speed_in = SPEED_HIGH;
  else if (speed_in < -SPEED_HIGH)
    speed_in = -SPEED_HIGH;

  if (motor == ML) {
    // Shift motor buffer
    for (byte i = 0; i < SPEED_BUF_SIZE - 1; i++) {
      speed_bufL[SPEED_BUF_SIZE - 1 - i] = speed_bufL[SPEED_BUF_SIZE - 2 - i];
    }
    speed_bufL[0] = speed_in;
    // Average calculation
    for (byte i = 0; i < SPEED_BUF_SIZE; i++) {
      average += speed_bufL[i];
    }
    average /= SPEED_BUF_SIZE;

    if (motor_stop == MOTOR_BREAK) { // HARD STATE_STOP
      digitalWrite(motorA_1A, LOW);   //Brake
      digitalWrite(motorA_2A, LOW);
    } else {
      if (average < 0) { //forward
        digitalWrite(motorA_1A, LOW);   //Disengage the Brake for Channel A
        analogWrite(motorA_2A, abs(average));   //Spins the motor on Channel A at full speed
      }
      else {        //backward
        analogWrite(motorA_1A, abs(average));   //Spins the motor on Channel A at full speed
        digitalWrite(motorA_2A, LOW);   //Disengage the Brake for Channel A
      }
    }
  } else { //motor == MR
    // Shift in
    for (byte i = 0; i < SPEED_BUF_SIZE - 1; i++) {
      speed_bufR[SPEED_BUF_SIZE - 1 - i] = speed_bufR[SPEED_BUF_SIZE - 2 - i];
    }
    speed_bufR[0] = speed_in;
    // Average calculation
    for (byte i = 0; i < SPEED_BUF_SIZE; i++) {
      average += speed_bufR[i];
    }
    average /= SPEED_BUF_SIZE;
    ///Serial.print("S "); for(byte i=0; i<SPEED_BUF_SIZE; i++){Serial.print(speed_bufR[i]); Serial.print("\t");}; Serial.print("\t A "); Serial.println(average);

    if (motor_stop == MOTOR_BREAK) {  // HARD STATE_STOP
      digitalWrite(motorB_1A, LOW);   //Brake
      digitalWrite(motorB_2A, LOW);
    } else {
      if (average < 0) { //forward
        digitalWrite(motorB_1A, LOW);   //Disengage the Brake for Channel A
        analogWrite(motorB_2A, abs(average));   //Spins the motor on Channel A at full speed
      }
      else {        //backward
        analogWrite(motorB_1A, abs(average));   //Spins the motor on Channel A at full speed
        digitalWrite(motorB_2A, LOW);   //Disengage the Brake for Channel A
      }
    }
  }//IF END

  return average;
}// Motor Driver END

// output: -90:90
int sweepIR() {
  int local_servo_ang = 999;
  unsigned char ir_read = 0;

  ///PRINTLN2("servo_current_pos", servo_current_pos);
  // Return if find IR tower
  #define IR_TOWER_SAMPLES 100
  for(int i=0; i<IR_TOWER_SAMPLES; i++){
    ir_read += !digitalRead(receiver_pin);  //Negative logic
    delayMicroseconds(10);
  }
  PRINTLN2("ir_read", ir_read);
  if (ir_read > IR_TOWER_SAMPLES/4) {     // IR Detect tower
    local_servo_ang = servo_current_pos-90; // Convert ang from servo range (0ª; 180º) to robot range(-90º; 90ª) 

    PRINTLN2("IR TOWER DETECTED", local_servo_ang);
    return (local_servo_ang);             // Stop sweep
  }

  if (servo_old_pos < 179 && servo_old_pos < servo_current_pos) {
    servo_old_pos += 5;
    servo_current_pos += 5;
  }
  else if (servo_current_pos >= 180) {
    servo_current_pos = 179;
    servo_old_pos = 180;
  }
  else if (servo_current_pos <= 4) {
    servo_current_pos = 5;
    servo_old_pos = 4;
  }
  else if (servo_old_pos > 5 && servo_old_pos > servo_current_pos) {
    servo_old_pos -= 5 ;
    servo_current_pos -= 5;
  }
  ir_servo.write(servo_current_pos);

  return (local_servo_ang);
}

void servo_off(){
    ///ir_servo.detach();
}

// LED RED /////////////////////////////////////////////////////////////////////////////////////////////////////////////
void turn_on_led_red() {
  digitalWrite(LED_PIN, HIGH);
}

void turn_off_led_red() {
  digitalWrite(LED_PIN, LOW);
}

void turn_on_led_blink(){
    
}

// IR_floor_scan ////////////////////////////////////////////////////////////
// Read all sensors x times and count number of LOW readings.
// input: 
// out: 0 : white floor. | n: binary number with sensor cobination taht detected black floor.
unsigned char IR_floor_scan(){
unsigned char ret =0;
unsigned char ir_floor_sensor[IR_SENSOR_NUM_SENSORS]={0, 0, 0}; //  

  // Scan Sensors
  for(int i=0; i<IR_SENSOR_NUM_READINGS; i++){    
    ///PRINT2("S", i); PRINTS2(" ");
    for(int s=0; s<IR_SENSOR_NUM_SENSORS; s++){
      i_tmp = analogRead(IR_FLOOR_PIN[s]);
      ///PRINT2("\t", i_tmp);
      if(i_tmp > IR_FLOOR_TH[s])  // Read black
        ir_floor_sensor[s] += 1;
    }  
  }// END FOR

  // Evakuate sensors readings
  for(int s=0; s<IR_SENSOR_NUM_SENSORS; s++){
    if (ir_floor_sensor[s] > IR_SENSOR_NUM_READINGS/2)  // More than 50% readings 
      ret |= 1<<s;
  }
  ///PRINT3("Sensors return ", ret);
  return ret; 
}

// ISR ///////////////////////////////////////////////////////////////////////////////////////////////////////
void int_stop_pressed() { // ISR stop button
  if (mouse_state != STATE_ABORT) {
    Serial.print("State: STATE_ABORT\n");
    mouse_state = STATE_ABORT;
  }
}

double getAngDistL(){
double l_ang;

  ///TEST 
  mainEnc_L_currpos = myEnc_L.read();
  ///mainEnc_L_currpos = 799;
  
  ///PRINT("mainEnc_L_currpos ", mainEnc_L_currpos);
  l_ang = (mainEnc_L_currpos - mainEnc_L_oldpos)*360/WHEEL_IMP_ROT; //360/1430 = 0,25 aproximation: 360/1430 = 1/4
  mainEnc_L_oldpos = mainEnc_L_currpos;
  return (l_ang);
}

double getAngDistR(){
double l_ang;

  ///TEST 
  mainEnc_R_currpos = myEnc_R.read();
   ///mainEnc_R_currpos = -799;
   
  ///PRINTLN("mainEnc_R_currpos ", mainEnc_R_currpos);
  l_ang = (mainEnc_R_currpos - mainEnc_R_oldpos)*360/WHEEL_IMP_ROT; //360/1430 = 0,25 aproximation: 360/1430 = 1/4
  mainEnc_R_oldpos = mainEnc_R_currpos;
  return (l_ang);
}

void updatePosition(){
  i_tmp = millis();
  double d_aux;
   // get the angular distance traveled by each wheel since the last update
   double leftDegrees  = getAngDistL(); //_leftWheel->getDistance();
   double rightDegrees = getAngDistR(); //_rightWheel->getDistance();

///PRINT2("leftDegrees ", leftDegrees); PRINTLN2(" rightDegrees ", rightDegrees);

   // convert the angular distances to linear distances
   double dLeft = leftDegrees / K;
   double dRight = rightDegrees / K;
///PRINTLN2("(360/WHEEL_PERIMETER) ", K); 
///PRINT2("dLeft ", dLeft); PRINTLN2(" dRight ", dRight);

   // calculate the length of the arc traveled by Colin
   double dCenter = (dLeft + dRight) / 2;
///PRINTLN2("dCenter ", dCenter); 
   // calculate Colin's change in angle
   
   double phi = (dRight - dLeft) / WHEEL_DIST;
///PRINTLN2("phi ", phi); 
   // add the change in angle to the previous angle
   robot_theta += phi;
///PRINTLN2("robot_theta ", robot_theta);
 
   // constrain robot_theta to the range 0 to 2 pi
   //if (robot_theta > 2.0 * pi) robot_theta -= pix2;
   //if (robot_theta < 0.0) robot_theta += pix2;
   if (robot_theta > pi) robot_theta -= pi;
   if (robot_theta < -pi) robot_theta += pi;

   // update Colin's x and y coordinates
   robot_xPosition += dCenter * cos(robot_theta);
   robot_yPosition += dCenter * sin(robot_theta);
   ///PRINTLN2("updatePosition compute time: ", millis()-i_tmp);
   PRINT2("x ", robot_xPosition); PRINT2(" y ", robot_yPosition);
   ///PRINT2("dCenter ", dCenter); 
   PRINTLN2(" robot_theta ", robot_theta);
}

int evaluate_azimute(){
  if(robot_theta >0)
    return azimute_LEFT;    //Tower is on LEFT side
  else
    return azimute_RIGHT;   //Tower is on RIGHT side
}

void printState(){
if(mouse_state != last_mouse_state){
    last_mouse_state = mouse_state;
    PRINTS("\t");
    switch(mouse_state){
      case (0): PRINTSLN("STATE_STOP \t"); break;
      case (1): PRINTSLN("STATE_FORWARD \t"); break;
      case (2): PRINTSLN("STATE_ROTATE_RIGHT \t"); break;
      case (3): PRINTSLN("STATE_ROTATE_LEFT \t"); break;
      case (4): PRINTSLN("STATE_ABORT \t"); break;
      case (5): PRINTSLN("STATE_ROTATE_TETA_RIGHT \t"); break;
      case (6): PRINTSLN("STATE_ROTATE_TETA_LEFT \t"); break;
      case (7): PRINTSLN("STATE_IR_TOWER_DETECTED \t"); break;
      case (8): PRINTSLN("STATE_TOWER_ARRIVE \t"); break;
      case (9): PRINTSLN("STATE_TIMEOUT_RACE \t"); break;
      case (10): PRINTSLN("STATE_BEGIN_RETURN \t"); break;
      case (11): PRINTSLN("STATE_ROTATE_IR_SCAN \t"); 
      if(sequence & 1){
          PRINTSLN("   ROTATE right");
        }else{
          PRINTSLN("   ROTATE left");
        }
      break;
      case (12): PRINTSLN("STATE_ROTATE_180 \t"); break;
    }
  }
  if(journey_state != last_journey_state){
    last_journey_state = journey_state;
    PRINTS("\t");
    switch(journey_state){
      case (0): PRINTSLN("STATE_FIND_IR_TOWER"); break;
      case (1): PRINTSLN("STATE_FIND_BEGINING"); break;
    }
  }  
}
