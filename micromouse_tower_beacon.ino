// Microrato 2018 - IR TOWER CONTROLLER
// 38 khz modulated on a 600hz pwm at 30% on rate
// 38 khz period is 26.3157895 uSeconds
// 600Hz period is 1666.66666 (1667) uSeconds
//
// On time is 30% of 1667 uSec onTime = 0.3 * 1667 = 500.1 uSec
// so the loop will be on 500 uSecs and of 1167 uSecs
// 
// 38khz timing is 26.315 / 2 = 13.16


#define ir_led 10

void setup() {
  pinMode(ir_led, OUTPUT);
  }


void Mod38khz(int duration){
  int sweep_loop = duration / 26; // sets how many times needs to loop -full period
  
  for (int i=0; i<= sweep_loop; i++){
    PORTB = B00000100;
    //digitalWrite(ir_led, HIGH); //half period on 
    delayMicroseconds(14); //14 to correct error read at osciloscope
    PORTB = B00000000;
    //digitalWrite(ir_led, LOW);  //half period off 
    delayMicroseconds(14);
  }
}
void loop() {
  Mod38khz(500);
  delayMicroseconds(1167); // 500 + 1167 gives the period of a 600 hz wave with a 30% duty cycle

}
