#include <arduino.h>

#define Null_Value 255
#define Invalid_Value 254
#define turn_on 0
#define turn_off 1
#define power_switch 10 
#define push_butt 2
#define Lauf_LED 9
#define Op_LED 10
#define Blink_freq 12

const unsigned int lng_press = 4000;
const unsigned int srt_press = 2000;

unsigned int butt_cnt=0;
volatile bool butt_flag=0;
volatile bool prev_butt_flag=0;

volatile unsigned int ctrl_sign=0;  //Traffic Control

void UART_Init(void);
char UART_com(void);

void Power_con_init(void);
void Power_con(void);

void Butt_con_init(void);
void Butt_con(void);

void LED_con_Init(void);
void LED_con(unsigned int);


void setup() {
  UART_Init();
  Power_con_init();
  LED_con_Init();
  Butt_con_init();
}
//------------------------------------------------------------------------------------
//  UART Initialization
//  ~ Sets serial communication to 38.4k
//  - Inputs: None
//  - Outputs: Prints UART Initialized message
//  - Side effects: Uart communication is initialized 
//------------------------------------------------------------------------------------
void UART_Init(){
  Serial.begin(38400); //Initialize serial communication with 38.4k communication baud 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  ctrl_sign |= 0x001; //Confirmation that UART is initialized - B 0000 0000 0000 0001
  //Serial.print("UART Initialized");  
}

//------------------------------------------------------------------------------------
//  UART Utilization
//  ~ Polls UART buffer
//    Receives UART data if it is available
//    Returns processed info back in upper loop
//  - Inputs: None (Indirectly takes data from UART Input buffer)
//  - Outputs: Received command from UART in form of char
//  - Side effects: None
//------------------------------------------------------------------------------------ 
char UART_com(){
  byte ReceivedByte = 0;
  if (Serial.available()>0){
    ReceivedByte = Serial.read();
    }
  if (ReceivedByte < Null_Value) {
    switch (ReceivedByte){
      
      case turn_on: //Check if Turn On is received
        return turn_on; //And return turn ON command
        break;
      
      case turn_off: //Check if Turn Off is received
        return turn_off; //And return turn OFF command
        break;
      
      default:  //Anything else received
        return Invalid_Value; //Else Return Invalid_Value
        break;
      }
    }
  else {
    return Null_Value;
    } 
  return Null_Value;
}

//-----------------------------------------------------------------------------------
//  Power Control Initialization
//  ~ Configures Power switch pin and sets it to low
//  - Inputs: None
//  - Outputs: None
//  - Side effects: Power switch pin set to low (turned off)
//-----------------------------------------------------------------------------------
void Power_con_init(){
  pinMode(power_switch, INPUT_PULLUP); //Set pin as input and set pull up impedance
  ctrl_sign |= 0x002; //Confirm that input switch is initialized - B 0000 0000 0000 0010
  
  //Serial.print("Power pin initialized");  //Print out that power pin is holding 
  //Serial.print(con_sign,BIN);
}

//-----------------------------------------------------------------------------------
//  Button Control Initialization
//  ~ Initializes Push button interrupt
//  - Inputs: None
//  - Outputs: None
//  - Side effects: Push Button Interrupt is activated 
//-----------------------------------------------------------------------------------
void Butt_con_init(){
  attachInterrupt (digitalPinToInterrupt (push_butt),Butt_con_ISR,CHANGE);
  butt_cnt =0; //reset push button counter
  ctrl_sign |= 0x004; //Confirm that push button interrupt is initialized; - B 0000 0000 0000 0100
  //Serial.print("Button Interrupt initialized");
  }

//-----------------------------------------------------------------------------------
//  Button Control ISR
//  ~ ISR for Button Control
//  - Inputs: None
//  - Outputs: None
//  - Side effects: Checks if button is pushed and sets a flag, if it is
//-----------------------------------------------------------------------------------
void Butt_con_ISR(){  
  if (digitalRead (push_butt)== LOW){ //Check if button is pressed
    butt_flag = true; //set flag for pressed
  }
  else {
    butt_flag = false;  //reset flag for not pressed
    }
  }

//-----------------------------------------------------------------------------------
//  Button Control
//  ~ Push button process information
//  - Inputs: None
//  - Outputs: None
//  - Side effects: Detection of pressed button status
//-----------------------------------------------------------------------------------  
void Butt_con(){
  if (butt_flag) {  // Check if button is pressed
    if (prev_butt_flag){  //and it is pressed previously
      butt_cnt +=0x01; //increase counter on time pressed
      }
    else {
      prev_butt_flag = butt_flag; //if not, set previously pressed flag 
      }
    }
  if ((!butt_flag)){  //if button is not any more pressed
    if (prev_butt_flag){  //and it was previously pressed, determine how long was button pressed
      if (butt_cnt > lng_press){ 
        ctrl_sign |= 0x008;  //set long press flag - B 0000 0000 0000 1000
        }
      else if (butt_cnt>srt_press){
        ctrl_sign |= 0x010;  //set short press flag - B 0000 0000 0001 0000 
        }
      prev_butt_flag=false; // reset previous flag
      butt_cnt=0; //reset time pressed flag
      }
    }
  }

//-----------------------------------------------------------------------------------
//  LED Signal Initialization
//  ~ LED Indication Initialization
//  - Inputs: None
//  - Outputs: None
//  - Side effects: Indication LEDs are set and ready for operation
//-----------------------------------------------------------------------------------
void LED_con_Init(){
  pinMode(Op_LED, OUTPUT);
  ctrl_sign |= 0x40;  //Confirm that turn on LED is set
  pinMode(Lauf_LED, OUTPUT);
  ctrl_sign |= 0x80; //Confirm that running LED is set
  }

//-----------------------------------------------------------------------------------
//  LED Control 
//  ~ LED Control 
//  - Inputs: Traffic Control variable
//  - Outputs: None
//  - Side effects: 
//---------------------------------------------------------------
void LED_con(byte led_indication){

 // if (led_indication && B100000000){
    if (led_indication && 0x180){
      analogWrite(Lauf_LED, Blink_freq);
      //ctrl_sign ^= B100000000;
    }
    else{
      digitalWrite(Lauf_LED, HIGH);
      //ctrl_sign ^= B110000000;
    }

    if (led_indication && 0x140){
      analogWrite(Op_LED, Blink_freq);
      //ctrl_sign ^= B100000000;
    }
    else{
      digitalWrite(Op_LED, HIGH);
      //ctrl_sign ^= B101000000;
    }
 // }
}


// Main Loop
//---------------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:

}
