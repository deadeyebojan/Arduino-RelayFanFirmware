#include <arduino.h>

#define Null_Value 255
#define Invalid_Value 254
#define turn_on 0
#define turn_off 1
#define power_switch 10 
#define push_butt 2
#define Lauf_LED 9
#define Op_LED 10
#define Relay_pin 6
#define Blink_freq 12

//Test out
const unsigned int lng_press = 4000;
const unsigned int srt_press = 2000;

unsigned int butt_cnt=0;
volatile bool butt_flag=0;
volatile bool prev_butt_flag=0;

volatile unsigned int ctrl_sign=0;  //Traffic Control

byte UART_Init(byte);
byte UART_com(byte);

byte Power_con_init(byte);
byte Power_con(byte);

byte Butt_con_init(byte);
byte Butt_con(byte);

byte LED_con_Init(byte);
byte LED_con(byte);

void Butt_con_ISR();

void setup()
{
  ctrl_sign = Power_con_init(ctrl_sign);
  ctrl_sign = LED_con_Init(ctrl_sign);
  ctrl_sign = Butt_con_init(ctrl_sign);
  UART_Init(ctrl_sign);
}
//------------------------------------------------------------------------------------
//  UART Initialization
//  ~ Sets serial communication to 38.4k
//  - Inputs: None
//  - Outputs: Prints UART Initialized message
//  - Side effects: Uart communication is initialized 
//------------------------------------------------------------------------------------
byte UART_Init(byte ctrl_status)
{
  Serial.begin(38400); //Initialize serial communication with 38.4k communication baud 
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB
  }
  //ctrl_sign |= 0x01; //Confirmation that UART is initialized - B 0000 0001 - Doesnt make sense to signalize UART communication. It can be written by UART
  
  if (ctrl_status && 0x02)  //Power pin Initializatied
  {
    Serial.println("Power Control OK");
  }
  else 
  {
    Serial.println("Power Control NOK");
  }
  if (ctrl_status && 0x04)  //Button pin initialized
  {
    Serial.println("Button Control OK");
  }
  else
  {
    Serial.println("Button Control NOK");
  }
  if (ctrl_status && 0x20) //Relay pin initialized
  {
    Serial.println("Relay Control OK");
  }
  else
  {
    Serial.println("Relay Control NOK");
  }
  if (ctrl_status && 0x40)  //LED Indication initialized
  {
    Serial.println("LED Control OK");
  }
  else
  {
    Serial.println("LED Control NOK");
  }
  
  
  //Serial.print("");  
}

//------------------------------------------------------------------------------------
//  UART com
//  ~ UART Communication
//    Polls UART buffer
//    Receives UART data if it is available
//    Returns processed info back in upper loop
//  - Inputs: ctrl_sign 
//            (Indirectly takes data from UART Input buffer)
//  - Outputs: Received command from UART in form of byte
//  - Side effects: None
//------------------------------------------------------------------------------------ 
byte UART_com(byte UART_ctrl)
//NEED TO INTRODUCE POINTER CONTROL HERE
{
  byte ReceivedByte = 0;
  if (Serial.available()>0)
    { /*
      If there is something on Serial comm
      Store it and set "something new" flag
      */
    ReceivedByte = Serial.read();
    UART_ctrl |= 0x01;
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
  /*
  Code Refactored - Removed this part
  else {
    return Null_Value;
    }*/ 
  return Null_Value;
}

//-----------------------------------------------------------------------------------
//  Power Control Initialization
//  ~ Configures Power switch pin and sets it to low
//  - Inputs: None
//  - Outputs: None
//  - Side effects: Power switch pin set to low (turned off)
//-----------------------------------------------------------------------------------
byte Power_con_init(byte Pwr_ctrl)
{
  pinMode(power_switch, INPUT_PULLUP); //Set pin as input and set pull up impedance
  Pwr_ctrl |= 0x03; //Confirm that input switch is initialized - B 0000 0011 and set "something new" flag
  return Pwr_ctrl;
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
byte Butt_con_init(byte butt_ctrl)
{
  attachInterrupt (digitalPinToInterrupt (push_butt),Butt_con_ISR,CHANGE);
  butt_cnt =0; //reset push button counter
  butt_ctrl |= 0x05; //Confirm that push button interrupt is initialized; - B 0000 0101
  return butt_ctrl;
  //Serial.print("Button Interrupt initialized");
}

//-----------------------------------------------------------------------------------
//  Button Control ISR
//  ~ ISR for Button Control
//  - Inputs: None
//  - Outputs: None
//  - Side effects: Checks if button is pushed and sets a flag, if it is
//-----------------------------------------------------------------------------------
void Butt_con_ISR()
{
  /*
  Used as an example
  digitalWrite(pin, digitalRead(pin) == HIGH ? LOW : HIGH);

  desired statement
  butt_flag = (digitalRead(push_butt)==LOW ? LOW : HIGH);
  */
  if (digitalRead (push_butt)== LOW)
  { //Check if button is pressed
    butt_flag = true; //set flag for pressed
  }
  else 
  {
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
byte Butt_con(byte butt_ctrl)
{
  if (!(butt_ctrl || 0x00)) //Check if there is something new
  {
    butt_ctrl &= 0xE7;  //Nothing new, reset short and long press flags
    
  }  
  if (butt_flag) 
  {  // Check if button is pressed
    if (prev_butt_flag)
    {  //and it is pressed previously
      butt_cnt +=0x01; //increase counter on time pressed
    }
    else
    { //if not, set previously pressed flag
      prev_butt_flag = butt_flag;  
    }
  }
  if ((!butt_flag))
  {  //if button is not any more pressed
    if (prev_butt_flag)
    {  //and it was previously pressed, proceed to determine how long was button pressed
      if (butt_cnt > lng_press)
      {
        butt_ctrl |= 0x09; //set long press flag and set "something new" flag - B 0000 1001
      }
      else if (butt_cnt>srt_press)
      {
        butt_ctrl |= 0x11; //set short press flag and set "something new" flag - B 0001 0001
      }
      //If button was realy short pressed
      //return everything in original state
      prev_butt_flag=false; // reset previous flag
      butt_cnt=0; //reset time pressed flag
    }
  }
  return butt_ctrl;
}

//-----------------------------------------------------------------------------------
//  LED Signal Initialization
//  ~ LED Indication Initialization
//  - Inputs: None
//  - Outputs: None
//  - Side effects: Indication LEDs are set and ready for operation
//-----------------------------------------------------------------------------------
byte LED_con_Init(byte LED_ctrl)
{
  pinMode(Op_LED, OUTPUT);  
  pinMode(Lauf_LED, OUTPUT);
  //digitalWrite(Op_LED, HIGH);
  analogWrite(Op_LED, Blink_freq);
  digitalWrite(Lauf_LED, HIGH);
  LED_ctrl |= 0x41;  //Confirm that LED indication is set and set "something new" flag - B 0100 0001
  analogWrite(Op_LED, Blink_freq);
  //ctrl_sign |= 0x80; //Confirm that running LED is set - B 1000 0000 - Had to erase this as there arent enough flags in Traffic Control
  return LED_ctrl;
}


//-----------------------------------------------------------------------------------
//  LED Control 
//  ~ LED Control 
//  - Inputs: Traffic Control variable
//  - Outputs: None
//  - Side effects: 
//-----------------------------------------------------------------------------------
byte LED_con(byte led_ctrl)
{ 
  if (led_ctrl && 0x01) //Check if there is something new in traffic control
  {
    /*
    if (led_ctrl && 0x40) //Check "Device ON" flag
    {
      analogWrite(Op_LED, Blink_freq);
      //led_ctrl ^= 0x01; //Remove "something new" flag
    }
    */
    if (led_ctrl && 0x10) //Check "Running" flag
    {
      analogWrite(Lauf_LED, Blink_freq);
      //led_ctrl ^= 0x01;  //Remove "something new" flag
    }
    if (led_ctrl && 0x30)
    {
      digitalWrite(Lauf_LED, HIGH); //Turn off "Running" pin
    }
    /*else
    {
      digitalWrite(Lauf_LED, HIGH); //Turn off "Running" pin
      //led_ctrl &= 0x7F;             //Remove "Running" flag
    }
    */
    
/*  Code Refactored - thus this part removed
    if (led_indication && 0x41){
      analogWrite(Op_LED, Blink_freq);
      ctrl_sign ^= 0x40;
    }
    else{
      digitalWrite(Op_LED, HIGH);
      ctrl_sign ^= 0x00;
    }
*/
  }
 return led_ctrl;
}

//-----------------------------------------------------------------------------------
//  Relay Con Init
//  ~ Relay Control Initialization
//  - Inputs: None
//  - Outputs: None
//  - Side effects: Initialize pins used for control of relay
//-----------------------------------------------------------------------------------
byte Relay_con_Init(byte relay_ctrl_init)
{ //Initialize Relay pin, set it as HIGH and set "something new" flag
  pinMode(Relay_pin, OUTPUT);
  digitalWrite(Relay_pin, HIGH);
  relay_ctrl_init |=0x21; //B 0020 0001
  return relay_ctrl_init; 
  //ctrl_sign |= 0x21;
}

//-----------------------------------------------------------------------------------
//  Relay Con 
//  ~ Relay Control 
//  - Inputs: ctrl_sign variable, 0x21 flags
//  - Outputs: None
//  - Side effects: Turns relay for Fans on and off
//-----------------------------------------------------------------------------------
byte Relay_con(byte relay_ctrl)
{ 
  if (relay_ctrl && 0x01) 
  { //Check if there is something new in traffic control
    if (relay_ctrl && 0x30)
    { //Check if relay is turned on and short press is performed
      digitalWrite(Relay_pin, HIGH);  //Turn off relay
      relay_ctrl ^= 0x20; //B 1000 0000 //Reset relay flag
    }
    else if (relay_ctrl && 0x10)  //Else check if relay is turned off and short press is performed
      { //
        digitalWrite(Relay_pin, LOW); //Turn on relay
        relay_ctrl |= 0x20; //B 1000 0000 //Set relay flag 
      }
  return relay_ctrl;      
  }

}
  
//-----------------------------------------------------------------------------------
//
// Main Loop
//
//-----------------------------------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:

}
