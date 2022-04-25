/* -------------------------------------------------------- */
/* -------------------JOYSTICK SETTINGS-------------------- */
/* -------------------------------------------------------- */

/*
 * Hello! Feel free to adjust these values within the recommended boundaries 
 * to configure the joystick as needed. Enjoy!
 */

/*
 * SENSITIVITY OFFSET
 * 
 * Description: 
 * - Increase this value to widen the joystick deadzone
 * - Decrease this value to narrow the joystick deadzone
 * - 6 is the recommended minimum!
 */

#define SENSITIVITY_OFFSET 6  

/*
 * MAX_SPEED
 * 
 * Description:
 * - This value limits the maximum speed of the motors. 
 * - 255 is the maximum possible value!
 */
#define MAX_SPEED 190

/*
 * MIN_SPEED
 * 
 * Description:
 * - This value limits the minimum speed of the motors.
 * - 40 is highly recommended to be the minimum possible value!
 */
#define MIN_SPEED 40

/* -------------------------------------------------------- */
/* ---------------END OF JOYSTICK SETTINGS----------------- */
/* -------------------------------------------------------- */

// MOTOR STATUS
#define BRAKE 0
#define CW    1
#define CCW   2

// MOTOR 1
#define MOTOR_A1_PIN  7
#define MOTOR_B1_PIN  8
#define PWM_MOTOR_1   5
#define EN_PIN_1      A0
#define MOTOR_1       0

// MOTOR 2
#define MOTOR_A2_PIN  4
#define MOTOR_B2_PIN  9
#define PWM_MOTOR_2   6
#define EN_PIN_2      A1
#define MOTOR_2       1

// JOYSTICK
#define X_AXIS_PIN   A4
#define Y_AXIS_PIN   A5

// DATA
int xData = 0;
int yData = 0;
int leftSpeed = 0;
int rightSpeed = 0;
float slope = 0.0;
int leftIntercept = 0;
int rightIntercept = 0;
float speedMultiplier = 1.0;
int upperLimit = 127 + SENSITIVITY_OFFSET;
int lowerLimit = 127 - SENSITIVITY_OFFSET;

const byte ledPin = 13;//digital pin to output state of wheels
const byte interruptPin_wireless = 2;//digital pin input from wireless killswitch relay
const byte interruptPin_wired = 3;//digital pin input from manual killswitch, a push button
//global to indicate status of wheels, if HIGH then allows wheels to move, if LOW then keeps brake on
volatile byte state = LOW;//defaults to LOW, thus when powered on, initially can not drive
volatile int a = 0, b = 0;//a <= pin 2, b <= pin 3

#define LED                         LED_BUILTIN  // digital pin connected to LED, for testing of switch code only
bool    led_status =                        LOW; // start with LED off, for testing of switch code only

int     button_switch =                       3; // external interrupt pin

#define switched                            true // value if the button switch has been pressed
#define triggered                           true // controls interrupt handler
#define interrupt_trigger_type            RISING // interrupt triggered on a RISING input
#define debounce                              10 // time to wait in milli secs

volatile  bool interrupt_process_status = {
  !triggered                                     // start with no switch press pending, ie false (!triggered)
};
bool initialisation_complete =            false; // inhibit any interrupts until initialisation is complete

//
// ISR for handling interrupt triggers arising from associated button switch
//
void button_interrupt_handler()
{
  if (initialisation_complete == true)
  { //  all variables are initialised so we are okay to continue to process this interrupt
    if (interrupt_process_status == !triggered) {
      // new interrupt so okay start a new button read process -
      // now need to wait for button release plus debounce period to elapse
      // this will be done in the button_read function
      if (digitalRead(button_switch) == HIGH) {
        // button pressed, so we can start the read on/off + debounce cycle wich will
        // be completed by the button_read() function.
        interrupt_process_status = triggered;  // keep this ISR 'quiet' until button read fully completed
      }
    }
  }
} // end of button_interrupt_handler

bool read_button() {
  int button_reading;
  // static variables because we need to retain old values between function calls
  static bool     switching_pending = false;
  static long int elapse_timer;
  if (interrupt_process_status == triggered) {
    // interrupt has been raised on this button so now need to complete
    // the button read process, ie wait until it has been released
    // and debounce time elapsed
    button_reading = digitalRead(button_switch);
    if (button_reading == HIGH) {
      // switch is pressed, so start/restart wait for button relealse, plus end of debounce process
      switching_pending = true;
      elapse_timer = millis(); // start elapse timing for debounce checking
    }
    if (switching_pending && button_reading == LOW) {
      // switch was pressed, now released, so check if debounce time elapsed
      if (millis() - elapse_timer >= debounce) {
        // dounce time elapsed, so switch press cycle complete
        switching_pending = false;             // reset for next button press interrupt cycle
        interrupt_process_status = !triggered; // reopen ISR for business now button on/off/debounce cycle complete
        return switched;                       // advise that switch has been pressed
      }
    }
  }
  return !switched; // either no press request or debounce period not elapsed
} // end of read_button function



//called once at startup
void setup() {
  // Setup Motor 1 (LEFT MOTOR)
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(EN_PIN_1, OUTPUT);

  // Setup Motor 2 (RIGHT MOTOR)
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);

  // Setup Joystick
  pinMode(X_AXIS_PIN, INPUT);
  pinMode(Y_AXIS_PIN, INPUT);
  slope = float(MAX_SPEED - MIN_SPEED) / 88.0;
  leftIntercept = MIN_SPEED - (16 * slope);
  rightIntercept = MAX_SPEED + (152 * slope);
  
  //output state variable
  pinMode(ledPin, OUTPUT);
  
  //setup dig pin 2 as input
  pinMode(interruptPin_wireless, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin_wireless), killswitch, CHANGE);//sensitive to any change to input
  
  //setup dig pin 3 as input
//  pinMode(interruptPin_wired, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(interruptPin_wired), killswitch2, CHANGE);//sensitive to only rising edge, from 0 to 1

  pinMode(LED, OUTPUT);
  pinMode(button_switch, INPUT);
  attachInterrupt(digitalPinToInterrupt(button_switch),
                  button_interrupt_handler,
                  interrupt_trigger_type);
  initialisation_complete = true; // open interrupt processing for business
  

  Serial.begin(9600);
  
}

//called repeatedly 
void loop() {
  //set output ledPin to state of wheels
  digitalWrite(ledPin, state);

  // test manual killswitch and process if pressed
  if (read_button() == switched) {
    // button on/off cycle now complete, so flip LED between HIGH and LOW
    led_status = HIGH - led_status; // toggle state
    digitalWrite(LED, led_status);
    Serial.print("dig pin 2: ");
  } 
  
  //check if wheels are off
  if(state == LOW || led_status == LOW){
    motorGo(MOTOR_1, BRAKE, 0);
    motorGo(MOTOR_2, BRAKE, 0);
    
    //restart loop call
    return;
  }

  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);
  
  xData = analogRead(X_AXIS_PIN) / 4;
  yData = analogRead(Y_AXIS_PIN) / 4;

  speedMultiplier = getSpeedMultiplier(yData);

  leftSpeed = getLeftSpeed(xData) * speedMultiplier;
  rightSpeed = getRightSpeed(xData) * speedMultiplier;

  // JOYSTICK CENTER
  if(yData < upperLimit && yData > lowerLimit){
    motorGo(MOTOR_1, BRAKE, 0);
    motorGo(MOTOR_2, BRAKE, 0);
  }

  // JOYSTICK FORWARD
  else if(yData > 127 + SENSITIVITY_OFFSET){
    motorGo(MOTOR_1, CCW, leftSpeed);
    motorGo(MOTOR_2, CW, rightSpeed);
  }

  // JOYSTICK REVERSE
  else{
    motorGo(MOTOR_1, CW, leftSpeed);
    motorGo(MOTOR_2, CCW, rightSpeed);
  }
}

/*
 * killswitch
 * 
 * Description:
 * - interrupt called on change of Digital Pin 2 or rising edge of Digital Pin 3
 */
void killswitch() {
  state = !state;
  a = digitalRead(2);
  Serial.print("dig pin 2: ");
  Serial.print(a);
  Serial.print("\n\n\n\n\n\n\n\n\n ");

  b = digitalRead(3);
  Serial.print("dig pin 3 (manual killswitch): ");
  Serial.print(b);
  Serial.print("\n\n\n\n\n\n\n\n\n ");
}


void killswitch2() {
  state = !state;
  a = digitalRead(2);
  Serial.print("dig pin 2: ");
  Serial.print(a);
  Serial.print("\n\n\n\n\n\n\n\n\n ");

  b = digitalRead(3);
  Serial.print("dig pin 3 (manual killswitch): ");
  Serial.print(b);
  Serial.print("\n\n\n\n\n\n\n\n\n ");
}

/*
 * getSpeedMultiplier
 * 
 * Description:
 * - This function uses the y-axis data from the joystick
 * to output a percentage of the maximum speed the motors will output
 * - What this means for the user is that the motor speed will increase in
 * a particular direction the further from the deadzone the joystick is and vice versa
 */
float getSpeedMultiplier(int yValue){
  float multiplier = 1.0;
  
  if(yValue > upperLimit){
    multiplier = float((yValue - upperLimit)) / float((255 - upperLimit));
  }
  else if(yValue < lowerLimit){
    multiplier = float((lowerLimit - yValue)) / float(lowerLimit);
  }

  return multiplier;
}

/*
 * getLeftSpeed
 * 
 * Description:
 * - This function takes in the x-axis data and outputs the pwm speed value
 * for the left motor.
 * - What this means for the user is that the further to the left the joystick is,
 * the slower the left motor will operate (and the faster the right motor will operate) 
 * to allow the wheelchair to turn left
 */
int getLeftSpeed(int xValue){
  int motorSpeed = 0;
  
  if(xValue <= 16){
    motorSpeed = MIN_SPEED;
  }
  else if(xValue <= 104){
    motorSpeed = (slope * xValue) + leftIntercept;
  }
  else{
    motorSpeed = MAX_SPEED;
  }

  return motorSpeed;
}

/*
 * getRightSpeed
 * 
 * Description:
 * - This function takes in the x-axis data and outputs the pwm speed value
 * for the right motor.
 * - What this means for the user is that the further to the right the joystick is,
 * the slower the right motor will operate (and the faster the left motor will operate) 
 * to allow the wheelchair to turn right
 */
int getRightSpeed(int xValue){
  int motorSpeed = 0;

  if(xValue <= 152){
    motorSpeed = MAX_SPEED;
  }
  else if(xValue <= 240){
    motorSpeed = (-slope * xValue) + rightIntercept;
  }
  else{
    motorSpeed = MIN_SPEED;
  }

  return motorSpeed;
}

/*
 * motorGo
 * 
 * Description:
 * - This function takes in the motor value, direction value, and pwm speed value 
 * and controls the motors accordingly
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm){
  if(motor == MOTOR_1){
    if(direct == CW){
      digitalWrite(MOTOR_A1_PIN, LOW); 
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if(direct == CCW){
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);      
    }
    else{
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_1, pwm); 
  }
  else if(motor == MOTOR_2){
    if(direct == CW){
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, HIGH);
    }
    else if(direct == CCW){
      digitalWrite(MOTOR_A2_PIN, HIGH);
      digitalWrite(MOTOR_B2_PIN, LOW);      
    }
    else{
      digitalWrite(MOTOR_A2_PIN, LOW);
      digitalWrite(MOTOR_B2_PIN, LOW);            
    }
    
    analogWrite(PWM_MOTOR_2, pwm);
  }
}
