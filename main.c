#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// Function Prototypes
void project_speed_control(void);
void move_motor(int dir, int pwmVal);
void stop_motor();
void read_volt_for_speed(int *data);
void project_pos_control(void);
void updateMotorPosition(void);
void buttonISR();
void read_volt_for_pos(int *data);
void calculate_rpm(void);

// Pin Definitions
#define ENCA 8
#define ENCB 3
#define PWM 5
#define IN2 6
#define IN1 7
#define MOTOR_DIR_CW  0
#define MOTOR_DIR_CCW 1
#define BUTTON_PIN1 2
#define BUTTON_PIN2 9
#define COUNTS_PER_REVOLUTION 780
#define btn_pressed LOW  // Adjusted for INPUT_PULLUP logic
#define btn_released HIGH
// Pin connected to the button
#define BUTTON_PRESSED HIGH   // Define the state when the button is pressed (INPUT_PULLUP logic)
#define BUTTON_RELEASED  LOW
#define DEBOUNCE_DELAY 150
// Global Variables
volatile uint8_t btn_flag = 0; // Tracks button state
int voltage_for_speed_val = 0;
const double the_div_nuber = 4.015686275;
int pwm_Val = 0;

volatile unsigned long lastDebounceTime = 0;
volatile unsigned long debounceDelay = 150; // Debounce time in ms

// Position control parameters
float pwmPercentage =0 ;
int voltage_for_pos_val = 0;
volatile int pos = 0;
volatile float kp = 2.0;
volatile float ki = 0.8;
volatile float kd = 0.1;
volatile float error = 0;
volatile float init_error=0;
volatile float curr_pos = 0;
volatile int target_pos = 0;
volatile float gain_p = 0;
volatile float gain_i = 0;
volatile float gain_d = 0;
volatile float gain_T = 0;
volatile int motor_dir = MOTOR_DIR_CW;
volatile float Delta_Time = 0.0;
volatile unsigned long curr_Time = 0;
volatile unsigned long init_Time = 0;
volatile float entegral_term = 0;
volatile float differential_term = 0;
volatile int func = 0; // Declare func properly
volatile int potValue_speed = 0;
volatile int potValue_pos = 0;

volatile unsigned long lastRPMCalcTime = 0; // Last time RPM was calculated
volatile int lastPos = 0; // Last position reading
volatile float rpm = 0; 

unsigned long lastButtonPressTime = 0;
int btn2_state=false;

void setup() {
    // Timer setup for consistent timing
    TCCR1B = (TCCR1B & 0b11111000) | 0x01; // Set timer prescaler

    // Pin setup
    pinMode(BUTTON_PIN1, INPUT_PULLUP); // Enable pull-up resistor for button
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN1), buttonISR, FALLING);
    pinMode(BUTTON_PIN2, INPUT_PULLUP); // Enable pull-up resistor for button
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN2), buttonISR, FALLING); // Button interrupt
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    pinMode(PWM, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // Encoder interrupt
    attachInterrupt(digitalPinToInterrupt(ENCB), updateMotorPosition, CHANGE);

    // Serial communication for debugging
    Serial.begin(9600);
}

void loop() {
    if (btn_flag == 0) {
        project_speed_control();
    } else {
        // Position control
        read_volt_for_pos(&target_pos);
        target_pos = target_pos / 2.8416667; // Scale target position
        curr_pos = pos * (360.0 / COUNTS_PER_REVOLUTION); // Calculate current position
        curr_Time = micros();
        Delta_Time = (curr_Time - init_Time) / 1e6; // Calculate time delta

        if (Delta_Time > 0) {
            // PID Control
            error = target_pos - curr_pos;
            gain_p = kp * error;

            entegral_term += error * Delta_Time; // Accumulate integral term
            entegral_term = constrain(entegral_term, -255, 255); // Anti-windup
            gain_i = ki * entegral_term;

            differential_term = (error - init_error) / Delta_Time; // Derivative term
            gain_d = kd * differential_term;

            func = gain_p + gain_i + gain_d;

            // Determine motor direction
            if (func > 0) {
            motor_dir = MOTOR_DIR_CW;
            } 
            else {
              motor_dir = MOTOR_DIR_CCW;
            }

            // Clamp and apply PWM
            gain_T = fabs(func);
            gain_T = constrain(gain_T, 0, 255);

            if (fabs(error) < 7) { // Stop if within tolerance
                stop_motor();
            } else {
                move_motor(motor_dir, (int)gain_T);
            }

            // Update for next iteration
            init_Time = curr_Time;
            init_error=error;
        }

        // Debug information
        Serial.print("  Target Pos: ");
        Serial.print(target_pos);
        Serial.print("  Current Pos: ");
        Serial.print(curr_pos);
        Serial.print("  Error: ");
        Serial.print(error);
        Serial.print(" PWM Percentage: ");
        Serial.print(gain_T*(0.3921568627));
        Serial.println("%");
    }
}

void move_motor(int dir, int pwmVal) {
     // Calculate PWM as percentage
    analogWrite(PWM, pwmVal); // Set motor speed

    if (dir == MOTOR_DIR_CW) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }

    // Debug print for PWM percentage

}

void stop_motor() {
    analogWrite(PWM, 0); // Stop motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
}

void read_volt_for_speed(int *data) {
    potValue_speed = analogRead(A0);
    *data = potValue_speed; // Return potentiometer value
}

void read_volt_for_pos(int *data) {
    potValue_pos = analogRead(A5);
    *data = potValue_pos; // Return potentiometer value
}

void project_speed_control() {
    read_volt_for_speed(&voltage_for_speed_val); // Read speed control value
    pwm_Val = constrain((int)(voltage_for_speed_val / the_div_nuber), 0, 255);
    btn2_state = isButtonPressed();
    if(btn2_state == BUTTON_PRESSED){
      motor_dir=!motor_dir;
    }
    else{

    }
    move_motor(motor_dir, pwm_Val);
    pwmPercentage = (pwm_Val / 254.0) * 100.0;
    calculate_rpm();
    Serial.print(" RPM: ");
    Serial.print(rpm); // Display current RPM
    Serial.print("  PWM Percentage: ");
    Serial.print(pwmPercentage);
    Serial.println("%"); // Move motor in CW direction
}

void updateMotorPosition() {
    int a = digitalRead(ENCA);
    int b = digitalRead(ENCB);
    if (b == a) {
        pos++;
    } else {
        pos--;
    }
}

void buttonISR() {
    unsigned long currentTime = millis();
    if ((currentTime - lastDebounceTime) > debounceDelay) {
        btn_flag = !btn_flag; // Toggle button flag
        lastDebounceTime = currentTime;
    }
}


void calculate_rpm(void){
  unsigned long currentTime = millis(); // Get current time in milliseconds
  unsigned long dt = currentTime - lastRPMCalcTime;
  if(dt >=1000){
    int deltaPos = pos - lastPos;
    float revolutions = (float)deltaPos / COUNTS_PER_REVOLUTION;
    rpm = (revolutions * 60000) / dt; // Convert to RPM (time in milliseconds)

    lastPos = pos; // Update last position
    lastRPMCalcTime = currentTime; // Update last calculation time
    
  }

}


int isButtonPressed() {
    int buttonState = digitalRead(BUTTON_PIN2);
    unsigned long currentTime = millis();

    // Check if the button is pressed and enough time has passed since the last press
    if (buttonState == BUTTON_PRESSED && (currentTime - lastButtonPressTime) > DEBOUNCE_DELAY) {
        lastButtonPressTime = currentTime; // Update the last button press time
        return BUTTON_PRESSED;
    }
    return BUTTON_RELEASED;
}








