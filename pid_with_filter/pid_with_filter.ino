// Pin definitions
const int motorPin1 = 9; 
const int encoderPin = 2; 

// PID constants
float Kp = 2.0;  
float Ki = 0.5; 
float Kd = 1.0; 


float setpoint = 100; 
float currentSpeed = 0; 
float error = 0, previousError = 0;
float integral = 0, derivative = 0;
float output = 0;
int motorSpeed = 0;

// Soft start filter variables (Exponential Smoothing)
float smoothedSpeed = 0;  
float alpha = 0.2;  

// Encoder variables
volatile long encoderCount = 0; 
unsigned long previousMillis = 0;
const long interval = 100;  

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countEncoder, RISING);
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();

  // Calculate speed every interval
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Measure the current motor speed using the encoder
    currentSpeed = encoderCount * (1000.0 / interval);  // Speed in counts per second
    encoderCount = 0;  // Reset encoder count for the next interval

    // Soft start using exponential smoothing
    smoothedSpeed = alpha * setpoint + (1 - alpha) * smoothedSpeed;

    // PID control
    error = smoothedSpeed - currentSpeed;
    integral += error * (interval / 1000.0);
    derivative = (error - previousError) / (interval / 1000.0);
    
    output = Kp * error + Ki * integral + Kd * derivative;
    motorSpeed = constrain(output, 0, 255);  // Ensure motorSpeed stays between 0 and 255 (PWM range)

    // Send the computed motor speed to the motor
    analogWrite(motorPin1, motorSpeed);
    
    previousError = error;  // Save the current error for the next iteration

    // Output for debugging
    Serial.print("Current Speed: ");
    Serial.print(currentSpeed);
    Serial.print("  Smoothed Speed: ");
    Serial.print(smoothedSpeed);
    Serial.print("  Motor Speed: ");
    Serial.println(motorSpeed);
  }
}

// Interrupt service routine (ISR) to count encoder ticks
void countEncoder() {
  encoderCount++;  // Increment the encoder count every time the interrupt is triggered
}
