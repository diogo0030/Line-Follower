#include <Arduino.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <VL53L0X.h>

// Encoder Pins
const int ENCODER1_A = 10;
const int ENCODER1_B = 9;
const int ENCODER2_A = 14;
const int ENCODER2_B = 13;

// Configuração dos Sensores de Linha (3 sensores)
#define NUM_SENSORS 3
const uint8_t sensorPins[NUM_SENSORS] = {28, 27, 26};
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// Sensor ToF
VL53L0X tof;
float distance_cm = 0.0;

// Pinos dos Motores
#define AIN1 1  // Motor Esquerdo
#define PWMA 6
#define BIN1 0  // Motor Direito
#define PWMB 7

// Function Prototypes
void setupEncoders();
void setupSensors();
void setupMotors();
void updateEncoder1();
void updateEncoder2();
void handleToFSensor();
void handleLineFollowing();
void balanceMotorSpeeds();
void setMotorSpeed(int motor1Speed, int motor2Speed);

// Encoder variables
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
unsigned long lastSpeedCheck = 0;
const int SPEED_CHECK_INTERVAL = 100;

// PID Constants
const float Kp = 0.4;
const float Kd =2.8;
const int BASE_SPEED = 90;
const int MAX_SPEED = 180;
const int TURN_SPEED = 60;

// PID Variables
float lastError = 0;
float error = 0;

void setup() {
    Serial.begin(115200);
    setupEncoders();
    setupSensors();
    setupMotors();
    
    // Calibração dos sensores de linha
    for (uint16_t i = 0; i < 400; i++) {
        qtr.calibrate();
        delay(10);
    }
}

void setupEncoders() {
    pinMode(ENCODER1_A, INPUT_PULLUP);
    pinMode(ENCODER1_B, INPUT_PULLUP);
    pinMode(ENCODER2_A, INPUT_PULLUP);
    pinMode(ENCODER2_B, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(ENCODER1_A), updateEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_A), updateEncoder2, RISING);
}

void setupSensors() {
    qtr.setTypeAnalog();
    qtr.setSensorPins(sensorPins, NUM_SENSORS);
    
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();
    
    tof.setTimeout(500);
    if (tof.init()) {
        tof.startContinuous();
    }
}

void setupMotors() {
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
}

void loop() {
    handleToFSensor();
    handleLineFollowing();
    balanceMotorSpeeds();
    delay(20);
}

void updateEncoder1() {
    encoder1Count += (digitalRead(ENCODER1_B) == HIGH) ? 1 : -1;
}

void updateEncoder2() {
    encoder2Count += (digitalRead(ENCODER2_B) == HIGH) ? 1 : -1;
}

void handleToFSensor() {
    distance_cm = tof.readRangeContinuousMillimeters() / 10.0;
    if (!tof.timeoutOccurred() && distance_cm < 15.0) {
        setMotorSpeed(0, 0); // Parada imediata ao detectar obstáculo
    }
}

void handleLineFollowing() {
    qtr.readCalibrated(sensorValues);
    
    // Ajustando para os valores corretos (branco ~1000, preto ~300)
    if (sensorValues[1] < 500) {  // Sensor central na linha preta
        error = 0;
    } else if (sensorValues[0] < 500) {  // Sensor esquerdo detecta linha
        error = -4;
    } else if (sensorValues[2] < 500) {  // Sensor direito detecta linha
        error = 4;
    }
    
    float derivative = error - lastError;
    int correction = (Kp * error * BASE_SPEED) + (Kd * derivative);
    lastError = error;
    
    setMotorSpeed(BASE_SPEED - correction, BASE_SPEED + correction);
}

void balanceMotorSpeeds() {
    if (millis() - lastSpeedCheck >= SPEED_CHECK_INTERVAL) {
        encoder1Count = 0;
        encoder2Count = 0;
        lastSpeedCheck = millis();
    }
}

void setMotorSpeed(int motor1Speed, int motor2Speed) {
    motor1Speed = constrain(motor1Speed, -MAX_SPEED, MAX_SPEED);
    motor2Speed = constrain(motor2Speed, -MAX_SPEED, MAX_SPEED);
    
    digitalWrite(AIN1, motor1Speed >= 0 ? HIGH : LOW);
    analogWrite(PWMA, abs(motor1Speed));
    
    digitalWrite(BIN1, motor2Speed >= 0 ? HIGH : LOW);
    analogWrite(PWMB, abs(motor2Speed));
}
