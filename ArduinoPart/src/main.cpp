// Để chạy, thay #define TIMER_ENABLED true đổi thành false trong NewPing.h

#define _TASK_SLEEP_ON_IDLE_RUN

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <ServoTimer2.h>
#include <SoftwareSerial.h>
#include <TaskScheduler.h>
#include <toneAC.h>
#include <NewPing.h>

static constexpr float SensorDistanceThreshold = 9;

static constexpr uint8_t BuzzerPin = 9;
static constexpr uint8_t LedPin = 8;

static constexpr uint8_t ExitSensorTriggerPin = 3;
static constexpr uint8_t ExitSensorEchoPin = 2;
static constexpr uint8_t EnterSensorTriggerPin = 5;
static constexpr uint8_t EnterSensorEchoPin = 4;
static constexpr uint8_t ServoPin = 7;
static constexpr uint8_t SerialReceivePin = 6;
static constexpr uint8_t SerialTransmitPin = 11;

int patientCount = 0;
bool isInside = false, isOutside = false;

Scheduler taskScheduler;
LiquidCrystal_I2C lcd(0x27, 16, 2);
ServoTimer2 servo;
SoftwareSerial swSerial(SerialReceivePin, SerialTransmitPin);
NewPing enterSensor(EnterSensorTriggerPin, EnterSensorEchoPin, 20);
NewPing exitSensor(ExitSensorTriggerPin, ExitSensorEchoPin, 20);

void doUltrasonicMeasurements();
void doLcdUpdate();

void doServoOpen();
void doServoClose();

void doLedEnable();
void doLedDisable();

void doBuzzerEnable();
void doBuzzerDisable();

void doDisplayHealthMeasuringMode();
void doFinalizeHealthCalculate();
void doRevertToPatientCountMode();

Task taskMeasureDistance(200 * TASK_MILLISECOND, TASK_FOREVER, &doUltrasonicMeasurements, &taskScheduler, true);
Task taskUpdateLcd(0, TASK_ONCE, &doLcdUpdate, &taskScheduler, false);
Task taskOpenServo(0, TASK_ONCE, &doServoOpen, &taskScheduler, false);
Task taskCloseServo(3 * TASK_SECOND, TASK_ONCE, &doServoClose, &taskScheduler, false);
Task taskEnableLed(0, TASK_ONCE, &doLedEnable, &taskScheduler, false);
Task taskDisableLed(3 * TASK_SECOND, TASK_ONCE, &doLedDisable, &taskScheduler, false);
Task taskEnableBuzzer(0, TASK_ONCE, &doBuzzerEnable, &taskScheduler, false);
Task taskDisableBuzzer(500 * TASK_MILLISECOND, TASK_ONCE, &doBuzzerDisable, &taskScheduler, false);
Task taskDisplayHealthMeasuringMode(0, TASK_ONCE, &doDisplayHealthMeasuringMode, &taskScheduler, false);
Task taskFinalizeHealthCalculate(5 * TASK_SECOND, TASK_ONCE, &doFinalizeHealthCalculate, &taskScheduler, false);
Task taskRevertToPatientCountMode(5 * TASK_SECOND, TASK_ONCE, &doRevertToPatientCountMode, &taskScheduler, false);

bool measuringHealthMode = false;
float heartRateSamples[8];
uint8_t spO2 = 0;

void setup() {
    Serial.begin(9600);
    swSerial.begin(9600);

    Serial.println("Initializing LCD...");

    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.cursor_off();
    lcd.setCursor(0, 0);
    lcd.print("So benh nhan:");
    lcd.setCursor(2, 1);
    lcd.print("0");

    pinMode(ExitSensorTriggerPin, OUTPUT);
    pinMode(ExitSensorEchoPin, INPUT);
    pinMode(EnterSensorTriggerPin, OUTPUT);
    pinMode(EnterSensorEchoPin, INPUT);
    pinMode(BuzzerPin, OUTPUT);
    pinMode(LedPin, OUTPUT);
    // pinMode(WarningLedPin, OUTPUT);

    pinMode(SerialReceivePin, INPUT);
    pinMode(SerialTransmitPin, OUTPUT);

    servo.attach(ServoPin);
    servo.write(0);

    // Disable the built-in LED on the Arduino UNO R3
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    digitalWrite(LedPin, LOW);

    pinMode(SerialTransmitPin, OUTPUT);
    pinMode(SerialReceivePin, INPUT);
}

void getSensorsDistances(float* enter, float* exit) {
    *enter = static_cast<float>(enterSensor.ping_cm());
    *exit = static_cast<float>(exitSensor.ping_cm());
}

void doUltrasonicMeasurements() {
    float enterDistance = 0, exitDistance = 0;

    getSensorsDistances(&enterDistance, &exitDistance);

    if (enterDistance <= SensorDistanceThreshold) {
        if (!isInside) {
            patientCount++;
            isInside = true;
            taskUpdateLcd.restart();
            taskOpenServo.restart();
            taskEnableLed.restart();
            taskEnableBuzzer.restart();
        }
    } else {
        if (isInside) {
            isInside = false;
        }
    }

    if (exitDistance <= SensorDistanceThreshold) {
        if (!isOutside) {
            patientCount = max(patientCount - 1, 0);
            isOutside = true;
            taskUpdateLcd.restart();
            taskOpenServo.restart();
        }
    } else {
        if (isOutside) {
            isOutside = false;
        }
    }
}

void doLcdUpdate() {
    Serial.println("doLcdUpdate");
    if (measuringHealthMode) return;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("So benh nhan: ");
    lcd.setCursor(2, 1);
    lcd.print(String(patientCount));
}

void doServoOpen() {
    Serial.println("doServoOpen");

    servo.write(1500);
    taskCloseServo.restartDelayed();
}

void doServoClose() {
    Serial.println("doServoClose");
    servo.write(0);
}

void doLedEnable() {
    Serial.println("doLedEnable");
    digitalWrite(LedPin, HIGH);
    taskDisableLed.restartDelayed();
}

void doLedDisable() {
    Serial.println("doLedDisable");
    digitalWrite(LedPin, LOW);
}

void doBuzzerEnable() {
    Serial.println("doBuzzerEnable");
    toneAC(1000, 10);
    taskDisableBuzzer.restartDelayed();
}

void doBuzzerDisable() {
    Serial.println("doBuzzerDisable");
    noToneAC();
}

void doDisplayHealthMeasuringMode() {
    Serial.println("displayHealthMeasuringMode");
    taskUpdateLcd.disable();
    measuringHealthMode = true;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Nhip tim: ");
    lcd.print(String(heartRateSamples[sizeof(heartRateSamples) / 4 - 1]));
    lcd.setCursor(0, 1);
    lcd.print("SpO2: ");
    lcd.print(String(spO2));
    lcd.print('%');

    taskFinalizeHealthCalculate.restartDelayed();
}

void doFinalizeHealthCalculate() {
    Serial.println("doFinalizeHealthCalculate");

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ket qua: ");

    float sumHeartRate = 0;
    for (const float sample : heartRateSamples) {
        sumHeartRate += sample;
    }

    lcd.setCursor(2, 1);
    lcd.print(String(sumHeartRate / (sizeof(heartRateSamples) / sizeof(heartRateSamples[0]))));
    lcd.print(" / ");
    lcd.print(String(spO2));
    lcd.print('%');

    memset(heartRateSamples, 0, sizeof(heartRateSamples));
    taskRevertToPatientCountMode.restartDelayed();
}

void doRevertToPatientCountMode() {
    Serial.println("doRevertToPatientCountMode");
    measuringHealthMode = false;

    taskUpdateLcd.restart();
}

void loop() {
    taskScheduler.execute();

    if (swSerial.available() > 0) {
        uint8_t buffer[9];

        if (swSerial.readBytes(buffer, 9) == 9 && memcmp(buffer, "HRIF", 4) == 0) {
            const float heartRate = *reinterpret_cast<float*>(buffer + 4);
            spO2 = buffer[8];

            for (int i = 0; i < sizeof(heartRateSamples) / sizeof(float) - 1; i++) {
                heartRateSamples[i] = heartRateSamples[i + 1];
            }
            heartRateSamples[sizeof(heartRateSamples) / sizeof(float) - 1] = heartRate;

            Serial.println("Received health data: " + String(heartRate) + ", " + String(spO2));
            taskDisplayHealthMeasuringMode.restart();
        }
    }
}