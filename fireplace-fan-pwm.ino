#include <AutoPID.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// General Settings
#define DEBUG true

// Pinout Settings
#define TEMP_READ_DELAY 1000 // OneWire can only read every ~750ms
#define ONE_WIRE_BUS 12 // DS18B20 Pin
#define PWM_OUTPUT_PIN 14 // PWM Output Pin

// PID Controller Settings
#define DESIRED_TEMP 44
#define OUTPUT_MIN 280
#define OUTPUT_MAX 800
#define KP -60
#define KI -0.7
#define KD -0

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temperatureSensors(&oneWire);
unsigned long lastTempUpdate;
double temperature, pwmPower, setPoint;
bool fanStarted = false;

AutoPID myPID(&temperature, &setPoint, &pwmPower, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

bool updateTemperature() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    temperature = temperatureSensors.getTempCByIndex(0);
    lastTempUpdate = millis();
    temperatureSensors.requestTemperatures();
    return true;
  }
  return false;
}

void setup() {
  pinMode(PWM_OUTPUT_PIN, OUTPUT);
  setPoint = DESIRED_TEMP;
  temperatureSensors.begin();
  temperatureSensors.requestTemperatures();
  while (!updateTemperature()) {}

  myPID.setTimeStep(4000);

  Serial.begin(115200);
}

void loop() {
    if (updateTemperature()) {
      if (DEBUG) {
        Serial.print("Temperature: ");
        Serial.println(temperature);
        Serial.print("setPoint: ");
        Serial.println(setPoint);
        Serial.print("Status: ");
        Serial.println(fanStarted);
      }

      if (temperature > DESIRED_TEMP) {
        startFanIfNeeded();
        if (DEBUG) {
          Serial.print("PWM Output: ");
          Serial.println(pwmPower);
        }
        analogWrite(PWM_OUTPUT_PIN, pwmPower);
      }

      // Stop the PWM if temperature is 10ºC below the setpoint value
      if (temperature < DESIRED_TEMP - 10) {
        analogWrite(PWM_OUTPUT_PIN, 0);
        fanStarted = false;
      }
    }

    myPID.run();
}

// Start the PWM at max power during a short period of time, avoiding damages for
// motor overheating. PWM is not the right way for regulate a motor speed. But it's cheap.
void startFanIfNeeded() {
  if (!fanStarted) {
    fanStarted = true;
    analogWrite(PWM_OUTPUT_PIN, OUTPUT_MAX);
    delay(3000);
  }
}
