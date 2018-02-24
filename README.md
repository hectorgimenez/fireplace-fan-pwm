# ESP8266 PID Controller
This project is a ESP8266 based PID controller. It uses a temperature sensor DS18B20 as an input value and an analog output pin for PWM PID output value.

## How it works
Reads the input value from the temperature sensor and try to stabilize at the setpoint value with the PWM output. In my case the PWM output is connected to a PWM board for AC devices. It's not the best way to control an AC motor, but it works.