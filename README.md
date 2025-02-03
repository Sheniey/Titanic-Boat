
# TITANIC
> **By:** [@Sheñey](https://github.com/Sheniey)
>
> **License:** None
>
> **Date:** 02/02/2025

---

[Scheme](#scheme "Goto the Scheme...") | [**Code**](#code "Goto the Arduino Code...")

---

<br>

## Scheme:
---
```txt
│ ┃ ┄ ┅ ┆ ┇ ┈ ┉ ┊ ┋
─ ━ ━ ┄ ┅ ┆ ┇ ┈ ┉ ┊ ┋
└ ┗ ┘ ┛ ┌ ┏ ┐ ┓ ├ ┝ ┞ ┟ ┠ ┡ ┢ ┣
┤ ┥ ┦ ┧ ┨ ┩ ┪ ┫ ┬ ┭ ┮ ┯ ┰ ┱ ┲ ┳
┴ ┵ ┶ ┷ ┸ ┹ ┺ ┻ ┼ ┽ ┾ ┿ ╀ ╁ ╂ ╃ ╄ ╅ ╆ ╇ ╈ ╉ ╊ ╋
█ ▓ ▒ ▀ ▄ ≡ ║ ╬ ⊞ ⌗ ☰ 井

        ││││││
 TX1 ─▓▓┴┴┴┴┴┴▓▓─ VIN
 RX0 ─▓▓▓▓▓▓▓▓▓▓─ GND
 RST ─▓▓▓▓▓▓▓▓▓▓─ RST
 GND ─▓▓▓▓▓▓▓▓▓▓─ 5V
  D2 ─▓▓▓▓▓▓▓▓▓▓─ A7
  D3 ─▓▓▓▓▓▓▓▓▓▓─ A6
  D4 ─▓▓▓▓▓▓▓▓▓▓─ A5
  D5 ─▓▓▓▓▓▓▓▓▓▓─ A4
  D6 ─▓▓▓▓▓▓▓▓▓▓─ A3
  D7 ─▓▓▓▓▓▓▓▓▓▓─ A2
  D8 ─▓▓▓▓▓▓▓▓▓▓─ A1
  D9 ─▓▓▓▓▓▓▓▓▓▓─ A0
 D10 ─▓▓▒▒▒▒▒▒▓▓─ REF
 D11 ─▓▓██████▓▓─ 3V3
 D12 ─▓▓██████▓▓─ D13
        ██████

 Arduino Nano [RST -  3p]: Pin [RESET] del Arduino Nano
 Arduino Nano [GND -  4p]: Pin [GND] del IC L293D y Pin [GND] del Modulo IR RQ-S005
 Arduino Nano [D3  -  6p]: Salida PWM que Reguala la Intensidad de los LEDs
 Arduino Nano [D5  -  8p]: Pin [IN 1] del IC L293D para el Motor 1
 Arduino Nano [D6  -  9p]: Pin [IN 2] del IC L293D para el Motor 1
 Arduino Nano [D7  - 10p]: Pin [IN 3] del IC L293D para el Motor 2
 Arduino Nano [D8  - 11p]: Pin [IN 4] del IC L293D para el Motor 2
 Arduino Nano [D9  - 12p]: Pin [Signal] del Modulo IR RQ-S005
 Arduino Nano [D10 - 13p]: Salida PWM para la Velocidad del Motor 1; Pin [EN 1,2] del IC L293D
 Arduino Nano [D11 - 14p]: Salida PWM para la Velocidad del Motor 2; Pin [EN 3,4] del IC L293D
 Arduino Nano [D12 - 15p]: Botón de Encendido/Apagado
 Arduino Nano [D13 - 16p]: Pin [Anodo] del LED de Estatus/Signal y de los LEDs
 Arduino Nano [A0  - 17p]: Salida Digital para la Terminal [Active] del Relé para el Pin [VCC2] del IC L293D
 Arduino Nano [5V  - 27p]: Pin [VCC1] del IC L293D y Pin [VCC] del Modulo IR RQ-S005
 Arduino Nano [GND - 29p]: Pin [Catodo] de los LEDs, Alimentación Negativa para el Arduino y para el Pin [NC] del Relé
 Arduino Nano [VIN - 30p]: Alimentación del Arduino con 4.4V


        L293D
 VCC1 ─▓▓▓▓▓▓▓─ VCC2
  IN1 ─▓▓▓▓▓▓▓─ IN4
 OUT1 ─▓▓▓▓▓▓▓─ OUT4
  GND ─▓▓▓▓▓▓▓─ GND
  GND ─▓▓▓▓▓▓▓─ GND
 OUT2 ─▓▓▓▓▓▓▓─ OUT3
  IN2 ─▓▓▓▓▓▓▓─ IN3
  EN1 ─▓▓▓▓▓▓▓─ EN2

 IC L293D [VCC1 -  1p]: Alimentación del IC [5V]
 IC L293D [IN1  -  2p]: Entrada de la Terminal 1 del Motor A
 IC L293D [OUT1 -  3p]: Salida de la Terminal 1 del Motor A
 IC L293D [GND  -  4p]: Terminal de Disipación Termica [Ground]
 IC L293D [GND  -  5p]: Terminal de Disipación Termica [Ground]
 IC L293D [OUT2 -  6p]: Salida de la Terminal 2 del Motor A
 IC L293D [IN2  -  7p]: Entrada de la Terminal 2 del Motor A
 IC L293D [EN1  -  8p]: Habilitador/Velocidad del Motor A
 IC L293D [EN2  -  9p]: Habilitador/Velocidad del Motor B 
 IC L293D [IN3  - 10p]: Entrada de la Terminal 1 del Motor B
 IC L293D [OUT3 - 11p]: Salida de la Terminal 1 del Motor B
 IC L293D [GND  - 12p]: Terminal de Disipación Termica [Ground]
 IC L293D [GND  - 13p]: Terminal de Disipación Termica [Ground]
 IC L293D [OUT4 - 14p]: Salida de la Terminal 2 del Motor B
 IC L293D [IN4  - 15p]: Entrada de la Terminal 2 del Motor B
 IC L293D [VCC2 - 16p]: Alimentación para los Motores [5V - 36V]
```
---
<br>

## Code:
---
```cpp
#include <IRemote.h>
#include <avr/sleep.h>

// Initialize IR Module RQ-S005:
IRrecv irrecv(IRMODULE);
decode_results res;

// Dictionaries:
static const int PINS[10] = { 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 }; // Active Pins
#define PIN_LEDS 1 // TODO: Place a ~330 Ohms Resistor in the Pin
#define PIN_MOTOR1_IN1 2
#define PIN_MOTOR1_IN2 3
#define PIN_MOTOR2_IN1 4
#define PIN_MOTOR2_IN2 5
#define PIN_L293D_SIGNAL 6
#define PIN_MOTOR1_EN 7
#define PIN_MOTOR2_EN 8
#define PIN_PWRBTN 9 // Power Button
#define PIN_STATUS_LED 10
#define PIN_MOTOR_VCC2 11

static const int MARCHS[7] = { 0, 43, 86, 129, 172, 215, 255 }; // Speed => PWM Wave
#define Z 0
#define M1 1
#define M2 2
#define M3 3
#define M4 4
#define M5 5
#define M6 6
#define R 4

static const int LED_MODES[10] = { 0, 614, 677, 715, 761, 818, 880, 921, 957, 1023 }; // LED Intensity

class Titanic
{
    private: 
        int POWER_PIN, MOTOR1[3], MOTOR2[3];
        static const unsigned int ROTATION_CONST = 3;
        static unsigned int __DELAY = 200;
        static unsigned int __ANOTHER_DELAY = __DELAY * 5;

        void setupEngines(int en1, int en2, bool in1, bool in2) {
            analogWrite(MOTOR1[0], en1); analogWrite(MOTOR2[0], en2); // Setup the Speed to Motors
            digitalWrite(MOTOR1[1], in1); digitalWrite(MOTOR1[2], !in1); // Motor 1
            digitalWrite(MOTOR2[1], in2); digitalWrite(MOTOR2[2], !in2); // Motor 2
        }

    public:
        static const int __DEFAULT_SPEED = MARCHS[M3];
        int statusLED, defaultSpeed;
        
        Titanic(
                int powerPin,
                int pinsMotor1[3],
                int pinsMotor2[3],
                int statusLEDPin = 13,
                int defaultMotorSpeed = __DEFAULT_SPEED
            )
        {
            POWER_PIN = powerPin;
            MOTOR1 = pinsMotor1; MOTOR2 = pinsMotor2;
            statusLED = statusLEDPin;
            defaultSpeed = defaultMotorSpeed;
            pinMode(POWER_PIN, OUTPUT);
            for (int i = 0; i < 3; i++) {
                pinMode(MOTOR1[i], OUTPUT); pinMode(MOTOR2[i], OUTPUT);
            }
            pinMode(statusLED, OUTPUT);
        }

        void turnON() {
            digitalWrite(POWER_PIN, 1);
        }

        void turnOFF() {
            digitalWrite(POWER_PIN, 0);
        }

        void initStatusLED() {
            digitalWrite(statusLED, 1);
            millis(__ANOTHER_DELAY);
            digitalWrite(statusLED, 0);
            millis(__ANOTHER_DELAY);
        }

        void writeErrorStatusLED() {
            __DELAY /= 10;
            __ANOTHER_DELAY = __DELAY * 5;
        }

        void testStatusLED(int dly = __DELAY) {
            digitalWrite(statusLED, 1);
            millis(dly);
            digitalWrite(statusLED, 0);
        }

        void suspendMotors() {
            for (int i = 0; i < 3; i++) {
                pinMode(MOTOR1[i], LOW); pinMode(MOTOR2[i], LOW);
            }
        }

        void activateMotors() {
            for (int i = 0; i < 3; i++) {
                pinMode(MOTOR1[i], OUTPUT); pinMode(MOTOR2[i], OUTPUT);
            }
        }

        void stop() {
            analogWrite(MOTOR1[0], 0); analogWrite(MOTOR2[0], 0);
            digitalWrite(MOTOR1[1], 0); digitalWrite(MOTOR1[2], 0);
            digitalWrite(MOTOR2[1], 0); digitalWrite(MOTOR2[2], 0);
        }

        void moveForward(int march = defaultSpeed) {
            setupEngines(MARCHS[march], MARCHS[march], 1, 1);
        }

        void moveBackward(int march = defaultSpeed) {
            setupEngines(MARCHS[march], MARCHS[march], 0, 0);
        }

        void moveLeft(int march = defaultSpeed) {
            setupEngines(MARCHS[march] / ROTATION_CONST, MARCHS[march], 1, 1);
        }

        void moveRight(int march = defaultSpeed) {
            setupEngines(MARCHS[march], MARCHS[march] / ROTATION_CONST, 1, 1);
        }

        void turnLeft(int march = defaultSpeed) {
            setupEngines(MARCHS[march], MARCHS[march], 0, 1);
        }

        void turnRight(int march = defaultSpeed) {
            setupEngines(MARCHS[march], MARCHS[march], 1, 0);
        }
}

const int motor1[3] = { PINS[PIN_MOTOR1_EN], PINS[PIN_MOTOR1_IN1], PINS[PIN_MOTOR1_IN2] }; // Motor 1 Pins
const int motor2[3] = { PINS[PIN_MOTOR2_EN], PINS[PIN_MOTOR2_IN1], PINS[PIN_MOTOR2_IN2] }; // Motor 2 Pins
Titanic ship(PINS[PIN_MOTOR_VCC2], motor1, motor2);

// Default Global Variables:
bool on = false;
bool isNumber = false;
unsigned int speed = M3;
char lastMove[3] = 'sp';
unsigned int ledIntensity = 0;

void sleepMode() {
    // Suspend the CPU
    sleep_enable();
    sleep_cpu();
    sleep_disable();

    ship.turnOFF();
    ship.suspendMotors();
    for (int pin = 0; pin < 10; pin++) {
        pinMode(PINS[pin], LOW);
    }
    pinMode(PINS[PIN_STATUS_LED], OUTPUT); pinMode(PINS[PIN_PWRBTN], INPUT);
}

void activeMode() {
    ship.turnON();
    ship.activateMotors();
    for (int pin = 0; pin < 10; pin++) {
        pinMode(PINS[pin], OUTPUT);
    }
    pinMode(PINS[PIN_L293D_SIGNAL], INPUT); pinMode(PINS[PIN_PWRBTN], INPUT);
}

void setup() {
    // Startup Pin Modes:
    pinMode(PINS[PIN_LEDS], OUTPUT); pinMode(PINS[PIN_STATUS_LED], OUTPUT); pinMode(PINS[PIN_MOTOR_POWER], OUTPUT);
    pinMode(PINS[PIN_L293D_SIGNAL], INPUT); pinMode(PINS[PIN_PWRBTN], INPUT);
    ship.turnON();

    irrecv.enableIRIn(); // Enable: IR Receptor

    // Disabling Communications:
    ADCSRA &= ~(1 << ADEN); // ADC
    TWCR &= ~(1 << TWEN); // I2C
    SPCR &= ~(1 << SPE); // SPI

    Serial.begin(9600);
    Serial.println("""
[*] The Ship's Lunch...
    - Default Speed:""" + ship.defaultSpeed + """PWM;
    * Status LED: true;
    * Test Motor 1: true;
    * Test Motor 2: true;
    - Ok!!
""");
    Serial.end();
    // Serial Disable [UART]
    UCSR0B &= ~(1 << RXEN0); // Receiver UART
    UCSR0B &= ~(1 << TXEN0); // Transmitter UART
}

void main(int response) {
    switch (response) {
        case 0x31: // LG Button: [1]
            speed = M1;
            isNumber = true;
            break;
        case 0x32: // LG Button: [2]
            speed = M2;
            isNumber = true;
            break;
        case 0x33: // LG Button: [3]
            speed = M3;
            isNumber = true;
            break;
        case 0x34: // LG Button: [4]
            speed = M4;
            isNumber = true;
            break;
        case 0x35: // LG Button: [5]
            speed = M5;
            isNumber = true;
            break;
        case 0x36: // LG Button: [6]
            speed = M6;
            isNumber = true;
            break;
        case 0x38: // LG Button: [8]
            digitalRead(PIN_MOTOR_POWER) ? ship.turnON() : ship.turnOFF(); // Turn <ON/OFF> the Motors
            break;
        case 0x30: // LG Button: [0]
            speed = Z;
            break;
        case 0xE0A2: // LG Button: [Mute]
            ship.stop();
            break;
        case 0xE09C: // LG Button: [UpCh+]
            speed += speed >= 6 ? 0 : 1;
            break;
        case 0xE09D: // LG Button: [DwCh-]
            speed -= speed <= 0 ? 0 : 1;
            break;
        case 0xE0A8: // LG Button: [UpVol+]
            ledIntensity += ledIntensity >= 10 ? 0 : 1;
            break;
        case 0xE0A9: // LG Button: [DwVol-]
            ledIntensity -= ledIntensity <= 0 ? 0 : 1;
            break;
        case 0xE09E: // LG Button: [UpArrow]
            ship.moveForward(speed);
            lastMove = 'mu';
            break;
        case 0xE09F: // LG Button: [LeftArrow]
            ship.moveLeftward(speed);
            lastMove = 'ml';
            break;
        case 0xE0A0: // LG Button: [DownArrow]
            ship.moveBack(speed);
            lastMove = 'md';
            break;
        case 0xE0A1: // LG Button: [RightArrow]
            ship.moveRight(speed);
            lastMove = 'mr';
            break;
        case 0xE0A3: // LG Button: [Back]
            ship.turnLeft(speed);
            lastMove = 'tl';
            break;
        case 0xE0A5: // LG Button: [I Don't Know but I think It's the one Below the RightArrow Button]
            ship.turnLeft(speed);
            lastMove = 'tr';
            break;
        case 0xE0A4: // LG Button: [Info]
            ship.testStatusLED();
            break;
        default:
            break;
        }

        if (isNumber) {
            switch (lastMove) {
                case 'sp':
                    speed = 0;
                    break;
                case 'mu':
                    main(0xE09E);
                    break;
                case 'ml':
                    main(0xE09F);
                    break;
                case 'md':
                    main(0xE0A0);
                    break;
                case 'mr':
                    main(0xE0A1);
                    break;
                case 'tl':
                    main(0xE0A2);
                    break;
                case 'tr':
                    main(0xE0A3);
                    break;
                default:
                    ship.writeErrorStatusLED();
                    break;
            }
            isNumber = false;
        }
}

void loop() {
    ship.initStatusLED();
    analogWrite(PINS[PIN_LEDS], LED_MODES[ledIntensity]); // LED Pins

    on = digitalRead(PINS[PIN_PWRBTN]);

    if (irrecv.decode(&res)) { // Decode to HEX Value
        if (on) {
            main(res.value);
        } else {
            powerButtonPressed = res.value == 0xC4 // LG: Button: [Power]
            on = powerButtonPressed ? !on : on;
            if (!on) {
                analogWrite(PINS[PIN_LEDS], 0); // LED Pins
                sleepMode();
            } else {
                activeMode();
            }
        }
        irrecv.resume();
    }
}

/* "lastMoveCode" Be Like:
*    __SP: Stop
*    __M<adr>: Move to <Address>
*    __T<adr>: Turn to <Address>
*/
```
---
