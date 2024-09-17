// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Ricardo Quesada
// http://retro.moe/unijoysticle2

#include "sdkconfig.h"
#include <Arduino.h>
#include <Bluepad32.h>
#include <stdio.h>
#include "driver/rmt.h"
#include "esp_err.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "driver/ledc.h"

#define POWER_ON_PIN 26
#define GPIO_FAULT_1 36
#define GPIO_FAULT_2 34
#define DEAD_BAND 50

#define LED_PIN 2
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define T0H 14
#define T0L 52
#define T1H 52
#define T1L 14
#define BITS_PER_LED_CMD 24

#define BATTERY_PIN ADC1_CHANNEL_7
#define DEFAULT_VREF 1100
static esp_adc_cal_characteristics_t* adc_chars;

#define SERVO_MIN_PULSEWIDTH 500
#define SERVO_MAX_PULSEWIDTH 2400
#define LEDC_FREQUENCY 50
#define LEDC_RESOLUTION 16

enum Buttons {
    BTN_B = 1,
    BTN_A = 2,
    BTN_Y = 4,
    BTN_X = 8,
    BUTTON_L1 = 16,
    BUTTON_R1 = 32,
    BUTTON_L2 = 64,
    BUTTON_R2 = 128,
    BUTTON_STICK_L = 256,
    BUTTON_STICK_R = 512
};

enum class DPad : unsigned long {
    UP = 1 << 0,
    DOWN = 1 << 1,
    RIGHT = 1 << 2,
    LEFT = 1 << 3
};

enum class MiscButtons : unsigned long {
    HOME = 1 << 0,
    MINUS = 1 << 1,
    PLUS = 1 << 2,
    DOT = 1 << 3
};

typedef struct {
    int pin1;
    int pin2;
    int channel_forward;
    int channel_reverse;
} Motor;

typedef struct {
    int pin;
    int channel;
    int angle;
} Servo;

Motor motors[4] = {
    {16, 25, 0, 1},  // Motor 1
    {32, 27, 2, 3},  // Motor 2
    {4, 12, 4, 5},   // Motor 3
    {15, 14, 6, 7}   // Motor 4
};

Servo servos[3] = {
    {13, 8, 90},  // Servo 1
    {33, 9, 90},  // Servo 2
    {17, 10, 90}  // Servo 3
};

int JoyLx = -1;
int JoyLy = -1;
int JoyRx = -1;
int JoyRy = -1;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void initADC() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_PIN, ADC_ATTEN_DB_12);
    adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    if (adc_chars == NULL) {
        ESP_LOGE("ADC", "Failed to allocate memory for ADC characteristics");
        return;
    }
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(
        ADC_UNIT_1,
        ADC_ATTEN_DB_12,
        ADC_WIDTH_BIT_12,
        DEFAULT_VREF,
        adc_chars
    );
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI("ADC", "ADC calibration using e-fuse Vref");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI("ADC", "ADC calibration using Two Point");
    } else {
        ESP_LOGI("ADC", "ADC calibration using default Vref");
    }
}

float readBatteryVoltage() {
    uint32_t adc_reading = adc1_get_raw(BATTERY_PIN);
    uint32_t voltage_at_pin_mV = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    float battery_voltage = (float)(voltage_at_pin_mV * 2) / 1000.0;
    return battery_voltage;
}

void setServoAngle(Servo* servo, int angle) {
    angle = constrain(angle, 0, 180);
    int pulseWidth = map(angle, 0, 180, SERVO_MIN_PULSEWIDTH, SERVO_MAX_PULSEWIDTH);
    int dutyCycle = (pulseWidth * ((1 << LEDC_RESOLUTION) - 1)) / 20000;
    ledcWrite(servo->channel, dutyCycle);
    servo->angle = angle;
}

void initServos() {
    for (int i = 0; i < 3; i++) {
        ledcSetup(servos[i].channel, LEDC_FREQUENCY, LEDC_RESOLUTION);
        ledcAttachPin(servos[i].pin, servos[i].channel);
    }
}

void initMotors() {
    for (int i = 0; i < 4; i++) {
        pinMode(motors[i].pin1, OUTPUT);
        pinMode(motors[i].pin2, OUTPUT);
        ledcSetup(motors[i].channel_forward, 5000, 8);
        ledcSetup(motors[i].channel_reverse, 5000, 8);
        ledcAttachPin(motors[i].pin1, motors[i].channel_forward);
        ledcAttachPin(motors[i].pin2, motors[i].channel_reverse);
    }
}

void initGPIOFaultCheck() {
    pinMode(GPIO_FAULT_1, INPUT);
    pinMode(GPIO_FAULT_2, INPUT);
}

void checkMotorDriverFault() {
    int fault1 = digitalRead(GPIO_FAULT_1);
    int fault2 = digitalRead(GPIO_FAULT_2);
    if (fault1 == LOW) {
        Console.println("Motor1 Driver Fault");
    } else if (fault2 == LOW) {
        Console.println("Motor2 Driver Fault");
    } else {
        Console.println("OK");
    }
}

void setPixelColor(rmt_item32_t* items, uint8_t red, uint8_t green, uint8_t blue) {
    uint32_t led_data = (green << 16) | (red << 8) | blue;
    for (int bit = 0; bit < BITS_PER_LED_CMD; bit++) {
        if (led_data & (1 << (23 - bit))) {
            items[bit].duration0 = T1H;
            items[bit].level0 = 1;
            items[bit].duration1 = T1L;
            items[bit].level1 = 0;
        } else {
            items[bit].duration0 = T0H;
            items[bit].level0 = 1;
            items[bit].duration1 = T0L;
            items[bit].level1 = 0;
        }
    }
}

void writePixelColor(uint8_t red, uint8_t green, uint8_t blue) {
    rmt_item32_t items[BITS_PER_LED_CMD];
    setPixelColor(items, red, green, blue);
    rmt_write_items(RMT_TX_CHANNEL, items, BITS_PER_LED_CMD, true);
    rmt_wait_tx_done(RMT_TX_CHANNEL, pdMS_TO_TICKS(100));
}

void initWS2812() {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX((gpio_num_t)LED_PIN, RMT_TX_CHANNEL);
    config.clk_div = 2;
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
    writePixelColor(200, 0, 0);
}

int normalizeJoystickInput(int axisValue) {
    if (abs(axisValue) < DEAD_BAND) {
        return 0;
    }
    return map(axisValue, -512, 512, -255, 255);
}

int calculatePWM(int axisX, int axisY) {
    int pwmValue = axisY + axisX;
    return constrain(pwmValue, -255, 255);
}

void controlMotor(Motor* motor, int pwmValue) {
    if (pwmValue >= 0) {
        ledcWrite(motor->channel_forward, pwmValue);
        ledcWrite(motor->channel_reverse, 0);
    } else {
        ledcWrite(motor->channel_forward, 0);
        ledcWrite(motor->channel_reverse, -pwmValue);
    }
}

void handleMotorControl(int axisX, int axisY, Motor* leftMotor, Motor* rightMotor) {
    int normX = normalizeJoystickInput(axisX);
    int normY = normalizeJoystickInput(axisY);
    int leftPWM = calculatePWM(normX, normY);
    int rightPWM = calculatePWM(normX, -normY);
    controlMotor(leftMotor, leftPWM);
    controlMotor(rightMotor, rightPWM);
}

void processButtons(ControllerPtr ctl) {
    unsigned long buttonState = ctl->buttons();
    if (buttonState) {
        if (buttonState & BUTTON_B) {
            Console.println("Button B pressed");
        }
        if (buttonState & BTN_A) {
            Console.println("Button A pressed");
        }
        if (buttonState & BTN_Y) {
            Console.println("Button Y pressed");
        }
        if (buttonState & BTN_X) {
            Console.println("Button X pressed");
        }
        if (buttonState & BUTTON_L1) {
            Console.println("Button L1 pressed");
        }
        if (buttonState & BUTTON_R1) {
            Console.println("Button R1 pressed");
        }
        if (buttonState & BUTTON_L2) {
            Console.println("Button L2 pressed");
        }
        if (buttonState & BUTTON_R2) {
            Console.println("Button R2 pressed");
        }
        if (buttonState & BUTTON_STICK_L) {
            Console.println("Stick Left pressed");
        }
        if (buttonState & BUTTON_STICK_R) {
            Console.println("Stick Right pressed");
        }
    }
    unsigned long dpadState = ctl->dpad();
    if (dpadState) {
        if (dpadState & static_cast<unsigned long>(DPad::UP)) {
            Console.println("D-pad Up pressed");
            setServoAngle(&servos[0], servos[0].angle + 10);
            Console.println(servos[0].angle);
        }
        if (dpadState & static_cast<unsigned long>(DPad::DOWN)) {
            Console.println("D-pad Down pressed");
            setServoAngle(&servos[0], servos[0].angle - 10);
            Console.println(servos[0].angle);
        }
        if (dpadState & static_cast<unsigned long>(DPad::RIGHT)) {
            Console.println("D-pad Right pressed");
            setServoAngle(&servos[1], servos[1].angle + 10);
            Console.println(servos[1].angle);
        }
        if (dpadState & static_cast<unsigned long>(DPad::LEFT)) {
            Console.println("D-pad Left pressed");
            setServoAngle(&servos[1], servos[1].angle - 10);
            Console.println(servos[1].angle);
        }
    }
    unsigned long miscState = ctl->miscButtons();
    if (miscState) {
        if (miscState & static_cast<unsigned long>(MiscButtons::HOME)) {
            Console.println("Home button pressed");
        }
        if (miscState & static_cast<unsigned long>(MiscButtons::MINUS)) {
            Console.println("Minus button pressed");
        }
        if (miscState & static_cast<unsigned long>(MiscButtons::PLUS)) {
            Console.println("Plus button pressed");
        }
        if (miscState & static_cast<unsigned long>(MiscButtons::DOT)) {
            Console.println("Dot button pressed");
        }
    }
}

void processGamepad(ControllerPtr ctl) {
    if (JoyLx == -1) {
        JoyLx = ctl->axisX();
    }
    if (JoyLy == -1) {
        JoyLy = ctl->axisY();
    }
    if (JoyRx == -1) {
        JoyRx = ctl->axisRX();
    }
    if (JoyRy == -1) {
        JoyRy = ctl->axisRY();
    }
    handleMotorControl(
        ctl->axisRX() - JoyRx,
        ctl->axisRY() - JoyRy,
        &motors[0],  // Left motor (Motor 1)
        &motors[1]   // Right motor (Motor 2)
    );
    handleMotorControl(
        ctl->axisX() - JoyLx,
        ctl->axisY() - JoyLy,
        &motors[2],  // Left motor (Motor 3)
        &motors[3]   // Right motor (Motor 4)
    );
    processButtons(ctl);
    float voltage = readBatteryVoltage();
    if (voltage > 4) {
        ctl->setPlayerLEDs(13 & 0x0f);
    } else if (voltage > 3.8) {
        ctl->setPlayerLEDs(7 & 0x0f);
    } else if (voltage > 3.5) {
        ctl->setPlayerLEDs(0 & 0x0f);
    } else {
        ctl->setPlayerLEDs(0 & 0x0f);
    }
}

void processMouse(ControllerPtr ctl) {
    if (ctl->scrollWheel() > 0) {
        // Do Something
    } else if (ctl->scrollWheel() < 0) {
        // Do something else
    }
}

void processKeyboard(ControllerPtr ctl) {
    if (!ctl->isAnyKeyPressed())
        return;
    if (ctl->isKeyPressed(Keyboard_A)) {
        Console.println("Key 'A' pressed");
    }
    if (ctl->isKeyPressed(Keyboard_LeftShift)) {
        Console.println("Key 'LEFT SHIFT' pressed");
    }
    if (ctl->isKeyPressed(Keyboard_LeftArrow)) {
        Console.println("Key 'Left Arrow' pressed");
    }
}

void processBalanceBoard(ControllerPtr ctl) {
    if (ctl->topLeft() > 10000) {
        // Do Something
    }
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else if (myController->isMouse()) {
                processMouse(myController);
            } else if (myController->isKeyboard()) {
                processKeyboard(myController);
            } else if (myController->isBalanceBoard()) {
                processBalanceBoard(myController);
            } else {
                Console.printf("Unsupported controller\n");
            }
        }
    }
}

void onConnectedController(ControllerPtr ctl) {
    writePixelColor(0, 180, 0);
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Console.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Console.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName(), properties.vendor_id,
                           properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Controller connected, but could not find empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    writePixelColor(200, 0, 0);
    bool foundController = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Console.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }
    if (!foundController) {
        Console.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void setup() {
    pinMode(POWER_ON_PIN, OUTPUT);
    digitalWrite(POWER_ON_PIN, HIGH);
    initMotors();
    initServos();
    initGPIOFaultCheck();
    initWS2812();
    initADC();
    Console.println("Pins Initialized");
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
    BP32.enableBLEService(false);
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    vTaskDelay(150);
}
