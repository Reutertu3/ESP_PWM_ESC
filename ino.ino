#include <Arduino.h>
#include "AiEsp32RotaryEncoder.h"

#include <SPI.h>
// For OLED
#include <Wire.h>
#include <U8g2lib.h>

//#define TFT_GREY 0x5AEB // New colour
#define OLED_SDA 6
#define OLED_SCK 7


#define ROTARY_ENCODER_A_PIN 21
#define ROTARY_ENCODER_B_PIN 20
#define ROTARY_ENCODER_BUTTON_PIN 10
#define ROTARY_ENCODER_STEPS 1

#define PWM_CHANNEL 0
#define PWM_PIN 0
#define PWM_FREQ 500 // PWM frequency in Hz
#define MIN_PULSE_WIDTH_US 1000 // Minimum pulse width in microseconds (1ms)
#define MAX_PULSE_WIDTH_US 2000 // Maximum pulse width in microseconds (2ms)
#define PWM_RESOLUTION 8 // PWM resolution (8-bit)


int dutyCycle = 0;
int dutyCyclePercent = 0;
int encoderPosition = 0;
int pulseWidth = MIN_PULSE_WIDTH_US; // Initialize pulse width to minimum

enum State {
  PWM,
  DSHOT
};
State currentState = PWM;

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); 

void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial begin...");

  pinMode(ROTARY_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENCODER_B_PIN, INPUT_PULLUP);
  //rotaryEncoder.areEncoderPinsPulldownforEsp32=false;
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(0, 255, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setAcceleration(12);
  //rotaryEncoder.disableAcceleration();
  
  //Display Stuff
  Wire.begin(6, 7);
  u8g2.begin();
  // Configure PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  startDisplay();
}



void loop() {
  
  updateDisplay();
  encoderPosition = rotaryEncoder.readEncoder();
  //Output rotary encoder
      if (rotaryEncoder.encoderChanged()) {
          Serial.println(encoderPosition);
      }
      if (rotaryEncoder.isEncoderButtonClicked())
      {
          if (currentState == PWM) {
              currentState = DSHOT;
              Serial.println("Current Mode: DSHOT");
          }
          else {
              currentState = PWM;
              Serial.println("Current Mode: PWM");
          }
    }


  dutyCycle = mapToDutyCycle(encoderPosition);
  ledcWrite(PWM_CHANNEL, dutyCycle);

  // Delay for the duration of the pulse
  delayMicroseconds(pulseWidth);

}

int mapToDutyCycle(int input) {
  // Map input value (0-100) to pulse widths (1000-2000 µs)
  int pulseWidthMicroseconds = map(input, 0, 255, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);
    // Calculate duty cycle based on pulse width and PWM frequency
  float dutyCycle = (pulseWidthMicroseconds / 1000.0) * (PWM_FREQ / 1000.0);
  // Return duty cycle as integer (0-255 for 8-bit resolution)
  return int(dutyCycle * 255.0);
}

void startDisplay() {
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_profont17_tr);
    u8g2.drawStr(6, 26, "ESC Testbench");
    u8g2.setFont(u8g2_font_haxrcorp4089_tr);
    //u8g2.drawStr(38, 8, "FH Dortmund");
    u8g2.drawStr(34, 8, "PWM / DSHOT");
    u8g2.sendBuffer();
    delay (3000);
}

void updateDisplay()  {
       //Display stuff
    u8g2.clearBuffer();
    u8g2.setFontMode(1);
    u8g2.setBitmapMode(1);
    u8g2.setFont(u8g2_font_profont12_tr);
    u8g2.drawStr(29, 9, "Duty Cycle %");
    u8g2.setFont(u8g2_font_profont29_tr);
    u8g2.setCursor(50, 32);
    u8g2.print(mapToPercentString(dutyCycle));
    //ModeSwitch
    switch (currentState) {
        case PWM:
                u8g2.setFont(u8g2_font_profont11_tr);
                u8g2.drawStr(2, 32, "PWM");
            break;
        case DSHOT:       
                u8g2.setFont(u8g2_font_profont11_tr);
                u8g2.drawStr(2, 22, "DSHOT");
            break;
    }   
    u8g2.sendBuffer();
}

String mapToPercentString(int value) {
  // Map value (127-255) to percentage (0-100)
  int percentage = map(value, 127, 255, 0, 100);
  
  // Convert percentage to string
  String result = String(percentage);
  
  return result;
}
