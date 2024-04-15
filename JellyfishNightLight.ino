#include "src/RGBConverter/RGBConverter.h"

#include <Servo.h>

#include <CapacitiveSensor.h>

////////// Constants //////////
constexpr int kBluePin = 3;
constexpr int kGreenPin = 5;
constexpr int kRedPin = 6;

constexpr int kMainButtonPin = 2;

constexpr int kCapacitiveSensorThreshold = 900;
constexpr int kSendPin = 11;
constexpr int kSensesPin = 10;

constexpr int kBSendPin = 9;
constexpr int kBSensesPin = 12;

constexpr int kPhotoresistorPin = A0;
constexpr int kMinPhotoresistor = 640;
constexpr int kMaxPhotoresistor = 870;
constexpr int kSmoothingFactor = 14;

constexpr int kServoPin = A5;

constexpr int kPotentiometerPin = A4;

constexpr int kDelayInterval = 6;
constexpr byte kMaxRgbValue = 255;

////////// Global variables //////////
int last_led_val = 0;
int mode = 0;
int color_val = 0;
int brightness_mode = 0;
long bright_val = 255;
float hue = 0.355;
float step = 0.001f;
int button_state;

////////// Objects //////////
Servo myservo;
RGBConverter rgb_converter;
CapacitiveSensor brightness_select = CapacitiveSensor(kSendPin, kSensesPin);
CapacitiveSensor color_select = CapacitiveSensor(kBSendPin, kBSensesPin);

////////// Function declarations //////////

// Blinks the LED in pink to indicate the current mode when first entered.
// - times: The number of times to blink the LED in pink
void BlinkPink(int times);

// Adjusts the brightness and crossfades the RGB LED in the HSL color space
// based on the given photoresistor value.
// - photoresistor_val: The value read from the photoresistor.
//
// - Adapted from the Interactive Physical Computing Textbook by Jon E. Froehlich:
// https://makeabilitylab.github.io/physcomp/arduino/rgb-led-fade.html#full-hsl-based-crossfader-code
// - Used Google Gemini to help debug and generate some parts of code when adding my own tweak 
//   to crossfade the colors back and forth across a spectrum of green/blue to red.
void CrossfadeBrightness(int photoresistor_val);

// Switches the color of the LED based on the color_val.
// - color_val: The value indicating which color to switch LED to
// - brightness: The brightness level of the LED (0-255, low to high respectively)
void ColorSwitch(int color_val, int brightness);

// Sets the color of the RGB LED based on the provided red, green, blue values, and brightness level.
// - red: The intensity of the red color component (0-255).
// - green: The intensity of the green color component (0-255).
// - blue: The intensity of the blue color component (0-255).
// - brightness: The brightness level of the LED (0-255, low to high respectively).
//
// - Adapted from the Interactive Physical Computing Textbook by Jon E. Froehlich:
// https://makeabilitylab.github.io/physcomp/arduino/rgb-led-fade.html#full-hsl-based-crossfader-code
// - Which adapted from https://gist.github.com/jamesotron/766994 (no longer available)
// - Used Google Gemini to help debug code when adding my own tweak for the brightness 
//   parameter
void ChangeColorAndBrightness(int red, int green, int blue, int brightness);

// If main button is triggered, LED will turn off and returns true. Otherwise false.
bool CheckModeSwitch();

void setup() {
  pinMode(kRedPin, OUTPUT);
  pinMode(kGreenPin, OUTPUT);
  pinMode(kBluePin, OUTPUT);
  pinMode(kMainButtonPin, INPUT_PULLUP);
  pinMode(kPhotoresistorPin, INPUT);
}

void loop() {
  button_state = digitalRead(kMainButtonPin);

  if (button_state == LOW) {
    mode++;
    delay(400);

    if (mode == 1) {
      BlinkPink(1);
    } else if (mode == 2) {
      BlinkPink(2);
    } else if (mode == 3) {
      BlinkPink(3);
    } else if (mode > 3) {
      mode = 0;
    }
  }

  switch (mode) {
  case 0:
    analogWrite(kRedPin, 0);
    analogWrite(kGreenPin, 0);
    analogWrite(kBluePin, 0);
    break;

  case 1:
    while (true) {
      if (CheckModeSwitch()) {
        break;
      }

      // detect brightness with photoresistor:
      int photoresistor_val = analogRead(kPhotoresistorPin);
      CrossfadeBrightness(photoresistor_val);
    }
    break;

  case 2:
    while (true) {
      if (CheckModeSwitch()) {
        break;
      }

      // If the value is greater than the kCapacitiveSensorThreshold,
      // switch color as that indicates a click
      long mode_two_color_val = color_select.capacitiveSensor(40);
      if (mode_two_color_val > kCapacitiveSensorThreshold) {
        color_val++;
        delay(400);
      }

      if (color_val > 7) {
        // wrap around if last color reached
        color_val = 0;
      }

      ColorSwitch(color_val, kMaxRgbValue);
    }
    break;

  case 3:
    while (true) {
      if (CheckModeSwitch()) {
        break;
      }

      // If the color select sensor is triggered, go to next color
      long mode_three_color_val = color_select.capacitiveSensor(40);
      if (mode_three_color_val > kCapacitiveSensorThreshold) {
        color_val++;
        delay(400);
        if (color_val > 7) {
          color_val = 0;
        }
      }

      // Adjust brightness based on the brightness sensor value
      int brightness = 0;
      bright_val = brightness_select.capacitiveSensor(40);
      if (bright_val > kCapacitiveSensorThreshold) {
        brightness_mode++;
        delay(400);
        if (brightness_mode > 2) {
          brightness_mode = 0;
        }
      }

      // Set the brightness level based on the brightness mode
      if (brightness_mode == 0) {
        brightness = 20;
      } else if (brightness_mode == 1) {
        brightness = 100;
      } else if (brightness_mode == 2) {
        brightness = kMaxRgbValue;
      }

      // Set the color with the adjusted brightness
      ColorSwitch(color_val, brightness);

      // Check if the potentiometer value is within the "off" range.
      // Mimics speed selection with potentiometer val
      int pot_val = analogRead(A4);
      int new_delay_val = map(pot_val, 0, 540, 3, 10);
      int delay_val = new_delay_val;

      if (pot_val < 500) {
        myservo.attach(kServoPin);
        for (int pos = 0; pos <= 180; pos++) {
          myservo.write(pos);
          delay_val = map(analogRead(A4), 0, 1023, 3, 10);
          delay(delay_val);
        }

        for (int pos = 180; pos >= 0; pos--) {
          myservo.write(pos);
          delay_val = map(analogRead(A4), 0, 540, 3, 10);
          delay(delay_val);
        }
      }

      myservo.detach();
    }

    color_val = 0;
    delay(100);
    myservo.detach();
    break;
  }
}

////////// Function definitions //////////
void CrossfadeBrightness(int photoresistor_val) {
  static bool is_forward = true;
  byte rgb[3];
  rgb_converter.hslToRgb(hue, 1, 0.5, rgb);

  // Adjusts target LED value based on the photoresistor value
  int target_led_val;
  if (photoresistor_val <= 775) {
    target_led_val = map(photoresistor_val, kMinPhotoresistor, 790, 0, 40);
  } else {
    target_led_val = map(photoresistor_val, 790, kMaxPhotoresistor, 40, 255);
  }
  target_led_val = constrain(target_led_val, 0, 255);

  // Smooths the LED brightness transition
  int led_val = (target_led_val * kSmoothingFactor + last_led_val * (100 - kSmoothingFactor)) / 100;
  last_led_val = led_val;

  // Sets the color and brightness of the LED
  ChangeColorAndBrightness(rgb[0] * led_val / 255, rgb[1] * led_val / 255, rgb[2] * led_val / 255, 255);

  // Crossfades across a spectrum, cutting out the yellows/bright greens, and 
  // fades back and forth on the ends of the spectrum
  if (is_forward) {
    hue += step;
    if (hue >= 0.995) {
      hue = 0.995;
      is_forward = false;
    }
  } else {
    hue -= step;
    if (hue <= 0.355) {
      hue = 0.355;
      is_forward = true;
    }
  }
  delay(kDelayInterval);
}

void ColorSwitch(int color_val, int brightness) {
  switch (color_val) {
  case 0:
    ChangeColorAndBrightness(0, 0, 0, brightness);
    break;
  case 1:
    ChangeColorAndBrightness(255, 0, 0, brightness);
    break;
  case 2:
    ChangeColorAndBrightness(255, 20, 0, brightness);
    break;
  case 3:
    ChangeColorAndBrightness(255, 75, 0, brightness);
    break;
  case 4:
    ChangeColorAndBrightness(0, 255, 5, brightness);
    break;
  case 5:
    ChangeColorAndBrightness(0, 5, 255, brightness);
    break;
  case 6:
    ChangeColorAndBrightness(200, 0, 200, brightness);
    break;
  case 7:
    ChangeColorAndBrightness(255, 20, 120, brightness);
    break;
  }
}

void ChangeColorAndBrightness(int red, int green, int blue, int brightness) {
  // scale the color values by the brightness level
  red = (red * brightness) / 255;
  green = (green * brightness) / 255;
  blue = (blue * brightness) / 255;

  analogWrite(kRedPin, red);
  analogWrite(kGreenPin, green);
  analogWrite(kBluePin, blue);
}

void BlinkPink(int times) {
  // more blinks will be faster
  int delay_time = 200 - (times * 25);
  for (int i = 0; i < times; i++) {
    // turn on 
    ChangeColorAndBrightness(255, 20, 120, 100);
    delay(delay_time);
    // turn off
    ChangeColorAndBrightness(0, 0, 0, 0);
    delay(100);
  }
  delay(300);
}

bool CheckModeSwitch() {
  button_state = digitalRead(kMainButtonPin);
  if (button_state == LOW) {
    color_val = 0;
    analogWrite(kRedPin, 0);
    analogWrite(kGreenPin, 0);
    analogWrite(kBluePin, 0);
    return true;
  }
  return false;
}
