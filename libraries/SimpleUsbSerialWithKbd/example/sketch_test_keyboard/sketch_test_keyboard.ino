#include <SimpleUsbSerialWithKbd.h>

#define BUTTON_PIN1 PB0
#define BUTTON_PIN2 PB14

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin();

  pinMode(BUTTON_PIN1, INPUT_PULLUP);
  pinMode(BUTTON_PIN2, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:

  int incoming = SerialUSB.available();
  if (incoming > 0) {
    for (int i = 0; i < incoming; i++) {
      char c = SerialUSB.read();
      SerialUSB.print(c);
    }
    SerialUSB.println("Echoed!");
    SerialUSB.flush();
  }

  static bool button1WasPressed = false;
  bool button1IsPressed = (digitalRead(BUTTON_PIN1) == LOW);
  if (button1IsPressed != button1WasPressed) {
    if (button1IsPressed) {
      // Button was just pressed
      Keyboard.press('A');
      button1WasPressed = true;
    } else if (!button1IsPressed) {
      // Button was just released
      Keyboard.release('A');
      button1WasPressed = false;
    }
    delay(50);  // simple debounce
  }

  static bool button2WasPressed = false;
  bool button2IsPressed = (digitalRead(BUTTON_PIN2) == LOW);
  if (button2IsPressed != button2WasPressed) {
    if (button2IsPressed) {
      // Button was just pressed
      Mouse.press(MOUSE_LEFT);
      button2WasPressed = true;
    } else if (!button2IsPressed) {
      // Button was just released
      Mouse.release(MOUSE_LEFT);
      button2WasPressed = false;
    }
    delay(50);  // simple debounce
  }
}
