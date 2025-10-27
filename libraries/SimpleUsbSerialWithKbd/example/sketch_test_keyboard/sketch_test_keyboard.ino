#include <SimpleUsbSerialWithKbd.h>

#define BUTTON_PIN PB0

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
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

  static bool buttonWasPressed = false;
  bool buttonIsPressed = (digitalRead(BUTTON_PIN) == LOW);
  if (buttonIsPressed != buttonWasPressed) {
    if (buttonIsPressed) {
      // Button was just pressed
      Keyboard.press('A');
      buttonWasPressed = true;
    } else if (!buttonIsPressed) {
      // Button was just released
      Keyboard.release('A');
      buttonWasPressed = false;
    }
    delay(50);  // simple debounce
  }
}
