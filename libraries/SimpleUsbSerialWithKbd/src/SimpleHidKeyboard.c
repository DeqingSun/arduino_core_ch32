// clang-format off
#include <stdint.h>
#include <stdbool.h>
#include "USBconstant.h"
#include "USBhandler.h"
// clang-format on

// clang-format off
extern uint8_t Ep3Buffer[];
// clang-format on

extern uint8_t keyboardLedStatus;
volatile extern uint8_t UsbConfig;

volatile uint8_t UpPoint3_Busy =
    0; // Flag of whether upload pointer is busy

uint8_t HIDKey[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

#define SHIFT 0x80
const uint8_t _asciimap[128] = {
    0x00, // NUL
    0x00, // SOH
    0x00, // STX
    0x00, // ETX
    0x00, // EOT
    0x00, // ENQ
    0x00, // ACK
    0x00, // BEL
    0x2a, // BS	Backspace
    0x2b, // TAB	Tab
    0x28, // LF	Enter
    0x00, // VT
    0x00, // FF
    0x00, // CR
    0x00, // SO
    0x00, // SI
    0x00, // DEL
    0x00, // DC1
    0x00, // DC2
    0x00, // DC3
    0x00, // DC4
    0x00, // NAK
    0x00, // SYN
    0x00, // ETB
    0x00, // CAN
    0x00, // EM
    0x00, // SUB
    0x00, // ESC
    0x00, // FS
    0x00, // GS
    0x00, // RS
    0x00, // US

    0x2c,         //  ' '
    0x1e | SHIFT, // !
    0x34 | SHIFT, // "
    0x20 | SHIFT, // #
    0x21 | SHIFT, // $
    0x22 | SHIFT, // %
    0x24 | SHIFT, // &
    0x34,         // '
    0x26 | SHIFT, // (
    0x27 | SHIFT, // )
    0x25 | SHIFT, // *
    0x2e | SHIFT, // +
    0x36,         // ,
    0x2d,         // -
    0x37,         // .
    0x38,         // /
    0x27,         // 0
    0x1e,         // 1
    0x1f,         // 2
    0x20,         // 3
    0x21,         // 4
    0x22,         // 5
    0x23,         // 6
    0x24,         // 7
    0x25,         // 8
    0x26,         // 9
    0x33 | SHIFT, // :
    0x33,         // ;
    0x36 | SHIFT, // <
    0x2e,         // =
    0x37 | SHIFT, // >
    0x38 | SHIFT, // ?
    0x1f | SHIFT, // @
    0x04 | SHIFT, // A
    0x05 | SHIFT, // B
    0x06 | SHIFT, // C
    0x07 | SHIFT, // D
    0x08 | SHIFT, // E
    0x09 | SHIFT, // F
    0x0a | SHIFT, // G
    0x0b | SHIFT, // H
    0x0c | SHIFT, // I
    0x0d | SHIFT, // J
    0x0e | SHIFT, // K
    0x0f | SHIFT, // L
    0x10 | SHIFT, // M
    0x11 | SHIFT, // N
    0x12 | SHIFT, // O
    0x13 | SHIFT, // P
    0x14 | SHIFT, // Q
    0x15 | SHIFT, // R
    0x16 | SHIFT, // S
    0x17 | SHIFT, // T
    0x18 | SHIFT, // U
    0x19 | SHIFT, // V
    0x1a | SHIFT, // W
    0x1b | SHIFT, // X
    0x1c | SHIFT, // Y
    0x1d | SHIFT, // Z
    0x2f,         // [
    0x31,         // bslash
    0x30,         // ]
    0x23 | SHIFT, // ^
    0x2d | SHIFT, // _
    0x35,         // `
    0x04,         // a
    0x05,         // b
    0x06,         // c
    0x07,         // d
    0x08,         // e
    0x09,         // f
    0x0a,         // g
    0x0b,         // h
    0x0c,         // i
    0x0d,         // j
    0x0e,         // k
    0x0f,         // l
    0x10,         // m
    0x11,         // n
    0x12,         // o
    0x13,         // p
    0x14,         // q
    0x15,         // r
    0x16,         // s
    0x17,         // t
    0x18,         // u
    0x19,         // v
    0x1a,         // w
    0x1b,         // x
    0x1c,         // y
    0x1d,         // z
    0x2f | SHIFT, // {
    0x31 | SHIFT, // |
    0x30 | SHIFT, // }
    0x35 | SHIFT, // ~
    0             // DEL
};

typedef void (*pTaskFn)(void);

void delayMicroseconds(uint16_t us);

void USB_EP3_IN() {
#if defined(CH57x)
    R8_UEP3_T_LEN = 0;
    R8_UEP3_CTRL = R8_UEP3_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // Default NAK
#elif defined(CH32X035)
    USBFSD->UEP3_TX_LEN = 0;
    USBFSD->UEP3_CTRL_H = USBFSD->UEP3_CTRL_H & ~USBFS_UEP_T_RES_MASK | USBFS_UEP_T_RES_NAK; // Default NAK
#endif
  UpPoint3_Busy = 0;                                       // Clear busy flag
}

uint8_t USB_EP3_send() {
  if (UsbConfig == 0) {
    return 0;
  }

  uint16_t waitWriteCount = 0;

  waitWriteCount = 0;
  while (UpPoint3_Busy) { // wait for 250ms or give up
    waitWriteCount++;
    //delayMicroseconds(5);
    //TODO: the following is a busy wait for CH573, need more test for CH32X035
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");
    if (waitWriteCount >= 50000)
      return 0;
  }

  for (uint8_t i = 0; i < sizeof(HIDKey); i++) { // load data for upload
    Ep3Buffer[0 + i] = HIDKey[i];
  }

#if defined(CH57x)
    R8_UEP3_T_LEN = sizeof(HIDKey); // data length
#elif defined(CH32X035)
    USBFSD->UEP3_TX_LEN = sizeof(HIDKey); // data length
#endif
  UpPoint3_Busy = 1;
#if defined(CH57x)
    R8_UEP3_CTRL = R8_UEP3_CTRL & ~MASK_UEP_T_RES |
              UEP_T_RES_ACK; // upload data and respond ACK
#elif defined(CH32X035)
    USBFSD->UEP3_CTRL = USBFSD->UEP3_CTRL & ~USBFS_UEP_T_RES_MASK |
              USBFS_UEP_T_RES_ACK; // upload data and respond ACK
#endif

  return 1;
}

uint8_t Keyboard_press(uint8_t k) {
  uint8_t i;
  if (k >= 136) { // it's a non-printing key (not a modifier)
    k = k - 136;
  } else if (k >= 128) { // it's a modifier key
    HIDKey[0] |= (1 << (k - 128));
    k = 0;
  } else { // it's a printing key
    k = _asciimap[k];
    if (!k) {
      // setWriteError();
      return 0;
    }
    if (k &
        0x80) { // it's a capital letter or other character reached with shift
      HIDKey[0] |= 0x02; // the left shift modifier
      k &= 0x7F;
    }
  }

  // Add k to the key report only if it's not already present
  // and if there is an empty slot.
  if (HIDKey[2] != k && HIDKey[3] != k && HIDKey[4] != k && HIDKey[5] != k &&
      HIDKey[6] != k && HIDKey[7] != k) {

    for (i = 2; i < 8; i++) {
      if (HIDKey[i] == 0x00) {
        HIDKey[i] = k;
        break;
      }
    }
    if (i == 8) {
      // setWriteError();
      return 0;
    }
  }
  USB_EP3_send();
  return 1;
}

uint8_t Keyboard_release(uint8_t k) {
  uint8_t i;
  if (k >= 136) { // it's a non-printing key (not a modifier)
    k = k - 136;
  } else if (k >= 128) { // it's a modifier key
    HIDKey[0] &= ~(1 << (k - 128));
    k = 0;
  } else { // it's a printing key
    k = _asciimap[k];
    if (!k) {
      return 0;
    }
    if (k &
        0x80) { // it's a capital letter or other character reached with shift
      HIDKey[0] &= ~(0x02); // the left shift modifier
      k &= 0x7F;
    }
  }

  // Test the key report to see if k is present.  Clear it if it exists.
  // Check all positions in case the key is present more than once (which it
  // shouldn't be)
  for (i = 2; i < 8; i++) {
    if (0 != k && HIDKey[i] == k) {
      HIDKey[i] = 0x00;
    }
  }

  USB_EP3_send();
  return 1;
}

void Keyboard_releaseAll(void) {
  for (uint8_t i = 0; i < sizeof(HIDKey); i++) { // load data for upload
    HIDKey[i] = 0;
  }
  USB_EP3_send();
}

uint8_t Keyboard_write(uint8_t c) {
  uint8_t p = Keyboard_press(c); // Keydown
  Keyboard_release(c);                  // Keyup
  return p; // just return the result of press() since release() almost always
            // returns 1
}

void Keyboard_print(char *str) {
  // using a generic pointer to handle pointer in any address space
  uint8_t c;
  while ((c = *str++)) {
    Keyboard_write(c);
  }
}

uint8_t Keyboard_getLEDStatus() {
  // keyboardLedStatus is updated from USB_EP0_OUT
  return keyboardLedStatus;
}
