/**
 *******************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * This software component is licensed by WCH under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

#include "pins_arduino.h"




// Digital PinName array
const PinName digitalPin[] = {
  PA_4,   //D0/A0    RXD3
  PA_5,   //D1/A1    TXD3
  PA_8,   //D2/A12   RXD1
  PA_9,   //D3/A13   TXD1/TMR0
  PA_10,  //D4       TMR1/X32KI
  PA_11,  //D5       TMR2/X32KO
  PA_12,  //D6/A2    PWM4/SCS
  PA_13,  //D7/A3    PWM5/SCK0
  PA_14,  //D8/A4    MOSI/TXD0_
  PA_15,  //D9/A5    MISO/RXD0_
  PB_0,   //D10/A8   PWM6/CTS
  PB_4,   //D11      PWM7/RXD0
  PB_6,   //D12/A9   PWM8/RTS
  PB_7,   //D13      PWM9/TXD0
  PB_10,  //D14      UD-/TMR1_
  PB_11,  //D15      UD+/TMR2_
  PB_12,  //D16      SCS_/RXD1_ (5VT)
  PB_13,  //D17      SCK0_/TXD1_ (5VT)
  PB_14,  //D18      TIO/PWM10/DSR/MOSI_ (5VT)
  PB_15,  //D19      TCK/DTR/MISO_ (5VT)
  PB_22,  //D20      RXD2/TMR3
  PB_23   //D21      RST#/PWM11/TXD2/TMR0_
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  // 0,  // A0,  PA0
  // 1,  // A1,  PA1
  // 2,  // A2,  PA2
  // 3,  // A3,  PA3
  // 4,  // A4,  PA4
  // 5   // A5,  PA5
};



