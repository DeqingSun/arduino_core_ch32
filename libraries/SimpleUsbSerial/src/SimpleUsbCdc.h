#ifndef __CH573_USB_CDC_H__
#define __CH573_USB_CDC_H__

// clang-format off
#include <stdint.h>
#if defined (CH57x)
#include "CH573SFR.h"
#include "core_riscv.h"
#elif defined (CH32X035)
#include "ch32x035_rcc.h"
#endif
// clang-format on

#ifdef __cplusplus
extern "C" {
#endif

void USBInit(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif