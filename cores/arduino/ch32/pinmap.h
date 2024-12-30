
/** \addtogroup hal */
/** @{*/
/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _PINMAP_H
#define _PINMAP_H

#include "PinNames.h"
#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CH32V10x) || defined(CH32V20x) || defined(CH32V30x) || defined(CH32V30x_C) || defined(CH32L10x)
extern const uint32_t pin_map[16];
#elif defined(CH32V00x) || defined(CH32VM00X)
extern const uint32_t pin_map[8];
#elif defined(CH32X035)
extern const uint32_t pin_map[24];
#elif defined(CH57x)
extern const uint32_t pin_map[1];  //todo: need to be fixed
#endif


#ifndef GPIO_AF_NONE
#define GPIO_AF_NONE 0
#endif
#define CH_MAP_GPIO_PIN(X) (pin_map[CH_PIN(X)])

// No peripheral
#define NP       0U

typedef struct {
  PinName pin;
  void *peripheral;
  int function;
} PinMap;

bool pin_in_pinmap(PinName pin, const PinMap *map);
void pin_function(PinName pin, int function);

static inline PinName pin_pinName(const PinMap *map)
{
  return map->pin;
}

void  pinmap_pinout(PinName pin, const PinMap *map);
void *pinmap_find_peripheral(PinName pin, const PinMap *map);
void *pinmap_peripheral(PinName pin, const PinMap *map);
PinName pinmap_find_pin(void *peripheral, const PinMap *map);
PinName pinmap_pin(void *peripheral, const PinMap *map);
uint32_t pinmap_find_function(PinName pin, const PinMap *map);
uint32_t pinmap_function(PinName pin, const PinMap *map);
void *pinmap_merge_peripheral(void *a, void *b);

#ifdef __cplusplus
}
#endif

#endif

/** @}*/
