/*
  SoftWire.cpp - Software I2C (bit-bang) library for CH573
*/

extern "C" {
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
}

#include "SoftWire.h"
#include "core_debug.h"

#if !defined(CH573)
#warning "SoftWire currently supports CH573 only. Methods are stubbed on other chips."
#endif

SoftWire WireSW;

SoftWire::SoftWire()
  : rxBuffer(nullptr),
    rxBufferAllocated(0),
    rxBufferIndex(0),
    rxBufferLength(0),
    txAddress(0),
    txBuffer(nullptr),
    txBufferAllocated(0),
    txDataSize(0),
    transmitting(0),
    _sdaPin(SOFTWIRE_DEFAULT_SDA),
    _sclPin(SOFTWIRE_DEFAULT_SCL),
    _delayUs(5),
    _stretchTimeoutMs(10),
    _enabled(false),
    _started(false),
    _warnedUnsupported(false)
{
}

SoftWire::SoftWire(uint32_t sda, uint32_t scl)
  : rxBuffer(nullptr),
    rxBufferAllocated(0),
    rxBufferIndex(0),
    rxBufferLength(0),
    txAddress(0),
    txBuffer(nullptr),
    txBufferAllocated(0),
    txDataSize(0),
    transmitting(0),
    _sdaPin(sda),
    _sclPin(scl),
    _delayUs(5),
    _stretchTimeoutMs(10),
    _enabled(false),
    _started(false),
    _warnedUnsupported(false)
{
}

void SoftWire::setSDA(uint32_t sda)
{
  _sdaPin = sda;
}

void SoftWire::setSCL(uint32_t scl)
{
  _sclPin = scl;
}

void SoftWire::setSDA(PinName sda)
{
  uint32_t pin = pinNametoDigitalPin(sda);
  if (pin < NUM_DIGITAL_PINS) {
    _sdaPin = pin;
  }
}

void SoftWire::setSCL(PinName scl)
{
  uint32_t pin = pinNametoDigitalPin(scl);
  if (pin < NUM_DIGITAL_PINS) {
    _sclPin = pin;
  }
}

void SoftWire::begin(uint32_t sda, uint32_t scl)
{
  _sdaPin = sda;
  _sclPin = scl;
  begin();
}

void SoftWire::begin()
{
  rxBufferIndex = 0;
  rxBufferLength = 0;
  resetRxBuffer();

  txAddress = 0;
  txDataSize = 0;
  transmitting = 0;
  resetTxBuffer();

#if defined(CH573)
  if ((_sdaPin >= NUM_DIGITAL_PINS) || (_sclPin >= NUM_DIGITAL_PINS)) {
    _enabled = false;
    return;
  }

  _enabled = true;
  _started = false;

  setClock(100000);

  pinMode(_sdaPin, INPUT_PULLUP);
  pinMode(_sclPin, INPUT_PULLUP);

  recoverBus();
#else
  _enabled = false;
  if (!_warnedUnsupported) {
    core_debug("SoftWire: this MCU is not supported. CH573 only.\n");
    _warnedUnsupported = true;
  }
#endif
}

void SoftWire::end()
{
#if defined(CH573)
  if (_enabled && _started) {
    (void)stopCondition();
  }
  if (_sdaPin < NUM_DIGITAL_PINS) {
    pinMode(_sdaPin, INPUT);
  }
  if (_sclPin < NUM_DIGITAL_PINS) {
    pinMode(_sclPin, INPUT);
  }
#endif

  _enabled = false;
  _started = false;

  free(txBuffer);
  txBuffer = nullptr;
  txBufferAllocated = 0;
  txDataSize = 0;

  free(rxBuffer);
  rxBuffer = nullptr;
  rxBufferAllocated = 0;
  rxBufferLength = 0;
  rxBufferIndex = 0;

  transmitting = 0;
}

void SoftWire::setClock(uint32_t frequency)
{
#if defined(CH573)
  if (frequency < 1000U) {
    frequency = 1000U;
  }
  if (frequency > 400000U) {
    frequency = 400000U;
  }

  uint32_t halfPeriodUs = 500000UL / frequency;
  _delayUs = (uint16_t)halfPeriodUs;
#else
  (void)frequency;
#endif
}

void SoftWire::beginTransmission(uint8_t address)
{
  transmitting = 1;
  txAddress = (uint8_t)(address & 0x7F);
  txDataSize = 0;
}

void SoftWire::beginTransmission(int address)
{
  beginTransmission((uint8_t)address);
}

uint8_t SoftWire::endTransmission(void)
{
  return endTransmission((uint8_t)true);
}

uint8_t SoftWire::endTransmission(uint8_t sendStop)
{
  uint8_t ret = 4;

  if (!_enabled || (transmitting == 0)) {
    return ret;
  }

  SoftWireStatus status = writeTransfer(txAddress, txBuffer, txDataSize, sendStop != 0);

  switch (status) {
    case SW_OK:
      ret = 0;
      break;
    case SW_DATA_TOO_LONG:
      ret = 1;
      break;
    case SW_NACK_ADDR:
      ret = 2;
      break;
    case SW_NACK_DATA:
      ret = 3;
      break;
    case SW_NOT_SUPPORTED:
    case SW_ERROR:
    case SW_TIMEOUT:
    case SW_BUSY:
    default:
      ret = 4;
      break;
  }

  resetTxBuffer();
  txDataSize = 0;
  transmitting = 0;

  return ret;
}

uint8_t SoftWire::requestFrom(uint8_t address, uint8_t quantity)
{
  return requestFrom(address, quantity, (uint8_t)true);
}

uint8_t SoftWire::requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop)
{
  return requestFrom(address, quantity, (uint32_t)0, (uint8_t)0, sendStop);
}

uint8_t SoftWire::requestFrom(uint8_t address, size_t quantity, bool sendStop)
{
  return requestFrom(address, (uint8_t)quantity, (uint8_t)sendStop);
}

uint8_t SoftWire::requestFrom(uint8_t address, uint8_t quantity,
                              uint32_t iaddress, uint8_t isize, uint8_t sendStop)
{
  uint8_t read = 0;

  if ((!_enabled) || (quantity == 0)) {
    return 0;
  }

  if (!allocateRxBuffer(quantity) || (rxBuffer == nullptr) || (rxBufferAllocated < quantity)) {
    return 0;
  }

  if (isize > 0) {
    beginTransmission(address);

    if (isize > 3) {
      isize = 3;
    }

    while (isize-- > 0) {
      write((uint8_t)(iaddress >> (isize * 8)));
    }

    if (endTransmission((uint8_t)false) != 0) {
      rxBufferIndex = 0;
      rxBufferLength = 0;
      return 0;
    }
  }

  SoftWireStatus status = readTransfer((uint8_t)(address & 0x7F), rxBuffer, quantity, sendStop != 0);
  if (status == SW_OK) {
    read = quantity;
  }

  rxBufferIndex = 0;
  rxBufferLength = read;

  return read;
}

uint8_t SoftWire::requestFrom(int address, int quantity)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
}

uint8_t SoftWire::requestFrom(int address, int quantity, int sendStop)
{
  return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
}

size_t SoftWire::write(uint8_t data)
{
  if (!transmitting) {
    return 0;
  }

  if (allocateTxBuffer(txDataSize + 1) == 0) {
    return 0;
  }

  txBuffer[txDataSize] = data;
  txDataSize++;

  return 1;
}

size_t SoftWire::write(const uint8_t *data, size_t quantity)
{
  if ((!transmitting) || (data == nullptr) || (quantity == 0)) {
    return 0;
  }

  if (allocateTxBuffer(txDataSize + quantity) == 0) {
    return 0;
  }

  memcpy(txBuffer + txDataSize, data, quantity);
  txDataSize += quantity;

  return quantity;
}

int SoftWire::available(void)
{
  return (int)(rxBufferLength - rxBufferIndex);
}

int SoftWire::read(void)
{
  int value = -1;

  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
    rxBufferIndex++;
  }

  return value;
}

int SoftWire::peek(void)
{
  int value = -1;

  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
  }

  return value;
}

void SoftWire::flush(void)
{
}

bool SoftWire::allocateRxBuffer(size_t length)
{
  if (length == 0) {
    return true;
  }

  if (rxBufferAllocated < length) {
    if (length < SOFTWIRE_BUFFER_LENGTH) {
      length = SOFTWIRE_BUFFER_LENGTH;
    }

    uint8_t *tmp = (uint8_t *)realloc(rxBuffer, length * sizeof(uint8_t));
    if (tmp != nullptr) {
      rxBuffer = tmp;
      rxBufferAllocated = (uint16_t)length;
    } else {
      return false;
    }
  }

  return true;
}

size_t SoftWire::allocateTxBuffer(size_t length)
{
  size_t ret = length;

  if (length > SOFTWIRE_MAX_TX_BUFF_LENGTH) {
    ret = 0;
  } else if (txBufferAllocated < length) {
    if (length < SOFTWIRE_BUFFER_LENGTH) {
      length = SOFTWIRE_BUFFER_LENGTH;
    }

    uint8_t *tmp = (uint8_t *)realloc(txBuffer, length * sizeof(uint8_t));
    if (tmp != nullptr) {
      txBuffer = tmp;
      txBufferAllocated = (uint16_t)length;
    } else {
      ret = 0;
    }
  }

  return ret;
}

void SoftWire::resetRxBuffer(void)
{
  if (rxBuffer != nullptr) {
    memset(rxBuffer, 0, rxBufferAllocated);
  }
}

void SoftWire::resetTxBuffer(void)
{
  if (txBuffer != nullptr) {
    memset(txBuffer, 0, txBufferAllocated);
  }
}

void SoftWire::recoverBus(void)
{
#if defined(CH573)
  if (!_enabled) {
    return;
  }

  sdaHigh();
  (void)sclHighWithStretch();

  if (sdaRead() == LOW) {
    for (int i = 0; i < 20; i++) {
      sclLow();
      delayMicroseconds(10);
      if (!sclHighWithStretch()) {
        break;
      }
      delayMicroseconds(10);
      if (sdaRead() == HIGH) {
        break;
      }
    }
    (void)stopCondition();
  }
#endif
}

void SoftWire::sdaHigh(void)
{
#if defined(CH573)
  pinMode(_sdaPin, INPUT_PULLUP);
#endif
}

void SoftWire::sdaLow(void)
{
#if defined(CH573)
  pinMode(_sdaPin, OUTPUT);
  digitalWrite(_sdaPin, LOW);
#endif
}

bool SoftWire::sclHighWithStretch(void)
{
#if defined(CH573)
  pinMode(_sclPin, INPUT_PULLUP);

  uint32_t start = millis();
  while (digitalRead(_sclPin) == LOW) {
    if ((millis() - start) > _stretchTimeoutMs) {
      return false;
    }
  }

  return true;
#else
  return false;
#endif
}

void SoftWire::sclLow(void)
{
#if defined(CH573)
  pinMode(_sclPin, OUTPUT);
  digitalWrite(_sclPin, LOW);
#endif
}

uint8_t SoftWire::sdaRead(void)
{
#if defined(CH573)
  return (uint8_t)digitalRead(_sdaPin);
#else
  return 1;
#endif
}

void SoftWire::i2cDelay(void)
{
#if defined(CH573)
  delayMicroseconds(_delayUs);
#endif
}

bool SoftWire::startCondition(void)
{
#if defined(CH573)
  if (!_enabled) {
    return false;
  }

  sdaHigh();
  i2cDelay();

  if (!sclHighWithStretch()) {
    return false;
  }
  i2cDelay();

  sdaLow();
  i2cDelay();

  sclLow();
  i2cDelay();

  _started = true;
  return true;
#else
  return false;
#endif
}

bool SoftWire::stopCondition(void)
{
#if defined(CH573)
  if (!_enabled) {
    return false;
  }

  sdaLow();
  i2cDelay();

  if (!sclHighWithStretch()) {
    return false;
  }
  i2cDelay();

  sdaHigh();
  i2cDelay();

  _started = false;
  return true;
#else
  return false;
#endif
}

bool SoftWire::writeBit(uint8_t bit)
{
#if defined(CH573)
  if (bit) {
    sdaHigh();
  } else {
    sdaLow();
  }
  i2cDelay();

  if (!sclHighWithStretch()) {
    return false;
  }
  i2cDelay();

  sclLow();
  i2cDelay();

  return true;
#else
  (void)bit;
  return false;
#endif
}

bool SoftWire::readBit(uint8_t *bit)
{
#if defined(CH573)
  sdaHigh();
  i2cDelay();

  if (!sclHighWithStretch()) {
    return false;
  }
  i2cDelay();

  *bit = sdaRead();

  sclLow();
  i2cDelay();

  return true;
#else
  if (bit != nullptr) {
    *bit = 1;
  }
  return false;
#endif
}

SoftWire::SoftWireStatus SoftWire::writeByte(uint8_t value, bool *acked)
{
#if defined(CH573)
  for (uint8_t i = 0; i < 8; i++) {
    if (!writeBit((uint8_t)((value & 0x80U) != 0U))) {
      return SW_TIMEOUT;
    }
    value <<= 1;
  }

  uint8_t nackBit = 1;
  if (!readBit(&nackBit)) {
    return SW_TIMEOUT;
  }

  if (acked != nullptr) {
    *acked = (nackBit == 0U);
  }

  return SW_OK;
#else
  (void)value;
  if (acked != nullptr) {
    *acked = false;
  }
  return SW_NOT_SUPPORTED;
#endif
}

SoftWire::SoftWireStatus SoftWire::readByte(uint8_t *value, bool ack)
{
#if defined(CH573)
  uint8_t out = 0;
  uint8_t bit = 0;

  for (uint8_t i = 0; i < 8; i++) {
    out <<= 1;
    if (!readBit(&bit)) {
      return SW_TIMEOUT;
    }
    out |= (uint8_t)(bit & 0x01U);
  }

  if (!writeBit((uint8_t)(ack ? 0U : 1U))) {
    return SW_TIMEOUT;
  }

  if (value != nullptr) {
    *value = out;
  }

  return SW_OK;
#else
  (void)ack;
  if (value != nullptr) {
    *value = 0;
  }
  return SW_NOT_SUPPORTED;
#endif
}

SoftWire::SoftWireStatus SoftWire::writeTransfer(uint8_t address7, const uint8_t *data,
                                                 uint16_t size, bool sendStop)
{
#if defined(CH573)
  if (!_enabled) {
    return SW_ERROR;
  }

  if (!startCondition()) {
    return SW_TIMEOUT;
  }

  bool acked = false;
  SoftWireStatus status = writeByte((uint8_t)((address7 << 1) & 0xFEU), &acked);
  if (status != SW_OK) {
    (void)stopCondition();
    return status;
  }
  if (!acked) {
    (void)stopCondition();
    return SW_NACK_ADDR;
  }

  for (uint16_t i = 0; i < size; i++) {
    status = writeByte(data[i], &acked);
    if (status != SW_OK) {
      (void)stopCondition();
      return status;
    }
    if (!acked) {
      (void)stopCondition();
      return SW_NACK_DATA;
    }
  }

  if (sendStop) {
    if (!stopCondition()) {
      return SW_TIMEOUT;
    }
  }

  return SW_OK;
#else
  (void)address7;
  (void)data;
  (void)size;
  (void)sendStop;
  return SW_NOT_SUPPORTED;
#endif
}

SoftWire::SoftWireStatus SoftWire::readTransfer(uint8_t address7, uint8_t *data,
                                                uint16_t size, bool sendStop)
{
#if defined(CH573)
  if (!_enabled) {
    return SW_ERROR;
  }

  if ((size > 0) && (data == nullptr)) {
    return SW_ERROR;
  }

  if (!startCondition()) {
    return SW_TIMEOUT;
  }

  bool acked = false;
  SoftWireStatus status = writeByte((uint8_t)((address7 << 1) | 0x01U), &acked);
  if (status != SW_OK) {
    (void)stopCondition();
    return status;
  }
  if (!acked) {
    (void)stopCondition();
    return SW_NACK_ADDR;
  }

  for (uint16_t i = 0; i < size; i++) {
    bool ack = (i < (uint16_t)(size - 1));
    status = readByte(&data[i], ack);
    if (status != SW_OK) {
      (void)stopCondition();
      return status;
    }
  }

  if (sendStop) {
    if (!stopCondition()) {
      return SW_TIMEOUT;
    }
  }

  return SW_OK;
#else
  (void)address7;
  (void)data;
  (void)size;
  (void)sendStop;
  return SW_NOT_SUPPORTED;
#endif
}
