/*
  SoftWire.h - Software I2C (bit-bang) library for CH573

  This library intentionally does not modify Wire.
  It provides a Wire-like master API using GPIO bit-banging.
*/

#ifndef SoftWire_h
#define SoftWire_h

#include "Stream.h"
#include "Arduino.h"

#define SOFTWIRE_BUFFER_LENGTH 32
#if !defined(SOFTWIRE_MAX_TX_BUFF_LENGTH)
  #define SOFTWIRE_MAX_TX_BUFF_LENGTH 1024U
#endif

#if defined(CH573)
  #define SOFTWIRE_DEFAULT_SDA PB12
  #define SOFTWIRE_DEFAULT_SCL PB13
#else
  #define SOFTWIRE_DEFAULT_SDA PNUM_NOT_DEFINED
  #define SOFTWIRE_DEFAULT_SCL PNUM_NOT_DEFINED
#endif

class SoftWire : public Stream {
  public:
    SoftWire();
    SoftWire(uint32_t sda, uint32_t scl);

    void setSDA(uint32_t sda);
    void setSCL(uint32_t scl);
    void setSDA(PinName sda);
    void setSCL(PinName scl);

    void begin();
    void begin(uint32_t sda, uint32_t scl);
    void end();

    void setClock(uint32_t frequency);

    void beginTransmission(uint8_t address);
    void beginTransmission(int address);

    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t sendStop);

    uint8_t requestFrom(uint8_t address, uint8_t quantity);
    uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);
    uint8_t requestFrom(uint8_t address, size_t quantity, bool sendStop);
    uint8_t requestFrom(uint8_t address, uint8_t quantity,
                        uint32_t iaddress, uint8_t isize, uint8_t sendStop);
    uint8_t requestFrom(int address, int quantity);
    uint8_t requestFrom(int address, int quantity, int sendStop);

    virtual size_t write(uint8_t data);
    virtual size_t write(const uint8_t *data, size_t quantity);

    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);

    inline size_t write(unsigned long n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(long n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(unsigned int n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(int n)
    {
      return write((uint8_t)n);
    }

    using Print::write;

  private:
    enum SoftWireStatus {
      SW_OK = 0,
      SW_DATA_TOO_LONG,
      SW_NACK_ADDR,
      SW_NACK_DATA,
      SW_ERROR,
      SW_TIMEOUT,
      SW_BUSY,
      SW_NOT_SUPPORTED
    };

    uint8_t *rxBuffer;
    uint16_t rxBufferAllocated;
    uint16_t rxBufferIndex;
    uint16_t rxBufferLength;

    uint8_t txAddress;
    uint8_t *txBuffer;
    uint16_t txBufferAllocated;
    uint16_t txDataSize;

    uint8_t transmitting;

    uint32_t _sdaPin;
    uint32_t _sclPin;
    uint16_t _delayUs;
    uint16_t _stretchTimeoutMs;
    bool _enabled;
    bool _started;
    bool _warnedUnsupported;

    bool allocateRxBuffer(size_t length);
    size_t allocateTxBuffer(size_t length);
    void resetRxBuffer(void);
    void resetTxBuffer(void);

    void recoverBus(void);

    void sdaHigh(void);
    void sdaLow(void);
    bool sclHighWithStretch(void);
    void sclLow(void);
    uint8_t sdaRead(void);
    void i2cDelay(void);

    bool startCondition(void);
    bool stopCondition(void);

    bool writeBit(uint8_t bit);
    bool readBit(uint8_t *bit);

    SoftWireStatus writeByte(uint8_t value, bool *acked);
    SoftWireStatus readByte(uint8_t *value, bool ack);

    SoftWireStatus writeTransfer(uint8_t address7, const uint8_t *data,
                                 uint16_t size, bool sendStop);
    SoftWireStatus readTransfer(uint8_t address7, uint8_t *data,
                                uint16_t size, bool sendStop);
};

extern SoftWire WireSW;

#endif
