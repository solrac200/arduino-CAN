// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ARDUINO_ARCH_ESP32

#include "MCP2515.h"

#define REG_BFPCTRL                0x0c
#define REG_TXRTSCTRL              0x0d

#define REG_CANCTRL                0x0f

#define REG_CNF3                   0x28
#define REG_CNF2                   0x29
#define REG_CNF1                   0x2a

#define REG_CANINTE                0x2b

// Whenever changing the CANINTF register, use BIT MODIFY instead of WRITE.
#define REG_CANINTF                0x2c

#define FLAG_RXnIE(n)              (0x01 << n)
#define FLAG_RXnIF(n)              (0x01 << n)
#define FLAG_TXnIF(n)              (0x04 << n)

// There is a 4-register gap between RXF2EID0 and RXF3SIDH.
#define REG_RXFnSIDH(n)            (0x00 + ((n + (n >= 3)) * 4))
#define REG_RXFnSIDL(n)            (0x01 + ((n + (n >= 3)) * 4))
#define REG_RXFnEID8(n)            (0x02 + ((n + (n >= 3)) * 4))
#define REG_RXFnEID0(n)            (0x03 + ((n + (n >= 3)) * 4))

#define REG_RXMnSIDH(n)            (0x20 + (n * 0x04))
#define REG_RXMnSIDL(n)            (0x21 + (n * 0x04))
#define REG_RXMnEID8(n)            (0x22 + (n * 0x04))
#define REG_RXMnEID0(n)            (0x23 + (n * 0x04))

#define REG_TXBnCTRL(n)            (0x30 + (n * 0x10))
#define REG_TXBnSIDH(n)            (0x31 + (n * 0x10))
#define REG_TXBnSIDL(n)            (0x32 + (n * 0x10))
#define REG_TXBnEID8(n)            (0x33 + (n * 0x10))
#define REG_TXBnEID0(n)            (0x34 + (n * 0x10))
#define REG_TXBnDLC(n)             (0x35 + (n * 0x10))
#define REG_TXBnD0(n)              (0x36 + (n * 0x10))

#define REG_RXBnCTRL(n)            (0x60 + (n * 0x10))
#define REG_RXBnSIDH(n)            (0x61 + (n * 0x10))
#define REG_RXBnSIDL(n)            (0x62 + (n * 0x10))
#define REG_RXBnEID8(n)            (0x63 + (n * 0x10))
#define REG_RXBnEID0(n)            (0x64 + (n * 0x10))
#define REG_RXBnDLC(n)             (0x65 + (n * 0x10))
#define REG_RXBnD0(n)              (0x66 + (n * 0x10))

#define FLAG_IDE                   0x08
#define FLAG_SRR                   0x10
#define FLAG_RTR                   0x40
#define FLAG_EXIDE                 0x08
#define FLAG_RXB0CTRL_BUKT         0x04

#define FLAG_RXM0                  0x20
#define FLAG_RXM1                  0x40


MCP2515Class::MCP2515Class() :
  CANControllerClass(),
  _spiSettings(10E6, MSBFIRST, SPI_MODE0),
  _csPin(MCP2515_DEFAULT_CS_PIN),
  _intPin(MCP2515_DEFAULT_INT_PIN),
  _clockFrequency(MCP2515_DEFAULT_CLOCK_FREQUENCY)
{
}

MCP2515Class::~MCP2515Class()
{
}

int MCP2515Class::begin(long baudRate, bool stayInConfigurationMode)
{
  CANControllerClass::begin(baudRate);

  pinMode(_csPin, OUTPUT);

  // start SPI
  SPI.begin();

  reset();

  if (!switchToConfigurationMode()) {
    return 0;
  }

  const struct {
    long clockFrequency;
    long baudRate;
    uint8_t cnf[3];
  } CNF_MAPPER[] = {
    {  (long)8E6, (long)1000E3, { 0x00, 0x80, 0x00 } },
    {  (long)8E6,  (long)500E3, { 0x00, 0x90, 0x02 } },
    {  (long)8E6,  (long)250E3, { 0x00, 0xb1, 0x05 } },
    {  (long)8E6,  (long)200E3, { 0x00, 0xb4, 0x06 } },
    {  (long)8E6,  (long)125E3, { 0x01, 0xb1, 0x05 } },
    {  (long)8E6,  (long)100E3, { 0x01, 0xb4, 0x06 } },
    {  (long)8E6,   (long)80E3, { 0x01, 0xbf, 0x07 } },
    {  (long)8E6,   (long)50E3, { 0x03, 0xb4, 0x06 } },
    {  (long)8E6,   (long)40E3, { 0x03, 0xbf, 0x07 } },
    {  (long)8E6,   (long)20E3, { 0x07, 0xbf, 0x07 } },
    {  (long)8E6,   (long)10E3, { 0x0f, 0xbf, 0x07 } },
    {  (long)8E6,    (long)5E3, { 0x1f, 0xbf, 0x07 } },

    { (long)16E6, (long)1000E3, { 0x00, 0xd0, 0x82 } },
    { (long)16E6,  (long)500E3, { 0x00, 0xf0, 0x86 } },
    { (long)16E6,  (long)250E3, { 0x41, 0xf1, 0x85 } },
    { (long)16E6,  (long)200E3, { 0x01, 0xfa, 0x87 } },
    { (long)16E6,  (long)125E3, { 0x03, 0xf0, 0x86 } },
    { (long)16E6,  (long)100E3, { 0x03, 0xfa, 0x87 } },
    { (long)16E6,   (long)80E3, { 0x03, 0xff, 0x87 } },
    { (long)16E6,   (long)50E3, { 0x07, 0xfa, 0x87 } },
    { (long)16E6,   (long)40E3, { 0x07, 0xff, 0x87 } },
    { (long)16E6,   (long)20E3, { 0x0f, 0xff, 0x87 } },
    { (long)16E6,   (long)10E3, { 0x1f, 0xff, 0x87 } },
    { (long)16E6,    (long)5E3, { 0x3f, 0xff, 0x87 } },
  };

  const uint8_t* cnf = NULL;

  for (unsigned int i = 0; i < (sizeof(CNF_MAPPER) / sizeof(CNF_MAPPER[0])); i++) {
    if (CNF_MAPPER[i].clockFrequency == _clockFrequency && CNF_MAPPER[i].baudRate == baudRate) {
      cnf = CNF_MAPPER[i].cnf;
      break;
    }
  }

  if (cnf == NULL) {
    return 0;
  }

  writeRegister(REG_CNF1, cnf[0]);
  writeRegister(REG_CNF2, cnf[1]);
  writeRegister(REG_CNF3, cnf[2]);

  writeRegister(REG_CANINTE, FLAG_RXnIE(1) | FLAG_RXnIE(0));
  writeRegister(REG_BFPCTRL, 0x00);
  writeRegister(REG_TXRTSCTRL, 0x00);

  // A combination of RXM1 and RXM0 is "Turns mask/filters off; receives any message".
  writeRegister(REG_RXBnCTRL(0), FLAG_RXM1 | FLAG_RXM0);
  writeRegister(REG_RXBnCTRL(1), FLAG_RXM1 | FLAG_RXM0);

  if (!stayInConfigurationMode) {
    if (!switchToNormalMode()) {
      return 0;
    }
  }

  return 1;
}

void MCP2515Class::end()
{
  SPI.end();

  CANControllerClass::end();
}

int MCP2515Class::endPacket()
{
  if (!CANControllerClass::endPacket()) {
    return 0;
  }

  // Currently, we don't need to use more than one TX buffer as we always wait
  // until the data has been fully transmitted. For the same reason, we don't
  // need to check in the beginning whether there is any data in the TX buffer
  // pending transmission. The performance can be optimized by utilizing all
  // three TX buffers of the MCP2515, but this will come at extra complexity.
  int n = 0;

  // Pre-calculate values for all registers so that we can write them
  // sequentially via the LOAD TX BUFFER instruction.
  // TX BUFFER
  uint8_t regSIDH;
  uint8_t regSIDL;
  uint8_t regEID8;
  uint8_t regEID0;
  if (_txExtended) {
    regSIDH = _txId >> 21;
    regSIDL =
        (((_txId >> 18) & 0x07) << 5) | FLAG_EXIDE | ((_txId >> 16) & 0x03);
    regEID8 = (_txId >> 8) & 0xff;
    regEID0 = _txId & 0xff;
  } else {
    regSIDH = _txId >> 3;
    regSIDL = _txId << 5;
    regEID8 = 0x00;
    regEID0 = 0x00;
  }

  uint8_t regDLC;
  if (_txRtr) {
    regDLC = 0x40 | _txLength;
  } else {
    regDLC = _txLength;
  }

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  // Send the LOAD TX BUFFER instruction to sequentially write registers,
  // starting from TXBnSIDH(n).
  SPI.transfer(0b01000000 | (n << 1));
  SPI.transfer(regSIDH);
  SPI.transfer(regSIDL);
  SPI.transfer(regEID8);
  SPI.transfer(regEID0);
  SPI.transfer(regDLC);
  if (!_txRtr) {
    for (uint8_t i = 0; i < _txLength; i++) {
      SPI.transfer(_txData[i]);
    }
  }
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  // Send the RTS instruction, which sets the TXREQ (TXBnCTRL[3]) bit for the
  // respective buffer, and clears the ABTF, MLOA and TXERR bits.
  SPI.transfer(0b10000000 | (1 << n));
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  // Wait until the transmission completes, or gets aborted.
  // Transmission is pending while TXREQ (TXBnCTRL[3]) bit is set.
  bool aborted = false;
  while (readRegister(REG_TXBnCTRL(n)) & 0x08) {
    // Read the TXERR (TXBnCTRL[4]) bit to check for errors.
    if (readRegister(REG_TXBnCTRL(n)) & 0x10) {
      // Abort on errors by setting the ABAT bit. The MCP2515 will should the
      // TXREQ bit shortly. We'll keep running the loop until TXREQ is cleared.
      modifyRegister(REG_CANCTRL, 0x10, 0x10);
      aborted = true;
    }

    yield();
  }

  if (aborted) {
    // Reset the ABAT bit.
    modifyRegister(REG_CANCTRL, 0x10, 0x00);
  }

  // Clear the pending TX interrupt, if any.
  modifyRegister(REG_CANINTF, FLAG_TXnIF(n), 0x00);

  // Report failure if either of the ABTF, MLOA or TXERR bits are set.
  // TODO: perhaps we can reuse the last value read from this register // earlier?
  return (readRegister(REG_TXBnCTRL(n)) & 0x70) ? 0 : 1;
}

int MCP2515Class::parsePacket()
{
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(0xb0);  // RX STATUS
  uint8_t rxStatus = SPI.transfer(0x00);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  int n;
  if (rxStatus & 0x40) {
    n = 0;
  } else if (rxStatus & 0x80) {
    n = 1;
  } else {
    _rxId = -1;
    _rxExtended = false;
    _rxRtr = false;
    _rxDlc = 0;
    _rxIndex = 0;
    _rxLength = 0;
    return 0;
  }

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  // Send READ RX BUFFER instruction to sequentially read registers, starting
  // from RXBnSIDH(n).
  SPI.transfer(0b10010000 | (n * 0x04));
  uint8_t regSIDH = SPI.transfer(0x00);
  uint8_t regSIDL = SPI.transfer(0x00);
  _rxExtended = (regSIDL & FLAG_IDE) ? true : false;

  // We could just skip the extended registers for standard frames, but that
  // would actually add more overhead, and increase complexity.
  uint8_t regEID8 = SPI.transfer(0x00);
  uint8_t regEID0 = SPI.transfer(0x00);
  uint8_t regDLC = SPI.transfer(0x00);
  uint32_t idA = (regSIDH << 3) | (regSIDL >> 5);
  if (_rxExtended) {
    uint32_t idB =
        ((uint32_t)(regSIDL & 0x03) << 16)
        | ((uint32_t)regEID8 << 8)
        | regEID0;

    _rxId = (idA << 18) | idB;
    _rxRtr = (regDLC & FLAG_RTR) ? true : false;
  } else {
    _rxId = idA;
    _rxRtr = (regSIDL & FLAG_SRR) ? true : false;
  }

  _rxDlc = regDLC & 0x0f;
  _rxIndex = 0;

  if (_rxRtr) {
    _rxLength = 0;
  } else {
    _rxLength = _rxDlc;

    // Get the data.
    for (uint8_t i = 0; i < _rxLength; i++) {
      _rxData[i] = SPI.transfer(0x00);
    }
  }

  // Don't need to unset the RXnIF(n) flag as this is done automatically when
  // setting the CS high after a READ RX BUFFER instruction.
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
  return _rxDlc;
}

void MCP2515Class::onReceive(void(*callback)(int))
{
  CANControllerClass::onReceive(callback);

  pinMode(_intPin, INPUT);

  if (callback) {
    SPI.usingInterrupt(digitalPinToInterrupt(_intPin));
    attachInterrupt(digitalPinToInterrupt(_intPin), MCP2515Class::onInterrupt, LOW);
  } else {
    detachInterrupt(digitalPinToInterrupt(_intPin));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_intPin));
#endif
  }
}

int MCP2515Class::filter(int id, int mask)
{
  id &= 0x7ff;
  mask &= 0x7ff;

  // config mode
  writeRegister(REG_CANCTRL, 0x80);
  // TODO: The requested mode must be verified by reading the OPMODE[2:0] bits (CANSTAT[7:5])
  if (readRegister(REG_CANCTRL) != 0x80) {
    return 0;
  }

  for (int n = 0; n < 2; n++) {
    // standard only
    // TODO: This doesn't look correct. According to the datasheet, the RXM0 and
    // RMX1 should either both be unset (in which case filters are active), or
    // both be unset (in which case all filters are ignored).
    // Either way, it's unclear why we write to the same register twice here.
    writeRegister(REG_RXBnCTRL(n), FLAG_RXM0);
    writeRegister(REG_RXBnCTRL(n), FLAG_RXM0);

    writeRegister(REG_RXMnSIDH(n), mask >> 3);
    writeRegister(REG_RXMnSIDL(n), mask << 5);
    writeRegister(REG_RXMnEID8(n), 0);
    writeRegister(REG_RXMnEID0(n), 0);
  }

  for (int n = 0; n < 6; n++) {
    writeRegister(REG_RXFnSIDH(n), id >> 3);
    writeRegister(REG_RXFnSIDL(n), id << 5);
    writeRegister(REG_RXFnEID8(n), 0);
    writeRegister(REG_RXFnEID0(n), 0);
  }

  // normal mode
  writeRegister(REG_CANCTRL, 0x00);
  // TODO: The requested mode must be verified by reading the OPMODE[2:0] bits (CANSTAT[7:5])
  if (readRegister(REG_CANCTRL) != 0x00) {
    return 0;
  }

  return 1;
}

boolean MCP2515Class::setFilterRegisters(
    uint16_t mask0, uint16_t filter0, uint16_t filter1,
    uint16_t mask1, uint16_t filter2, uint16_t filter3, uint16_t filter4, uint16_t filter5,
    bool allowRollover)
{
  mask0 &= 0x7ff;
  filter0 &= 0x7ff;
  filter1 &= 0x7ff;
  mask1 &= 0x7ff;
  filter2 &= 0x7ff;
  filter3 &= 0x7ff;
  filter4 &= 0x7ff;
  filter5 &= 0x7ff;

  if (!switchToConfigurationMode()) {
    return false;
  }

  writeRegister(REG_RXBnCTRL(0), allowRollover ? FLAG_RXB0CTRL_BUKT : 0);
  writeRegister(REG_RXBnCTRL(1), 0);
  for (int n = 0; n < 2; n++) {
    uint8_t mask = (n == 0) ? mask0 : mask1;
    writeRegister(REG_RXMnSIDH(n), mask >> 3);
    writeRegister(REG_RXMnSIDL(n), mask << 5);
    writeRegister(REG_RXMnEID8(n), 0);
    writeRegister(REG_RXMnEID0(n), 0);
  }

  uint8_t filter_array[6] =
      {filter0, filter1, filter2, filter3, filter4, filter5};
  for (int n = 0; n < 6; n++) {
    uint8_t id = filter_array[n];
    writeRegister(REG_RXFnSIDH(n), id >> 3);
    writeRegister(REG_RXFnSIDL(n), id << 5);
    writeRegister(REG_RXFnEID8(n), 0);
    writeRegister(REG_RXFnEID0(n), 0);
  }

  if (!switchToNormalMode()) {
    return false;
  }

  return true;
}

int MCP2515Class::filterExtended(long id, long mask)
{
  id &= 0x1FFFFFFF;
  mask &= 0x1FFFFFFF;

  // config mode
  writeRegister(REG_CANCTRL, 0x80);
  // TODO: The requested mode must be verified by reading the OPMODE[2:0] bits (CANSTAT[7:5])
  if (readRegister(REG_CANCTRL) != 0x80) {
    return 0;
  }

  for (int n = 0; n < 2; n++) {
    // extended only
    // TODO: This doesn't look correct. According to the datasheet, the RXM0 and
    // RMX1 should either both be unset (in which case filters are active), or
    // both be unset (in which case all filters are ignored).
    // Either way, it's unclear why we write to the same register twice here.
    writeRegister(REG_RXBnCTRL(n), FLAG_RXM1);
    writeRegister(REG_RXBnCTRL(n), FLAG_RXM1);

    writeRegister(REG_RXMnSIDH(n), mask >> 21);
    writeRegister(REG_RXMnSIDL(n), (((mask >> 18) & 0x03) << 5) | FLAG_EXIDE | ((mask >> 16) & 0x03));
    writeRegister(REG_RXMnEID8(n), (mask >> 8) & 0xff);
    writeRegister(REG_RXMnEID0(n), mask & 0xff);
  }

  for (int n = 0; n < 6; n++) {
    writeRegister(REG_RXFnSIDH(n), id >> 21);
    writeRegister(REG_RXFnSIDL(n), (((id >> 18) & 0x03) << 5) | FLAG_EXIDE | ((id >> 16) & 0x03));
    writeRegister(REG_RXFnEID8(n), (id >> 8) & 0xff);
    writeRegister(REG_RXFnEID0(n), id & 0xff);
  }

  // normal mode
  writeRegister(REG_CANCTRL, 0x00);
  // TODO: The requested mode must be verified by reading the OPMODE[2:0] bits (CANSTAT[7:5])
  if (readRegister(REG_CANCTRL) != 0x00) {
    return 0;
  }

  return 1;
}

bool MCP2515Class::switchToNormalMode() {
  // TODO: Should we use modifyRegister(REG_CANCTRL, 0xe0, 0x00) here instead?
  writeRegister(REG_CANCTRL, 0x00);
  return (readRegister(REG_CANCTRL) & 0xe0) == 0x00;
}

bool MCP2515Class::switchToConfigurationMode()
{
  // TODO: Should we use modifyRegister(REG_CANCTRL, 0xe0, 0x80) here instead?
  writeRegister(REG_CANCTRL, 0x80);
  return (readRegister(REG_CANCTRL) & 0xe0) == 0x80;
}

int MCP2515Class::observe()
{
  writeRegister(REG_CANCTRL, 0x60);
  if (readRegister(REG_CANCTRL) != 0x60) {
    return 0;
  }

  return 1;
}

int MCP2515Class::loopback()
{
  writeRegister(REG_CANCTRL, 0x40);
  // TODO: The requested mode must be verified by reading the OPMODE[2:0] bits (CANSTAT[7:5])
  if (readRegister(REG_CANCTRL) != 0x40) {
    return 0;
  }

  return 1;
}

int MCP2515Class::sleep()
{
  writeRegister(REG_CANCTRL, 0x01);
  // TODO: The requested mode must be verified by reading the OPMODE[2:0] bits (CANSTAT[7:5])
  if (readRegister(REG_CANCTRL) != 0x01) {
    return 0;
  }

  return 1;
}

int MCP2515Class::wakeup()
{
  writeRegister(REG_CANCTRL, 0x00);
  // TODO: The requested mode must be verified by reading the OPMODE[2:0] bits (CANSTAT[7:5])
  if (readRegister(REG_CANCTRL) != 0x00) {
    return 0;
  }

  return 1;
}

void MCP2515Class::setPins(int cs, int irq)
{
  _csPin = cs;
  _intPin = irq;
}

void MCP2515Class::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void MCP2515Class::setClockFrequency(long clockFrequency)
{
  _clockFrequency = clockFrequency;
}

void MCP2515Class::dumpImportantRegisters(Stream& out) {
  out.print("TEC: ");
  out.println(readRegister(0x1C), HEX);
  out.print("REC: ");
  out.println(readRegister(0x1D), HEX);
  out.print("CANINTE: ");
  out.println(readRegister(0x2B), HEX);

  out.print("CANINTF: ");
  uint8_t regCANINTF = readRegister(0x2C);
  out.print(regCANINTF, HEX);
  if (regCANINTF & 0x80) {
    out.print(" MERRF");
  }
  if (regCANINTF & 0x40) {
    out.print(" WAKIF");
  }
  if (regCANINTF & 0x20) {
    out.print(" ERRIF");
  }
  if (regCANINTF & 0x10) {
    out.print(" TX2IF");
  }
  if (regCANINTF & 0x08) {
    out.print(" TX1IF");
  }
  if (regCANINTF & 0x04) {
    out.print(" TX0IF");
  }
  if (regCANINTF & 0x02) {
    out.print(" RX1IF");
  }
  if (regCANINTF & 0x01) {
    out.print(" RX0IF");
  }
  out.println();

  out.print("EFLG: ");
  uint8_t regEFLG = readRegister(0x2D);
  out.print(regEFLG, HEX);
  if (regEFLG & 0x80) {
    out.print(" RX1OVR");
  }
  if (regEFLG & 0x40) {
    out.print(" RX0OVR");
  }
  if (regEFLG & 0x20) {
    out.print(" TXBO");
  }
  if (regEFLG & 0x10) {
    out.print(" TXEP");
  }
  if (regEFLG & 0x08) {
    out.print(" RXEP");
  }
  if (regEFLG & 0x04) {
    out.print(" TXWAR");
  }
  if (regEFLG & 0x02) {
    out.print(" RXWAR");
  }
  if (regEFLG & 0x01) {
    out.print(" EWARN");
  }
  out.println();
}

void MCP2515Class::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    byte b = readRegister(i);

    out.print("0x");
    if (i < 16) {
      out.print('0');
    }
    out.print(i, HEX);
    out.print(": 0x");
    if (b < 16) {
      out.print('0');
    }
    out.println(b, HEX);
  }
}

void MCP2515Class::reset()
{
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(0xc0);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  // From the data sheet:
  // The OST keeps the device in a Reset state for 128 OSC1 clock cycles after
  // the occurrence of a Power-on Reset, SPI Reset, after the assertion of the
  // RESET pin, and after a wake-up from Sleep mode. It should be noted that no
  // SPI protocol operations should be attempted until after the OST has
  // expired.
  // We sleep for 160 cycles to match the old behavior with 16 MHz quartz, and
  // to be on the safe side for 8 MHz devices.
  delayMicroseconds(ceil(160 * 1000000.0 / _clockFrequency));
}

void MCP2515Class::handleInterrupt()
{
  if (readRegister(REG_CANINTF) == 0) {
    return;
  }

  while (parsePacket() || _rxId != -1) {
    _onReceive(available());
  }
}

uint8_t MCP2515Class::readRegister(uint8_t address)
{
  uint8_t value;

  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(0x03);
  SPI.transfer(address);
  value = SPI.transfer(0x00);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();

  return value;
}

void MCP2515Class::modifyRegister(uint8_t address, uint8_t mask, uint8_t value)
{
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(0x05);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(value);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

void MCP2515Class::writeRegister(uint8_t address, uint8_t value)
{
  SPI.beginTransaction(_spiSettings);
  digitalWrite(_csPin, LOW);
  SPI.transfer(0x02);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(_csPin, HIGH);
  SPI.endTransaction();
}

void MCP2515Class::onInterrupt()
{
  CAN.handleInterrupt();
}

MCP2515Class CAN;

#endif
