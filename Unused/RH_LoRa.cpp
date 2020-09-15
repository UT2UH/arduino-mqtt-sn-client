#include <RH_LoRa.h>

#define MAX_MCU_INTERUPTS 3

#define LORA_FREQUENCY_DEFAULT 915e6
#define PABOOST true

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LR_OCP				 0X0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_PKT_SNR_VALUE        0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PaDac				 0x4d//add REG_PaDac

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06


// IRQ flags stored in REG_IRQ_FLAGS
#define IRQF_RX_TIMEOUT            0x80
#define IRQF_RX_DONE               0x40
#define IRQF_PAYLOAD_CRC_ERROR     0x20
#define IRQF__VALID_HEADER         0x10
#define IRQF_TX_DONE               0x08
#define IRQF_CAD_DONE              0x04
#define IRQF_FHSS_CHANGE_CHANNEL   0x02
#define IRQF_CAD_DETECTED          0x01


RH_LoRa* RH_LoRa::_deviceForInterrupt[MAX_MCU_INTERUPTS] = {0, 0, 0};
uint8_t RH_LoRa::_interruptCount = 0; // Index into _deviceForInterrupt for next device

#if (RH_PLATFORM == RH_PLATFORM_RPI)
#include "wiringPi.h"

void attachInterrupt(uint8_t gpio, void (*function)(void), int mode) {
	pinMode(gpio, INPUT);
	if( wiringPiISR(gpio, mode, function) < 0 ){
		  printf("attachInterrupt() : wiringPiIsr isr failed!\n");
	}
}
#endif


RH_LoRa::RH_LoRa(uint8_t slaveSelectPin, uint8_t DIO0Pin, uint8_t resetPin, RHGenericSPI& spi) :
  RHSPIDriver(slaveSelectPin, spi),
  _dio0(DIO0Pin),
  _reset(resetPin)
{
    _myInterruptIndex = 0xff; // Not allocated yet
}


bool RH_LoRa::init()
{
    if (_initCalled != 0) {
        return (_initCalled == 1);
    }
    _initCalled = -1;

    if (!RHSPIDriver::init()) {
        printf("Failed to initialize SPI driver\n");
        return false;
    }


    pinMode(_reset, OUTPUT);
    pinMode(_dio0, INPUT);

    // Do a chip check...
    digitalWrite(_reset, LOW);
    delay(100);
    digitalWrite(_reset, HIGH);
    delay(100);
    uint8_t version = spiRead(REG_VERSION);
    if (version == 0x12) {
        // sx1276
        printf("SX1276 detected.\n");
    } else {
        printf("SX1276 version check failed (version reported as %d)\n", version);
        return false;
    }

    // put in sleep mode
    setModeSleep();

    // set frequency
    setFrequency(LORA_FREQUENCY_DEFAULT);

    // set base addresses
    spiWrite(REG_FIFO_TX_BASE_ADDR, 0);
    spiWrite(REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    spiWrite(REG_LNA, spiRead(REG_LNA) | 0x03);

    // set auto AGC
    spiWrite(REG_MODEM_CONFIG_3, 0x04);

    // set output power to 14 dBm
    if (PABOOST == true)
        setTxPower(14, RF_PACONFIG_PASELECT_PABOOST);
    else
        setTxPower(14, RF_PACONFIG_PASELECT_RFO);

    setSpreadingFactor(11);

    setSignalBandwidth(125E3);

    setSyncWord(0x34);

    enableCrc();

    setModeIdle();
    
    // Set up interrupt handler
    // Since there are a limited number of interrupt glue functions isr*() available,
    // we can only support a limited number of devices simultaneously.
#if (RH_PLATFORM != RH_PLATFORM_RPI)
    // ON some devices, notably most Arduinos, the interrupt pin passed in is actuallt the
    // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
    // yourself based on knwledge of what Arduino board you are running on.
    // Determine the interrupt number that corresponds to the interruptPin
    int interruptNumber = digitalPinToInterrupt(_dio0);
    if (interruptNumber == NOT_AN_INTERRUPT) {
       printf("Bad interupt initialization\n");
	   return false;
    }
#endif

    if (_myInterruptIndex == 0xff) {
    
	    // First run, no interrupt allocated yet
	    if (_interruptCount < MAX_MCU_INTERUPTS) {
    	    _myInterruptIndex = _interruptCount++;
      }
	    else {
         printf("LoRa radio count exceeds MCU interupt capability.\n");
	       return false; // Too many devices, not enough interrupt vectors
      }
      _deviceForInterrupt[_myInterruptIndex] = this;
   }
   
#if (RH_PLATFORM == RH_PLATFORM_RPI)
	switch( _myInterruptIndex ){
		case 0:
			attachInterrupt(_dio0, isr0, INT_EDGE_RISING);
			break;
		case 1:
			attachInterrupt(_dio0, isr1, INT_EDGE_RISING);
			break;
		case 2:
			attachInterrupt(_dio0, isr2, INT_EDGE_RISING);
			break;
		default: {
            printf("Bad interrupt index: %d\n", _myInterruptIndex);
			return false;
        }
	}
    printf("Interrupt handler assigned on pin %d\n", _dio0);
#else
    if (_myInterruptIndex == 0)
	attachInterrupt(interruptNumber, isr0, RISING);
    else if (_myInterruptIndex == 1)
	attachInterrupt(interruptNumber, isr1, RISING);
    else if (_myInterruptIndex == 2)
	attachInterrupt(interruptNumber, isr2, RISING);
    else {
        printf("Bad interrupt index: %d\n", _myInterruptIndex);
   	    return false; // Too many devices, not enough interrupt vectors
    }
#endif

    _initCalled = 1;
    return true;
}

// C++ level interrupt handler for this instance
// LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefuly
// connnected to the processor.
// We use this to get RxDone and TxDone interrupts
void RH_LoRa::handleInterrupt()
{
#if (RH_PLATFORM == RH_PLATFORM_RPI)
piLock(0);
#endif
    // Read the interrupt register
    uint8_t irq_flags = spiRead(REG_IRQ_FLAGS);
    if (_mode == RHModeRx && irq_flags & (IRQF_RX_TIMEOUT | IRQF_PAYLOAD_CRC_ERROR))
    {
	  _rxBad++;
    }
    else if (_mode == RHModeRx && irq_flags & IRQF_RX_DONE)
    {
        // Have received a packet
        uint8_t len = spiRead(REG_RX_NB_BYTES);

        // Reset the fifo read ptr to the beginning of the packet
        spiWrite(REG_FIFO_ADDR_PTR, spiRead(REG_FIFO_RX_CURRENT_ADDR));
        spiBurstRead(REG_FIFO, _buf, len);
        _bufLen = len;
        spiWrite(REG_IRQ_FLAGS, 0xff); // Clear all IRQ flags

        // Remember the RSSI of this packet
        _lastRssi = packetRssi();

        // We have received a message.
        validateRxBuf();
        
        if (_rxBufValid) {
            setModeIdle(); // Got one
        }
    }
    else if (_mode == RHModeTx && irq_flags & IRQF_TX_DONE)
    {
	  _txGood++;
	  setModeIdle();
    }

    spiWrite(REG_IRQ_FLAGS, 0xff); // Clear all IRQ flags
#if (RH_PLATFORM == RH_PLATFORM_RPI)
piUnlock(0);
#endif
}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_LoRa.
// 3 interrupts allows us to have 3 different devices
void RH_LoRa::isr0()
{
    if (_deviceForInterrupt[0])
	_deviceForInterrupt[0]->handleInterrupt();
}
void RH_LoRa::isr1()
{
    if (_deviceForInterrupt[1])
	_deviceForInterrupt[1]->handleInterrupt();
}
void RH_LoRa::isr2()
{
    if (_deviceForInterrupt[2])
	_deviceForInterrupt[2]->handleInterrupt();
}

// Check whether the latest received message is complete and uncorrupted
void RH_LoRa::validateRxBuf()
{
    if (_bufLen < 4) {
	   return; // Too short to be a real message
    }

    // Extract the 4 headers
    _rxHeaderTo    = _buf[0];
    _rxHeaderFrom  = _buf[1];
    _rxHeaderId    = _buf[2];
    _rxHeaderFlags = _buf[3];

    if (_promiscuous ||
	    _rxHeaderTo == _thisAddress ||
	    _rxHeaderTo == RH_BROADCAST_ADDRESS) {
	   _rxGood++;
	   _rxBufValid = true;
    }
}


bool RH_LoRa::available()
{
    bool rxBufValid;    

#if (RH_PLATFORM == RH_PLATFORM_RPI)
piLock(0);
#endif
    if (_mode == RHModeTx)
	   return false;

    setModeRx();

    rxBufValid = _rxBufValid;

#if (RH_PLATFORM == RH_PLATFORM_RPI)
piUnlock(0);
#endif

    return rxBufValid; // Will be set by the interrupt handler when a good message is received
}


void RH_LoRa::clearRxBuf()
{
    ATOMIC_BLOCK_START;
    _rxBufValid = false;
    _bufLen = 0;
    ATOMIC_BLOCK_END;
}


bool RH_LoRa::recv(uint8_t* buf, uint8_t* len)
{
    if (!available()) {
	    return false;
    }

    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-RH_LORA_HEADER_LEN)
	    *len = _bufLen-RH_LORA_HEADER_LEN;
	memcpy(buf, _buf+RH_LORA_HEADER_LEN, *len);
	ATOMIC_BLOCK_END;
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

bool RH_LoRa::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_LORA_MAX_MESSAGE_LEN) {
	   return false;
    }

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();
    explicitHeaderMode();

    // Position at the beginning of the FIFO
    spiWrite(REG_FIFO_ADDR_PTR, 0);

    // Write out the headers
    spiWrite(REG_FIFO, _txHeaderTo);
    spiWrite(REG_FIFO, _txHeaderFrom);
    spiWrite(REG_FIFO, _txHeaderId);
    spiWrite(REG_FIFO, _txHeaderFlags);

    // Write the actual message data
    spiBurstWrite(REG_FIFO, data, len);

    // Set the total payload length
    spiWrite(REG_PAYLOAD_LENGTH, len + RH_LORA_HEADER_LEN);

    setModeTx(); // Start the transmitter

    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}


uint8_t RH_LoRa::maxMessageLength()
{
    return RH_LORA_MAX_MESSAGE_LEN;
}


void RH_LoRa::setModeIdle()
{
    if (_mode != RHModeIdle)
    {
	spiWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
	_mode = RHModeIdle;
    }
}


void RH_LoRa::setModeSleep()
{
    if (_mode != RHModeSleep) {
	   spiWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
	   _mode = RHModeSleep;
    }
}


bool RH_LoRa::sleep() { 
   setModeSleep(); 
   return true; 
}


void RH_LoRa::setModeRx()
{
    if (_mode != RHModeRx) {
      explicitHeaderMode();
   	  spiWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
	  spiWrite(REG_DIO_MAPPING_1, 0x00); // Interrupt on RxDone
	  _mode = RHModeRx;
    }
}


void RH_LoRa::setModeTx()
{
    if (_mode != RHModeTx) {
	   spiWrite(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
	   spiWrite(REG_DIO_MAPPING_1, 0x40); // Interrupt on TxDone
	   _mode = RHModeTx;
    }
}


int RH_LoRa::packetRssi()
{
  return (spiRead(REG_PKT_RSSI_VALUE) - (_frequency < 868E6 ? 164 : 157));
}

float RH_LoRa::packetSnr()
{
  return ((int8_t)spiRead(REG_PKT_SNR_VALUE)) * 0.25;
}


void RH_LoRa::setTxPower(int8_t power, int8_t outputPin)
{
	  uint8_t paConfig = 0;
	  uint8_t paDac = 0;

	  paConfig = spiRead( REG_PA_CONFIG );
	  paDac = spiRead( REG_PaDac );

	  paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | outputPin;
	  paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

	  if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
	  {
	    if( power > 17 )
	    {
	      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
	    }
	    else
	    {
	      paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
	    }
	    if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
	    {
	      if( power < 5 )
	      {
	        power = 5;
	      }
	      if( power > 20 )
	      {
	        power = 20;
	      }
	      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
	    }
	    else
	    {
	      if( power < 2 )
	      {
	        power = 2;
	      }
	      if( power > 17 )
	      {
	        power = 17;
	      }
	      paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
	    }
	  }
	  else
	  {
	    if( power < -1 )
	    {
	      power = -1;
	    }
	    if( power > 14 )
	    {
	      power = 14;
	    }
	    paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
	  }
	  spiWrite( REG_PA_CONFIG, paConfig );
	  spiWrite( REG_PaDac, paDac );
}

void RH_LoRa::setTxPowerMax(int level)
{
	if (level < 5)		{
		level = 5;
	}
	else if(level > 20)	{
		level = 20;
	}
	spiWrite(REG_LR_OCP,0x3f);
	spiWrite(REG_PaDac,0x87);//Open PA_BOOST
	spiWrite(REG_PA_CONFIG, RF_PACONFIG_PASELECT_PABOOST | (level - 5));
}

bool RH_LoRa::setFrequency(long frequency)
{
  _frequency = frequency;
  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  spiWrite(REG_FRF_MSB, (uint8_t)(frf >> 16));
  spiWrite(REG_FRF_MID, (uint8_t)(frf >> 8));
  spiWrite(REG_FRF_LSB, (uint8_t)(frf >> 0));

  return true;
}

void RH_LoRa::setSpreadingFactor(int sf)
{
  if (sf < 6) {
  	sf = 6; 
	}
  else if (sf > 12) {
  	sf = 12; 
  	}
  if (sf == 6) {
    spiWrite(REG_DETECTION_OPTIMIZE, 0xc5);
    spiWrite(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    spiWrite(REG_DETECTION_OPTIMIZE, 0xc3);
    spiWrite(REG_DETECTION_THRESHOLD, 0x0a);
  }
  spiWrite(REG_MODEM_CONFIG_2, (spiRead(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void RH_LoRa::setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) { bw = 0; }
  else if (sbw <= 10.4E3) { bw = 1; }
  else if (sbw <= 15.6E3) { bw = 2; }
  else if (sbw <= 20.8E3) { bw = 3; }
  else if (sbw <= 31.25E3) { bw = 4; }
  else if (sbw <= 41.7E3) { bw = 5; }
  else if (sbw <= 62.5E3) { bw = 6; }
  else if (sbw <= 125E3) { bw = 7; }
  else if (sbw <= 250E3) { bw = 8; }
  else /*if (sbw <= 250E3)*/ { bw = 9; }
  spiWrite(REG_MODEM_CONFIG_1,(spiRead(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void RH_LoRa::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }
  int cr = denominator - 4;
  spiWrite(REG_MODEM_CONFIG_1, (spiRead(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void RH_LoRa::setPreambleLength(long length)
{
  spiWrite(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  spiWrite(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void RH_LoRa::setSyncWord(int sw)
{
  spiWrite(REG_SYNC_WORD, sw);
}

void RH_LoRa::enableCrc()
{
  spiWrite(REG_MODEM_CONFIG_2, spiRead(REG_MODEM_CONFIG_2) | 0x04);
}

void RH_LoRa::disableCrc()
{
  spiWrite(REG_MODEM_CONFIG_2, spiRead(REG_MODEM_CONFIG_2) & 0xfb);
}

void RH_LoRa::explicitHeaderMode()
{
  spiWrite(REG_MODEM_CONFIG_1, spiRead(REG_MODEM_CONFIG_1) & 0xfe);
}
