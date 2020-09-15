// RH_LoRa.h
//
// Definitions for SX1276 based radios in LoRa mode. 
// This code is a mixture of the original RH_RF95 RadioHead code
// as well as LoRa code found in ESP32 based LoRa code for Heltec's WiFi Lora 32 V2 MCU board.
// This code allows for the Dragino HAT to work, unmodified, on Raspian Buster and beyond,
// as well as communicate with Heltec Lora 32 boards.
// 
#ifndef RH_LORA_h
#define RH_LORA_h

#include "RHSPIDriver.h"

#if (RH_PLATFORM == RH_PLATFORM_RPI)
// Dragino Raspberry Pi LoRa HAT settings
// using WiringPi numbering scheme...
#define LORA_DEFAULT_SS_PIN     6
#define LORA_DEFAULT_RESET_PIN  0
#define LORA_DEFAULT_DIO0_PIN   7
#else
// Heltec LoRa settings...
#define LORA_DEFAULT_SS_PIN 18
#define LORA_DEFAULT_RESET_PIN  14
#define LORA_DEFAULT_DIO0_PIN   26
#endif


// Max number of octets the LORA Rx/Tx FIFO can hold
#define RH_LORA_FIFO_SIZE 255

// This is the maximum number of bytes that can be carried by the LORA.
// We use some for headers, keeping fewer for RadioHead messages
#define RH_LORA_MAX_PAYLOAD_LEN RH_LORA_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the LORA's payload
#define RH_LORA_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#ifndef RH_LORA_MAX_MESSAGE_LEN
 #define RH_LORA_MAX_MESSAGE_LEN (RH_LORA_MAX_PAYLOAD_LEN - RH_LORA_HEADER_LEN)
#endif


/*!
 * RegPaConfig
 */
#define RF_PACONFIG_PASELECT_MASK                   0x7F
#define RF_PACONFIG_PASELECT_PABOOST                0x80
#define RF_PACONFIG_PASELECT_RFO                    0x00 // Default

#define RF_PACONFIG_MAX_POWER_MASK                  0x8F

#define RF_PACONFIG_OUTPUTPOWER_MASK                0xF0

/*!
 * RegPaDac
 */
#define RF_PADAC_20DBM_MASK                         0xF8
#define RF_PADAC_20DBM_ON                           0x07
#define RF_PADAC_20DBM_OFF                          0x04  // Default


/////////////////////////////////////////////////////////////////////
/// \class RH_LoRa RH_LoRa.h
/// \brief Driver to send and receive unaddressed, unreliable datagrams via a LoRa 
/// capable radio transceiver.
///
/// For Semtech SX1276 chip found in Heltec ESP32 LoRa V2 and Dragino LoRa HAT
/// This class has been tested with an unmodified Dragino Lora HAT on a Raspberry Pi 3
/// with Raspian Buster as the OS. It has also been tested with a Heltec Wifi Lora 32 V2
/// board.
/// Getting the Semtech SX1276 chip to work on these boards require a functioning SPI
/// system that allows the "Slave select" pin (aka SS, NSS, Chip Select, CS) to be
/// specified, along with the pin connected to the SX1276 DIO0 pin and the pin
/// connected to the SX1276 reset pin.  It is important to note that the SS pin
/// on the Dragino Lora HAT (even at board version 1.4) is NOT connected to the standard
/// pin used by Raspian's kernel SPI implementation. That is the primary reason this
/// driver was written (vs. a hardware hack to get this to work with RH_RF95 driver).
///
/// For Raspberry Pi, there are THREE types of pin numbering systems floating around:
///   1. The pin system used by the Bcm2835 library
///   2. The pin system used by the WiringPi library
///   3. The actual pin numbers found on the GPIO header board
/// This class uses the WiringPi library for its implementation, but WiringPi can be
/// initialized with any one of the above systems, depending on which "setup" method is
/// called.  See http://wiringpi.com/reference/setup/ for details.
/// 
/// For reference, here are pin numbers used by both the Dragino Hat and the Heltec MCU
/// when connected to their respective SX1276 chips:
///
/// Dragino LoRa HAT to RPi pin wiring:
/// http://www.dragino.com/downloads/downloads/LoRa-GPS-HAT/LoRa_GPS_HAT_UserManual_v1.0.pdf
/// BCM25/wPi6 (22) = Lora NSS (SPI slave select)
/// BCM17/wPi0 (11) = Lora Reset
/// BCM4/wPi7 (7) = Lora DIO0 (aka Interupt Pin in RadioHead's RH_RF95.h)
/// BCM9/wPi13 (21) = Lora/RPiSPI0 MISO
/// BCM10/wPi12 (19) = Lora/RPiSPI0 MOSI
/// BCM11/wPi14 (23) = Lora/RPiSPI0 SCLK
/// BCM23/wPi4 (16) = DIO1
/// BCM24/wPi5 (18) = DIO2
/// BCM18/wPi1 (12) = 1PPS
/// 
/// NSS is SPI abbreviation for "Slave Select" pin. Thus above, that pin is 25 in BCM numbering, 6 in
/// WiringPi number, and physical pin 22 on the Dragino Board/RasPi Header pins.
/// 
/// DIO = "Digital I/O" pin from LoRa chip. They can be assigned different functions via software.
/// This class uses DIO0 as an interrupt pin for Tx/Rx done
/// 
/// Heltec LoRa pin connections
/// (19) = Lora_MISO
/// (27) = Lora_MOSI
///  (5) = Lora_SCLK
/// (14) = Lora reset
/// (18) = Lora NSS (Slave select)
/// (26) = Lora DIO0
/// (35) = Lora DIO1
/// (34) = Lora DIO2
/// 

class RH_LoRa : public RHSPIDriver
{
public:
    RH_LoRa(uint8_t slaveSelectPin=LORA_DEFAULT_SS_PIN, uint8_t DIO0Pin=LORA_DEFAULT_DIO0_PIN, uint8_t resetPin=LORA_DEFAULT_RESET_PIN, RHGenericSPI& spi = hardware_spi);


    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool    init();

    /// Tests whether a new message is available
    /// from the Driver. 
    /// On most drivers, this will also put the Driver into RHModeRx mode until
    /// a message is actually received by the transport, when it wil be returned to RHModeIdle.
    /// This can be called multiple times in a timeout loop
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv()
    virtual bool    available();

    /// Turns the receiver on if it not already on.
    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    virtual bool    recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted. 
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send
    /// \return true if the message length was valid and it was correctly queued for transmit
    virtual bool    send(const uint8_t* data, uint8_t len);


    /// Returns the maximum message length 
    /// available in this Driver.
    /// \return The maximum legal message length
    virtual uint8_t maxMessageLength();


    /// If current mode is Rx or Tx changes it to Idle. If the transmitter or receiver is running, 
    /// disables them.
    void setModeIdle();

    /// If current mode is Tx or Idle, changes it to Rx. 
    /// Starts the receiver in the RF95/96/97/98.
    void setModeRx();

    /// If current mode is Rx or Idle, changes it to Rx. F
    /// Starts the transmitter in the RF95/96/97/98.
    void setModeTx();

    void setModeSleep();
    virtual bool sleep() override; // From RHGenericDriver


    // From Heltec LoRa code...
    void setTxPower(int8_t power, int8_t outputPin);
    void setTxPowerMax(int level);
    bool setFrequency(long frequency);
    void setSpreadingFactor(int sf);
    void setSignalBandwidth(long sbw);
    void setCodingRate4(int denominator);
    void setPreambleLength(long length);
    void setSyncWord(int sw);
    void enableCrc();
    void disableCrc();
    int packetRssi();
    float packetSnr();

protected:
    void explicitHeaderMode();
    void implicitHeaderMode();
    int beginPacket(int implicitHeader = false);
    int endPacket();
    int parsePacket(int size = 0);

    /// This is a low level function to handle the interrupts for one instance of RH_RF95.
    /// Called automatically by isr*()
    /// Should not need to be called by user code.
    void handleInterrupt();

    /// Examine the revceive buffer to determine whether the message is for this node
    void validateRxBuf();

    /// Clear our local receive buffer
    void clearRxBuf();

private:
    /// Low level interrupt service routine for device connected to interrupt 0
    static void         isr0();

    /// Low level interrupt service routine for device connected to interrupt 1
    static void         isr1();

    /// Low level interrupt service routine for device connected to interrupt 2
    static void         isr2();

    /// Array of instances connected to interrupts 0 and 1
    static RH_LoRa*     _deviceForInterrupt[];

    /// Index of next interrupt number to use in _deviceForInterrupt
    static uint8_t      _interruptCount;

    /// The configured interrupt pin connected to this instance
    uint8_t             _dio0;

    /// The index into _deviceForInterrupt[] for this device (if an interrupt is already allocated)
    /// else 0xff
    uint8_t             _myInterruptIndex;

    /// Number of octets in the buffer
    volatile uint8_t    _bufLen;
    
    /// The receiver/transmitter buffer
    uint8_t             _buf[RH_LORA_MAX_PAYLOAD_LEN];

    /// True when there is a valid message in the buffer
    volatile bool       _rxBufValid;

    /// Pin connected to SX1276 reset pin
    uint8_t _reset;

    int _initCalled = 0;

    int _frequency;
    
};

#endif
