
#ifndef ___ARDUINO_SERIAL_HDLC_H___
#define ___ARDUINO_SERIAL_HDLC_H___

#include <Arduino.h>
#include <inttypes.h>

#define HDLC_FRAME_MAX_SIZE 128

#define CRC_CCITT_RFC1171

typedef void (*serial_hdlc_frame_handler)(const uint8_t* frameBuffer,
                                          const uint16_t frameLength);

class SerialHDLC {
   public:
    SerialHDLC(serial_hdlc_frame_handler frameHandler, uint16_t maxFrameLength);
    ~SerialHDLC();

   public:
    void read();
    void write(const uint8_t* frameBuffer, const uint16_t frameLength) const;

   private:
    void readByte(uint8_t data);
    void writeByte(uint8_t data) const;

    void sendACK(void) const;
    void sendNACK(const uint8_t code) const;

   private:
    serial_hdlc_frame_handler _frameHandler;
    uint8_t* _frameBuffer;
    uint16_t _frameOffset;
    uint16_t _frameChecksum;
    uint16_t _maxFrameLength;
    bool _escapeByte;
};

#endif  // ___ARDUINO_SERIAL_HDLC_H___
