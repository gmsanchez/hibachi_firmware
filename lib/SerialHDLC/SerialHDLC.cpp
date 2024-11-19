
// #include <util/crc16.h>
#include "crc16.h"
#include "SerialHDLC.h"

#define HDLC_FRAME_BOUNDRY_FLAG 0x7E
#define HDLC_ESCAPE_FLAG        0x7D
#define HDLC_ESCAPE_XOR         0x20

#define HDLC_RESPONSE_ACK       0xA0
#define HDLC_RESPONSE_NACK      0xA1
#define HDLC_CRC_ERROR          0xE1
#define HDLC_BUFFER_OVERFLOW    0xE2

#ifdef CRC_CCITT_RFC1171
  #define HDLC_CRC_INIT_VALUE 0xFFFF
  #define CRC_UPDATE          _crc_ccitt_update
#else
  #ifdef CRC_CCITT_XMODEM
    #define HDLC_CRC_INIT_VALUE 0x0000
    #define CRC_UPDATE          _crc_xmodem_update
  #else
    #error CRC format not defined
  #endif
#endif

SerialHDLC::SerialHDLC(serial_hdlc_frame_handler frameHandler,
                       uint16_t maxFrameLength)
    : _frameHandler(frameHandler),
      _frameBuffer(NULL),
      _frameOffset(0),
      _frameChecksum(HDLC_CRC_INIT_VALUE),
      _maxFrameLength(maxFrameLength),
      _escapeByte(false) {
    if (maxFrameLength > 0) _frameBuffer = (uint8_t*)malloc(maxFrameLength + 1);
}

SerialHDLC::~SerialHDLC() {
    if (_frameBuffer != NULL) free(_frameBuffer);
}

void SerialHDLC::read() {
    while (Serial.available() > 0) {
        uint8_t data = (uint8_t)Serial.read();
        readByte(data);
    }
}

void SerialHDLC::readByte(uint8_t data) {
    if (data == HDLC_FRAME_BOUNDRY_FLAG) {
        if (_escapeByte) {
            _escapeByte = false;
        } else if (_frameOffset >= 2 &&
                   _frameChecksum == ((_frameBuffer[_frameOffset - 1] << 8) |
                                      _frameBuffer[_frameOffset - 2])) {
            sendACK();
            if (_frameHandler != NULL) {
                (*_frameHandler)(_frameBuffer, _frameOffset - 2);
            }
        } else if (_frameOffset >= 2) {
            sendNACK(HDLC_CRC_ERROR);
        }
        _frameOffset = 0;
        _frameChecksum = HDLC_CRC_INIT_VALUE;
        return;
    }

    if (_escapeByte) {
        _escapeByte = false;
        data ^= HDLC_ESCAPE_XOR;
    } else if (data == HDLC_ESCAPE_FLAG) {
        _escapeByte = true;
        return;
    }

    _frameBuffer[_frameOffset] = data;
    if (_frameOffset >= 2) {
        _frameChecksum =
            CRC_UPDATE(_frameChecksum, _frameBuffer[_frameOffset - 2]);
    }
    _frameOffset++;

    if (_frameOffset >= _maxFrameLength) {
        _frameOffset = 0;
        _frameChecksum = HDLC_CRC_INIT_VALUE;
        sendNACK(HDLC_BUFFER_OVERFLOW);
    }
}

void SerialHDLC::write(const uint8_t* frameBuffer,
                       const uint16_t frameLength) const {
    // Write the frame boundry flag
    Serial.write(HDLC_FRAME_BOUNDRY_FLAG);

    // Write the frame
    uint16_t checksum = HDLC_CRC_INIT_VALUE;
    for (uint16_t i = 0; i < frameLength; i++) {
        checksum = CRC_UPDATE(checksum, frameBuffer[i]);
        writeByte(frameBuffer[i]);
    }

    // Write the checksum
    writeByte(checksum & 0xFF);
    writeByte((checksum >> 8) & 0xFF);

    // Write the frame boundry flag
    Serial.write(HDLC_FRAME_BOUNDRY_FLAG);
}

void SerialHDLC::writeByte(uint8_t data) const {
    if ((data == HDLC_ESCAPE_FLAG) || (data == HDLC_FRAME_BOUNDRY_FLAG)) {
        Serial.write(HDLC_ESCAPE_FLAG);
        data ^= HDLC_ESCAPE_XOR;
    }
    Serial.write(data);
}

void SerialHDLC::sendACK(void) const {
    uint8_t data = HDLC_RESPONSE_ACK;
    write(&data, 1);
}

void SerialHDLC::sendNACK(const uint8_t code) const {
    uint8_t buffer[2];
    buffer[0] = HDLC_RESPONSE_NACK;
    buffer[1] = code;
    write(buffer, 2);
}
