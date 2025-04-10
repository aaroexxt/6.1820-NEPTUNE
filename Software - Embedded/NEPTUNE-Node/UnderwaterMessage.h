// UnderwaterMessage.h
#ifndef UNDERWATER_MESSAGE_H
#define UNDERWATER_MESSAGE_H

#include <Arduino.h>


/*
MESSAGE PROTOCOL DESCRIPTION

Total data bits: 16
Total error correcting bits: 21 (hamming code)

| ID (4 bits) | TYPE (2 bits) | MSG (10 bits) |

ID = 0 to 15, the sensor node you want to target.
TYPE = Command to send back & forth
TYPE 0: LED TOGGLE
TYPE 1: AVG GYRO DATA (averages all axes)
TYPE 2: AVG ACCEL DATA (averages all axes)
TYPE 3: AVG TEMP DATA (averages all axes)

*/

class UnderwaterMessage {
public:
    // Bit fields for the payload:
    // 4 bits for device ID, 2 bits for message type, 10 bits for the message
    uint16_t id : 4;   // Device ID [0-15]
    uint16_t type : 2;   // Message type [0-3]
    uint16_t msg : 10;  // Message value [0-1023]
    
    static const uint8_t DATA_BITS = 16; // Total data bits
    static const uint8_t HAMMING_BITS = 21; // Total bits after adding 5 parity bits

    // Constructors (0, explicit)
    UnderwaterMessage() : id(0), type(0), msg(0) {}
    UnderwaterMessage(uint16_t _id, uint16_t _type, uint16_t _msg): id(_id & 0xF), type(_type & 0x3), msg(_msg & 0x3FF) {}

    // Combines the fields into a 16-bit integer:
    // bits [15:12] = id, [11:10] = type, [9:0] = msg
    uint16_t getData() const {
        // ooo fancy bit shifts
        // 3FF = 11 1111 1111 (only pass 10 bits)
        return ((id & 0xF) << 12) | ((type & 0x3) << 10) | (msg & 0x3FF);
    }

    // generic getter functions
    uint16_t getID() const {
        return id & 0xF;
    }

    uint16_t getType() const {
        return type & 0x3;
    }

    uint16_t getMsg() const {
        return msg & 0x3FF;
    }

    // Creates an UnderwaterMessage from a 16-bit data value
    static UnderwaterMessage fromData(uint16_t data) {
        UnderwaterMessage um;
        um.id   = (data >> 12) & 0xF;
        um.type = (data >> 10) & 0x3;
        um.msg  = data & 0x3FF;
        return um;
    }

    // Encode the 16-bit data into a 21-bit Hamming(21,16) code.
    // The returned uint32_t will have the 21 code bits in its lower bits.
    uint32_t encodeHamming() const {
        uint16_t data = getData();
        uint32_t encoded = 0;
        int dataPos = 0;
        // Place data bits into positions that are not powers of 2 (1-indexed)
        for (int i = 1; i <= HAMMING_BITS; i++) {
            if ((i & (i - 1)) != 0) { // i is not a power of 2, so it's a data bit
                if ((data >> dataPos) & 1) {
                    encoded |= (1UL << (i - 1));
                }
                dataPos++;
            }
        }
        // Compute parity bits for positions 1, 2, 4, 8, 16 (1-indexed)
        for (int i = 0; i < 5; i++) {
            int p = 1 << i;  // Parity bit position in 1-indexed numbering (1,2,4,8,16)
            int parity = 0;
            for (int j = 1; j <= HAMMING_BITS; j++) {
                if (j & p) { // If bit position j contributes to parity bit at position p
                    parity ^= (encoded >> (j - 1)) & 1;
                }
            }
            if (parity) {
                encoded |= (1UL << (p - 1)); // Set parity bit at position p
            }
        }
        return encoded;
    }

    // Decode a 21-bit Hamming code (packed into a 32-bit int) and return the corrected UnderwaterMessage.
    // If an error was detected and corrected, *errorPos will hold the 1-indexed bit position (0 if no error).
    static UnderwaterMessage decodeHamming(uint32_t encoded, int *errorPos = nullptr) {
        int syndrome = 0;
        // Recompute the parity bits to calculate the syndrome
        for (int i = 0; i < 5; i++) {
            int p = 1 << i; // parity bit position (1-indexed)
            int parity = 0;
            for (int j = 1; j <= HAMMING_BITS; j++) {
                if (j & p) {
                    parity ^= (encoded >> (j - 1)) & 1;
                }
            }
            if (parity) {
                syndrome |= p;
            }
        }
        // If syndrome is non-zero and within our message length, correct the bit.
        if (syndrome != 0 && syndrome <= HAMMING_BITS) {
            encoded ^= (1UL << (syndrome - 1));
        }
        if (errorPos) {
            *errorPos = syndrome; // 0 if no error, or the bit position that was corrected
        }
        // Extract the original 16 data bits from the encoded 21-bit message.
        uint16_t data = 0;
        int dataPos = 0;
        for (int i = 1; i <= HAMMING_BITS; i++) {
            if ((i & (i - 1)) != 0) { // Not a parity position
                if ((encoded >> (i - 1)) & 1) {
                    data |= (1 << dataPos);
                }
                dataPos++;
            }
        }
        return fromData(data);
    }
};

#endif // UNDERWATER_MESSAGE_H
