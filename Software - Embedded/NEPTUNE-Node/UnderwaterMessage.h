// UnderwaterMessage.h
#ifndef UNDERWATER_MESSAGE_H
#define UNDERWATER_MESSAGE_H

#include <Arduino.h>


/*
MESSAGE PROTOCOL DESCRIPTION

Total data bits: 10

| ID (2 bits) | MSG (8 bits) | 4 bits hamming parity + 1 overall parity | (total 15 bits)

Transmitter sends 2 bit ID of node it wants, then message type it's requesting (8 bits), then hamming parity bits, then overall parity bit (for dual error detection)
Receiver responds with its own ID, then data (8 bits), then hamming parity bits, then overall parity bit

*/


class UnderwaterMessage {
public:
    static const uint8_t DATA_BITS = 10;   // Total data bits (2 + 8)
    static const uint8_t HAMMING_BITS = 15;  // Total bits after adding 4 parity bits (Hamming(14,10)) + 1 overall parity bit

    // Bit fields for the payload:
    uint16_t id : 2;   // Device ID [0-3]
    uint16_t msg : 8;  // Message value [0-255]
    static const uint16_t size = HAMMING_BITS; // total size

    // Constructors
    UnderwaterMessage() : id(0), msg(0) {}
    UnderwaterMessage(uint16_t _id, uint16_t _msg) : id(_id & 0x3), msg(_msg & 0xFF) {}

    // Combines the fields into a 10-bit integer:
    // Bits [9:8] represent the 2-bit id and bits [7:0] represent the 8-bit msg.
    uint16_t getData() const {
        return ((id & 0x3) << 8) | (msg & 0xFF);
    }

    // Generic getter functions
    uint16_t getID() const {
        return id & 0x3;
    }
    
    uint16_t getMsg() const {
        return msg & 0xFF;
    }

    // Creates an UnderwaterMessage from a 10-bit data value.
    static UnderwaterMessage fromData(uint16_t data) {
        UnderwaterMessage um;
        um.id  = (data >> 8) & 0x3;
        um.msg = data & 0xFF;
        return um;
    }

    // Encode the 10-bit data into a 14-bit Hamming(14,10) code + 1 extra overall parity bit.
    // Returns a uint16_t with the 15 total bits in the lower bits.
    uint16_t encodeHamming() const {
        uint16_t data = getData();
        uint16_t encoded = 0;
        int dataPos = 0;

        // Place data bits into positions that are not powers of 2 (using 1-indexed positions).
        // In a 14-bit hamming code, positions 1, 2, 4, and 8 are reserved for parity bits.
        for (int i = 1; i <= HAMMING_BITS; i++) {
            if ((i & (i - 1)) != 0) { // i is not a power of 2, so it is a data bit position.
                if ((data >> dataPos) & 1) {
                    encoded |= (1U << (i - 1));
                }
                dataPos++;
            }
        }

        // Compute parity bits for positions 1, 2, 4, and 8 (1-indexed).
        for (int i = 0; i < 4; i++) {
            int p = 1 << i;  // Parity bit position: 1, 2, 4, or 8.
            int parity = 0;
            for (int j = 1; j <= HAMMING_BITS; j++) {
                if (j & p) { // If bit position j contributes to parity bit at position p.
                    parity ^= (encoded >> (j - 1)) & 1;
                }
            }
            if (parity) {
                encoded |= (1U << (p - 1)); // Set the parity bit at position p.
            }
        }

        // Compute overall parity for SECDED (bit 15, position 14 in 0-indexed)
        uint8_t overall = 0;
        for (int i = 0; i < 14; ++i) {
            overall ^= (encoded >> i) & 1;
        }
        if (overall) {
            encoded |= (1U << 14);
        }
 
        return encoded;
    }

    // Decode 15-bit encoded message with SECDED
    static UnderwaterMessage decodeHamming(uint16_t encoded, int *errorPos = nullptr, bool *doubleBit = nullptr) {
        int syndrome = 0;

        // Recompute syndrome from parity bits
        for (int i = 0; i < 4; ++i) {
            int p = 1 << i;
            int parity = 0;
            for (int j = 1; j <= 14; ++j) {
                if (j & p) {
                    parity ^= (encoded >> (j - 1)) & 1;
                }
            }
            if (parity) syndrome |= p;
        }

        // Check overall parity
        uint8_t overall = 0;
        for (int i = 0; i < 15; ++i) {
            overall ^= (encoded >> i) & 1;
        }

        // Handle error cases
        if (syndrome == 0 && overall) {
            // Double-bit error detected
            if (doubleBit) *doubleBit = true;
            if (errorPos) *errorPos = -2;
            return UnderwaterMessage();
        }

        if (syndrome != 0) {
            if (overall == 0) {
                // Conflicting parity: likely double-bit error
                if (doubleBit) *doubleBit = true;
                if (errorPos) *errorPos = -1;
                return UnderwaterMessage();
            } else {
                // Single-bit error, correct it
                encoded ^= (1U << (syndrome - 1));
                if (errorPos) *errorPos = syndrome;
            }
        } else {
            if (errorPos) *errorPos = 0; // no error
        }

        // Extract original 10 data bits
        uint16_t data = 0;
        int dataPos = 0;
        for (int i = 1; i <= 14; ++i) {
            if ((i & (i - 1)) != 0) {
                if ((encoded >> (i - 1)) & 1) {
                    data |= (1U << dataPos);
                }
                dataPos++;
            }
        }

        return fromData(data);
    }
};

#endif // UNDERWATER_MESSAGE_H
