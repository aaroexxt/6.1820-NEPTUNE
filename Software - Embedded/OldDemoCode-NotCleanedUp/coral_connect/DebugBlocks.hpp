//////////////////////////////////////////////////////////////////////
//
// Ultrasonic Hearing - Enabling you to extend your senses into
// the ultrasonic range using a Teensy 4.1
// Copyright (C) 2021  Maximilian Wagenbach
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//////////////////////////////////////////////////////////////////////


#ifndef DEBUGBLOCKS_HPP
#define DEBUGBLOCKS_HPP

#include <Arduino.h>
#include <AudioStream.h>


class Counter : public AudioStream
{
public:
    Counter() : AudioStream(0, NULL),
    m_counter{1}
    {}

    void update(void){
        // allocate output block
        audio_block_t *output_block;
        output_block = allocate();
        if (!output_block) return;

        // fill buffer
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            output_block->data[i] = m_counter;
        }
        m_counter++;

        transmit(output_block, 0);
        release(output_block);
    }
private:
    uint16_t m_counter;
};


class Printer : public AudioStream
{
public:
    Printer() : AudioStream(1, inputQueueArray)
    {}

    void update(void){
        // get input block
        audio_block_t *input_block;
        input_block = receiveReadOnly();
        if (!input_block) return;

        // fill buffer
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
            Serial.print(input_block->data[i]);
            Serial.print(", ");
        }
        Serial.println();
        release(input_block);
    }
private:
    audio_block_t *inputQueueArray[1];
};

#endif // DEBUGBLOCKS_HPP