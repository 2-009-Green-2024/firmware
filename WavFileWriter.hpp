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


#ifndef WAVFILEWRITER_INCLUDE
#define WAVFILEWRITER_INCLUDE

#include <SD.h>
#include <record_queue.h>

class WavFileWriter
{
public:
    WavFileWriter(AudioRecordQueue& queue);
    bool open(const char *fileName, unsigned int sampleRate, unsigned int channelCount);
    bool isWriting();
    bool update();
    bool close();

private:
    void writeHeader(unsigned int sampleRate, unsigned int channelCount);

    bool                  m_isWriting;
    File                  m_file;
    AudioRecordQueue&     m_queue;
    uint32_t              m_totalBytesWritten;
    uint8_t               m_buffer[512];
};

#endif // WAVFILEWRITER_INCLUDE
