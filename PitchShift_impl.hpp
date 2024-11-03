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


#include "PitchShift.hpp"

#include <cmath>
#include <algorithm>


namespace {
    // Returns the remainder of the division x / y
    // NOTE: we can't simply use std::fmod instead, because due to "conventions"
    // for std::fmod, the sign of the result is the sign of the dividend (x),
    // while for modulo the sign of the result is the sign of the divisor (y)...
    float modulo(float x, float y)
    {
        return x - y * std::floor(x / y);
    }


    // Wrap an arbitrary phase in radians into the range ]-pi, pi]
    // Also known as principle argument function
    float wrap_phase(float phase)
    {
        static const float pi = 3.14159265f;
        return modulo(phase + pi, -2.f * pi) + pi;
    }


    // Approximation of the atan2 function loosely based on the book “Approximations for Digital Computers” by Cecil Hastings Jr. from 1955
    // |max_error| < 0.005
    float atan2_approximation(float y, float x)
    {
        static const float pi = 3.14159265f;
        static const float halfPi = 1.5707963f;

        if (x == 0.0f) {
            if (y > 0.0f)
                return halfPi;
            if (y < 0.0f)
                return -halfPi;
            return 0.0f;
        }

        float atan;
        float z = y / x;
        if (std::fabs(z) < 1.0f) {
            atan = z / (1.0f + 0.28f * z * z);
            if (x < 0.0f) {
                if (y < 0.0f)
                    return atan - pi;
                return atan + pi;
            }
        }
        else {
            atan = halfPi - z / (z * z + 0.28f);
            if (y < 0.0f)
                return atan - pi;
        }
        return atan;
    }
}


template <uint16_t FRAME_SIZE>
PitchShift<FRAME_SIZE>::PitchShift(uint32_t sampleRate, float32_t pitchShiftFactor) :
    AudioStream(1, inputQueueArray),
    m_halfSampleRate{static_cast<uint32_t>(std::round(sampleRate / 2.f))},
    m_binFrequencyWidth{static_cast<float32_t>(sampleRate) / FRAME_SIZE},
    m_highPassCutoff{0.f},
    m_startIndex{1},
    m_audibleRangeEndIndex{static_cast<uint16_t>(std::round(22000.f / m_binFrequencyWidth))},
    m_offset{0}
{
    // initialize FFT
#if defined(KINETISK)
    arm_status status = arm_rfft_init_f32(&m_fftInst, &m_fftComplexInst, FRAME_SIZE, 0, 1);
    if (status == ARM_MATH_ARGUMENT_ERROR)
        Serial.println("FFT failed to initilize! FFT size not supported.");

    status = arm_rfft_init_f32(&m_ifftInst, &m_ifftComplexInst, FRAME_SIZE, 1, 1);
    if (status == ARM_MATH_ARGUMENT_ERROR)
        Serial.println("iFFT failed to initilize! iFFT size not supported.");
#elif defined(__IMXRT1062__)
    arm_status status = arm_rfft_fast_init_f32(&m_fftInst, FRAME_SIZE);
    if (status == ARM_MATH_ARGUMENT_ERROR)
        Serial.println("FFT failed to initilize! FFT size not supported.");
#endif
    // set pitch shift factor
    if (pitchShiftFactor <= 0.f)
    {
        Serial.println("Pitch shift factor has to be bigger than 0. Overwritting it to 1.");
        pitchShiftFactor = 1.f;
    }
    m_pitchShiftFactor = pitchShiftFactor;

    // generate window
    generateWindow();

    // initialize buffers
    std::memset(m_inputBuffer, 0, sizeof m_inputBuffer);
    std::memset(m_outputBuffer, 0, sizeof m_outputBuffer);
    std::memset(m_overlapBuffer, 0, sizeof m_overlapBuffer);
    std::memset(m_previousPhases, 0, sizeof m_previousPhases);
    std::memset(m_magnitudes, 0, sizeof m_magnitudes);
    std::memset(m_frequencies, 0, sizeof m_frequencies);
    std::memset(m_synthesisMagnitudes, 0, sizeof m_synthesisMagnitudes);
    std::memset(m_synthesisFrequencies, 0, sizeof m_synthesisFrequencies);
    std::memset(m_phaseSum, 0, sizeof m_phaseSum);
}


template <uint16_t FRAME_SIZE>
void PitchShift<FRAME_SIZE>::generateWindow()
{
    // generate a Hann window with 0 on both ends
    for(int i = 0; i < FRAME_SIZE; i++)
    {
        m_window[i] = 0.5 * (1.0 - std::cos(2.0 * M_PI * (static_cast<double>(i) / (FRAME_SIZE - 1))));
    }
}


template <uint16_t FRAME_SIZE>
void PitchShift<FRAME_SIZE>::setHighPassCutoff(float cutoff)
{
    if (cutoff < 0.f)
    {
        Serial.println("PitchShift highpass cutoff can not be smaller than 0. Overwritting it to 0.");
        cutoff = 0.f;
    }
    else if (cutoff > m_halfSampleRate)
    {
        Serial.printf("PitchShift highpass cutoff can not be bigger than half of the sample rate. Overwritting it to %d.\n", m_halfSampleRate);
        cutoff = m_halfSampleRate;
    }
    m_highPassCutoff = cutoff;

    // the highpass is applied by skipping the lower FFT bins in the analysis and shifting stage
    const uint16_t highpassIndex = std::round(m_highPassCutoff / m_binFrequencyWidth) + 1;
    m_startIndex = std::min(highpassIndex, HALF_FRAME_SIZE);
}


#ifdef UNIT_TEST
    template <uint16_t FRAME_SIZE>
    audio_block_t* PitchShift<FRAME_SIZE>::allocate()
    {
        audio_block_t* block = new audio_block_t;
        std::memset(block->data, 0, sizeof block->data);
        return block;
    }

	template <uint16_t FRAME_SIZE>
    audio_block_t* PitchShift<FRAME_SIZE>::receiveReadOnly()
    {
        Serial.print("inputBlock = [");
        audio_block_t* block = allocate();
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            block->data[i] = m_inputGenerator();
            Serial.print(block->data[i]);
            Serial.print(", ");
        }
        Serial.println("]");

        return block;
    }

    template <uint16_t FRAME_SIZE>
    void PitchShift<FRAME_SIZE>::transmit(audio_block_t *block, unsigned char)
    {
        Serial.print("outputBlock = [");
        for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)
        {
            Serial.print(block->data[i]);
            Serial.print(", ");
        }
        Serial.println("]");
    }

    template <uint16_t FRAME_SIZE>
    void PitchShift<FRAME_SIZE>::release(audio_block_t * block)
    {
        delete block;
    }
#endif


template <uint16_t FRAME_SIZE>
void PitchShift<FRAME_SIZE>::update(void)
{
    // get input block
    audio_block_t* input_block;
    input_block = receiveReadOnly();
    if (!input_block)
        return;

    // allocate output block
    audio_block_t* output_block;
    output_block = allocate();
    if (!output_block) {
        release(input_block);
        return;
    }

#if defined(__ARM_ARCH_7EM__)  // Armv7-M (Cortex-M4) with FPU
    // copy the input block data into the input buffer
    // starting with an offset of FRAME_OVERLAP iteratively filling a chunck of length HOP_SIZE (making overlap & add easier later)
    std::memcpy(m_inputBuffer + m_offset + FRAME_OVERLAP, input_block->data, sizeof(int16_t) * AUDIO_BLOCK_SAMPLES);

    // fill the output block
    std::memcpy(output_block->data, m_outputBuffer + m_offset, sizeof(int16_t) * AUDIO_BLOCK_SAMPLES);

    // send out the output block
    transmit(output_block, 0);
    release(input_block);
    release(output_block);

    // check if we have collected enough new buffers for another FFT
    m_offset += AUDIO_BLOCK_SAMPLES;
    if (m_offset < HOP_SIZE) {
        return;
    }
    m_offset = 0;

    // convert buffer to float
    arm_q15_to_float(m_inputBuffer, m_floatInBuffer, FRAME_SIZE);

    // as preparation for the next FFT, move a chunck of length FRAME_OVERLAP by HOP_SIZE to the beginning of the input buffer
    std::memmove(m_inputBuffer, m_inputBuffer + HOP_SIZE, sizeof(int16_t) * FRAME_OVERLAP);

    // apply the window function
    arm_mult_f32(m_floatInBuffer, m_window, m_floatInBuffer, FRAME_SIZE);

    // do the FFT
    // the FFT output is complex and in the following format
    // {real(0), imag(0), real(1), imag(1), ...}
    // real[0] and imag[0] are both real valued.
    // They are ignored throughout the processing, which is why the loop counters start at 1 unless the highpass increases that starting index
    // real[0] represents the DC offset, and imag[0] should be 0
    // all other value are pairs of complex numbers
#if defined(KINETISK)
    arm_rfft_f32(&m_fftInst, m_floatInBuffer, m_floatComplexBuffer);
#elif defined(__IMXRT1062__)
    arm_rfft_fast_f32(&m_fftInst, m_floatInBuffer, m_floatComplexBuffer, 0);
#endif

    // analyse the lower half of the signal (upper half is the same just mirrored)
    // skipping lower FFT bins as they are not processed later on
    for (int i = m_startIndex; i < HALF_FRAME_SIZE; i++) {
        // deinterlace the FFT result
        float32_t real = m_floatComplexBuffer[i * 2];
        float32_t imag = m_floatComplexBuffer[i * 2 + 1];

        // compute phase and magnitude
        float32_t magnitude = std::sqrt(real * real + imag * imag);  // Note to future optimizers: std::sqrt is faster than arm_sqrt_f32, I measured... (possibly because it uses sqrt hardware instructions better ¯\_(ツ)_/¯)
        float32_t phase = atan2_approximation(imag, real);

        // noise gate to remove continous tones in the output, that result from many bins with little energy accumulating into few bins with high energy
        if (magnitude <= 0.15f)  // 0.05f is the noise floor without a mic, 0.15f was emperically determined in a quiet room
            magnitude = 0.f;
        // TODO: this could be used as an optimization omitting calculations

        // compute phase difference (derivative)
        float32_t frequency = phase - m_previousPhases[i];
        m_previousPhases[i] = phase;

        // subtract the expected phase increment (to get the phase offset of the bin)
        frequency -= static_cast<float32_t>(i) * m_omega;


        // wrap phase into the range ]-pi, pi]
        frequency = wrap_phase(frequency);

        // get deviation from bin frequency from the +/- Pi interval
        // (this takes out the influence of the overlap on the bins phase)
        frequency = OVERSAMPLING_FACTOR * frequency / (2. * M_PI);

        // compute the i-th partials' true frequency
        frequency = static_cast<float32_t>(i) * m_binFrequencyWidth + frequency * m_binFrequencyWidth; // could be rewritten as (i + f) * b

        // save magnitude and true frequency
        m_magnitudes[i] = magnitude;
        m_frequencies[i] = frequency;
    }

    std::memset(m_synthesisMagnitudes, 0, sizeof m_synthesisMagnitudes);
    std::memset(m_synthesisFrequencies, 0, sizeof m_synthesisFrequencies);
    // apply a high pass filter by skipping lower FFT bins
    for (int i = m_startIndex; i < HALF_FRAME_SIZE; i++) {
        // do the actual pitch shifting
        const uint16_t binIndex = std::round(i * m_pitchShiftFactor);
        if (binIndex <= HALF_FRAME_SIZE) {
            m_synthesisMagnitudes[binIndex] += m_magnitudes[i];
            m_synthesisFrequencies[binIndex] = m_frequencies[i] * m_pitchShiftFactor;
        }
    }

    // synthesize the signal
    // as an optimization only the elements in the audible range are calculated
    for (int i = 1; i < m_audibleRangeEndIndex; i++) {
        // get new magnitude and true frequency from the synthesis array
        float32_t magnitude = m_synthesisMagnitudes[i];
        float32_t phase = m_synthesisFrequencies[i];

        // TODO: continue; when magnitude < threshold

        // subtract bin mid frequency
        phase -= static_cast<float32_t>(i) * m_binFrequencyWidth;

        // get bin deviation from freq deviation
        phase /= m_binFrequencyWidth;

        // take oversampling into account
        phase = 2.0 * M_PI * phase / OVERSAMPLING_FACTOR;

        // add the overlap phase advance back in
        phase += static_cast<float32_t>(i) * m_omega;

        // accumulate delta phase to get the bin phase
        // warp phase to avoid float precision issues
        m_phaseSum[i] = wrap_phase(m_phaseSum[i] + phase);

        // compute new real and imaginary part and re-interleave
        // Note: the arm sine & cosine functions use lookup tables and are faster than their std counterpart at the expense of precision (which is sufficient in this case)
        m_floatComplexBuffer[i * 2] = magnitude * arm_cos_f32(m_phaseSum[i]);
        m_floatComplexBuffer[i * 2 + 1] = magnitude * arm_sin_f32(m_phaseSum[i]);
    }

    // zero out all frequencies above the hearing threshold as well as the mirrored second half of the FFT spectrum
    std::memset(m_floatComplexBuffer + (2 * m_audibleRangeEndIndex), 0, sizeof(float32_t) * (FRAME_SIZE + FRAME_SIZE - (2 * m_audibleRangeEndIndex)));

    // do the iFFT
#if defined(KINETISK)
    arm_rfft_f32(&m_ifftInst, m_floatComplexBuffer, m_floatOutBuffer);
#elif defined(__IMXRT1062__)
    arm_rfft_fast_f32(&m_fftInst, m_floatComplexBuffer, m_floatOutBuffer, 1);
#endif

    // apply window function again (because we synthesized the signal from scratch)
    arm_mult_f32(m_floatOutBuffer, m_window, m_floatOutBuffer, FRAME_SIZE);

    // this factor corrects the amplitude change introduced by overlapping window functions
    arm_scale_f32(m_floatOutBuffer, AMPLITUDE_CORRECTION_FACTOR, m_floatOutBuffer, FRAME_SIZE);

    // convert floats back to int
    arm_float_to_q15(m_floatOutBuffer, m_outputBuffer, FRAME_SIZE);

    // add overlap of the previous output to the new output
    arm_add_q15(m_outputBuffer, m_overlapBuffer, m_outputBuffer, FRAME_OVERLAP);

    // save the overlap for the next round
    std::memcpy(m_overlapBuffer, m_outputBuffer + HOP_SIZE, sizeof(int16_t) * FRAME_OVERLAP);

#else
    release(input_block);
    release(output_block);
#endif
}
