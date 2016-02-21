/*
 * Copyright (c) 2015 Chris Reed
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "argon/argon.h"
#include "audio_output.h"
#include "audio_output_converter.h"
#include "audio_filter.h"
#include "audio_ramp.h"
#include "asr_envelope.h"
#include "sequencer.h"
#include "audio_mixer.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_fxos.h"
#include "arm_math.h"
#include <stdio.h>
#include <math.h>

//------------------------------------------------------------------------------
// Definitions
//------------------------------------------------------------------------------

#define OVER_SAMPLE_RATE (384U)
#define BUFFER_SIZE (256)
#define CHANNEL_NUM (2)
#define BUFFER_NUM (2)

class SineGenerator : public AudioFilter
{
public:
    SineGenerator()
    :   m_delta(0),
        m_sinFreq(0),
        m_phase(0),
        m_env(),
        m_previous(0),
        m_seq(NULL)
    {
    }

    void set_freq(float freq) { m_sinFreq = freq; }
    void set_sequence(Sequencer * seq) { m_seq = seq; }
    void enable_sustain(bool enable) { m_env.enable_sustain(enable); }
    void set_attack(float seconds) { m_env.set_length_in_seconds(ASREnvelope::kAttack, seconds); }
    void set_release(float seconds) { m_env.set_length_in_seconds(ASREnvelope::kRelease, seconds); }
    void init();

    virtual void process(float * samples, uint32_t count);

protected:
    float m_delta;
    float m_sinFreq; // 80 Hz
    float m_phase;
    ASREnvelope m_env;
    float m_previous;
    Sequencer * m_seq;
};

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

void accel_thread(void * arg);
void init_audio_out();
void init_board();

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

uint32_t g_xtal0Freq = 8000000U;
uint32_t g_xtal32Freq = 32768U;

float g_audioBuf[BUFFER_SIZE];
float g_mixBuf[BUFFER_SIZE];
int16_t g_outBuf[BUFFER_NUM][BUFFER_SIZE * CHANNEL_NUM];

const float kSampleRate = 32000.0f; // 32kHz
float g_sinFreq = 80.0f; // 80 Hz
float g_currRadians = 0.0f;

AudioOutput g_audioOut;
AudioOutputConverter g_audioOutConverter;
SineGenerator g_kickGen;
Sequencer g_kickSeq;
SineGenerator g_bassGen;
Sequencer g_bassSeq;
// SineGenerator g_tickGen;
// Sequencer g_tickSeq;
AudioMixer g_mixer;
i2c_master_handle_t g_i2cHandle;
fxos_handle_t g_fxos;

Ar::ThreadWithStack<512> g_accelThread("accel", accel_thread, 0, 120, kArSuspendThread);

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

void my_timer_fn(Ar::Timer * t, void * arg)
{
//     printf("x\n");

    GPIO_TogglePinsOutput(GPIOA, (1 << 1)|(1 << 2));
    GPIO_TogglePinsOutput(GPIOD, (1 << 5));
}
Ar::Timer g_myTimer("mine", my_timer_fn, 0, kArPeriodicTimer, 1500);

void SineGenerator::init()
{
    m_delta = 2.0f * PI * (m_sinFreq / m_sampleRate);

    m_env.set_sample_rate(get_sample_rate());
    m_env.set_curve_type(ASREnvelope::kAttack, AudioRamp::kLinear);
    m_env.set_curve_type(ASREnvelope::kRelease, AudioRamp::kLinear);
    m_env.set_peak(1.0f);
//     m_env.set_length_in_seconds(ASREnvelope::kAttack, 0.01f);
//     m_env.set_length_in_seconds(ASREnvelope::kRelease, 1.0f);
}

void SineGenerator::process(float * samples, uint32_t count)
{
    // Check for a trigger.
    Sequencer::Event triggerEvent = m_seq->get_next_event(count);
    int32_t triggerSample = triggerEvent.m_timestamp;
    if (triggerEvent.m_event == Sequencer::kNoteStopEvent)
    {
        m_env.set_release_offset(triggerSample);
        triggerSample = -1;
    }
    bool needsRestartOnZeroCrossing = false;
    float previous = m_previous;
    float * sample = samples;
    int i;
    for (i = 0; i < count; ++i)
    {
        // Detect trigger point.
        if (triggerSample == i)
        {
            needsRestartOnZeroCrossing = true;
        }

        float f = arm_sin_f32(m_phase);
        float v = f * m_env.next();
        *sample++ = v;

        // After triggered, restart  on a zero crossing to prevent popping.
        bool zeroCrossing = ((previous >= 0.0f && v <= 0.0f) || (previous <= 0.0f && v >= 0.0f));
        if (needsRestartOnZeroCrossing && zeroCrossing)
        {
            m_env.trigger();
            m_phase = 0.0f;
            needsRestartOnZeroCrossing = false;
        }

        m_phase += m_delta;
        if (m_phase >= 2.0f * PI)
        {
            m_phase = 0.0f;
        }

        previous = v;
    }

    m_previous = previous;
}

void accel_thread(void * arg)
{
    memset(&g_fxos, 0, sizeof(g_fxos));
    g_fxos.base = I2C0;
    g_fxos.i2cHandle = &g_i2cHandle;
    g_fxos.xfer.slaveAddress = 0x1c;
    FXOS_Init(&g_fxos);

    while (1)
    {
        fxos_data_t data;
        status_t status = FXOS_ReadSensorData(&g_fxos, &data);
        if (status == kStatus_Success)
        {
//             printf("acc[x=%6d y=%6d z=%6d] mag[x=%6d y=%6d z=%6d]\r\n",
//                    data.accelX, data.accelY, data.accelZ,
//                    data.magX, data.magY, data.magZ);
        }

        Ar::Thread::sleep(20);
    }
}

void init_audio_out()
{
    // Configure the audio format
    sai_transfer_format_t format;
    format.bitWidth = kSAI_WordWidth16bits;
    format.channel = 0U;
    format.sampleRate_Hz = kSAI_SampleRate32KHz;
    format.masterClockHz = OVER_SAMPLE_RATE * format.sampleRate_Hz;
    format.protocol = kSAI_BusLeftJustified;
    format.stereo = kSAI_Stereo;
    format.watermark = FSL_FEATURE_SAI_FIFO_COUNT / 2U;

    // Configure Sgtl5000 I2C
    uint32_t i2cSourceClock = CLOCK_GetFreq(kCLOCK_BusClk);
    i2c_master_config_t i2cConfig = {0};
    I2C_MasterGetDefaultConfig(&i2cConfig);
    I2C_MasterInit(I2C0, &i2cConfig, i2cSourceClock);
    I2C_MasterTransferCreateHandle(I2C0, &g_i2cHandle, NULL, NULL);

    g_audioOut.init(&format, I2C0, &g_i2cHandle);
//     g_audioOut.dump_sgtl5000();

    AudioOutput::Buffer buf;
    buf.dataSize = BUFFER_SIZE * CHANNEL_NUM * sizeof(int16_t);
    buf.data = (uint8_t *)&g_outBuf[0][0];
    g_audioOut.add_buffer(&buf);
    buf.data = (uint8_t *)&g_outBuf[1][0];
    g_audioOut.add_buffer(&buf);

    g_audioOut.set_source(&g_audioOutConverter);
    AudioBuffer audioBuf(&g_audioBuf[0], BUFFER_SIZE);
    g_audioOutConverter.set_buffer(audioBuf);
    g_audioOutConverter.set_source(&g_mixer);

    g_kickSeq.set_sample_rate(kSampleRate);
    g_kickSeq.set_tempo(100.0f);
//     g_kickSeq.set_sequence("x---x---x---x-x-x---x---x---x---xx--x--x--xxx-x-");
    g_kickSeq.set_sequence("x---x-x----xx---");
    g_kickSeq.init();

    g_kickGen.set_sample_rate(kSampleRate);
    g_kickGen.set_sequence(&g_kickSeq);
    g_kickGen.set_freq(50.0f);
    g_kickGen.enable_sustain(false);
    g_kickGen.init();
    g_kickGen.set_attack(0.01f);
    g_kickGen.set_release(0.6f);

    g_bassSeq.set_sample_rate(kSampleRate);
    g_bassSeq.set_tempo(100.0f);
    g_bassSeq.set_sequence("--s>>>p-----s>>>>>>p----");
    g_bassSeq.init();

    g_bassGen.set_sample_rate(kSampleRate);
    g_bassGen.set_sequence(&g_bassSeq);
    g_bassGen.set_freq(40.0f);
    g_bassGen.enable_sustain(true);
    g_bassGen.init();
    g_bassGen.set_attack(0.3f);
    g_bassGen.set_release(1.0f);

//     g_tickSeq.set_sample_rate(kSampleRate);
//     g_tickSeq.set_tempo(100.0f);
//     g_tickSeq.set_sequence("----x-----x-");
//     g_tickSeq.init();
//
//     g_tickGen.set_sample_rate(kSampleRate);
//     g_tickGen.set_sequence(&g_tickSeq);
//     g_tickGen.set_freq(4000.0f);
//     g_tickGen.enable_sustain(false);
//     g_tickGen.init();
//     g_tickGen.set_attack(0.04f);
//     g_tickGen.set_release(0.3f);

    AudioBuffer mixBuf(&g_mixBuf[0], BUFFER_SIZE);
    g_mixer.set_buffer(mixBuf);
    g_mixer.set_input_count(2);
    g_mixer.set_input(0, &g_kickGen, 0.5f);
    g_mixer.set_input(1, &g_bassGen, 0.5f);
//     g_mixer.set_input(2, &g_tickGen, 0.3f);
}

void init_board()
{
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortD);

    // I2C0 pins
    const port_pin_config_t pinConfig = {
        .pullSelect = kPORT_PullDisable,
        .slewRate = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .openDrainEnable = kPORT_OpenDrainEnable,
        .driveStrength = kPORT_LowDriveStrength,
        .mux = kPORT_MuxAlt2,
    };
    PORT_SetMultiplePinsConfig(PORTB, (1 << 3)|(1 << 2), &pinConfig);

    // SAI pins
    PORT_SetPinMux(PORTC, 8, kPORT_MuxAlt4);
    PORT_SetPinMux(PORTA, 5, kPORT_MuxAlt6);
    PORT_SetPinMux(PORTA, 12, kPORT_MuxAlt6);
    PORT_SetPinMux(PORTA, 13, kPORT_MuxAlt6);
    PORT_SetPinMux(PORTC, 5, kPORT_MuxAlt4);

    // LED pins
    PORT_SetPinMux(PORTA, 1, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTA, 2, kPORT_MuxAsGpio);
    PORT_SetPinMux(PORTD, 5, kPORT_MuxAsGpio);

    const gpio_pin_config_t gpioOut = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0,
    };
    GPIO_PinInit(GPIOA, 1, &gpioOut);
    GPIO_PinInit(GPIOA, 2, &gpioOut);
    GPIO_PinInit(GPIOD, 5, &gpioOut);
}

int main(void)
{
    printf("Hello...\r\n");

    init_board();
    init_audio_out();

    g_myTimer.start();
    g_audioOut.start();
    g_accelThread.resume();

    Ar::Thread::getCurrent()->suspend();
    while (1)
    {
    }
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
