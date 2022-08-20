/**
 * Handles audio processing on Windows using WinMM
 */
#ifndef AUDIO_WINMM_H
#define AUDIO_WINMM_H

#include <Windows.h>
#include <mmreg.h>
#include <mmsystem.h>
#include <mutex>

#define SAMPLE_RATE 24000
#define CHANNELS 1
#define BYTES_PER_SAMPLE 2
// AAudio framesPerBurst on DevKit is 642 in regular performance mode, 290 in LOW_LATENCY mode
#define AUDIO_BUFFER_SAMPLES 642

using namespace std;

struct audio_state
{
    // guards the rest of audio_state
    mutex* data_mutex;
    condition_variable* cv;
    int16_t* audio_buffer;
    int audio_data_length_written;
    bool audio_data_ready;
    bool* data_ready;

    audio_state() :
        data_mutex(NULL),
        cv(NULL),
        audio_buffer(new int16_t[AUDIO_BUFFER_SAMPLES * CHANNELS]),
        audio_data_length_written(0),
        audio_data_ready(false),
        data_ready(NULL)
    {}

    ~audio_state()
    {
        delete audio_buffer;
    }
};

struct audio
{
    // double buffer of samples * channels * bytesPerSample bytes
    char buffers[2][AUDIO_BUFFER_SAMPLES * CHANNELS * BYTES_PER_SAMPLE];
    WAVEHDR headers[2] = { {},{} };                 // initialize headers to zeros
    HWAVEIN wi;
};

struct audio* audio_init(audio_state& cfg);
void audio_close(audio* audio);
static void CALLBACK audio_callback_wavedata(HWAVEIN hwi, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2);

#endif