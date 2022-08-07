/**
 * Handles audio processing on Windows using WinMM
 */
#ifndef AUDIO_WINMM_H
#define AUDIO_WINMM_H

#include <Windows.h>
#include <mmreg.h>
#include <mmsystem.h>
#include <mutex>

#define SAMPLE_RATE 44100
#define CHANNELS 1
#define BYTES_PER_SAMPLE 4
#define AUDIO_BUFFER_SAMPLES 256

using namespace std;

struct audio_state
{
    // guards the rest of audio_state
    mutex* data_mutex;
    condition_variable* cv;
    float* audio_buffer;
    int audio_data_length_written;
    bool audio_data_ready;
    bool* data_ready;

    audio_state() :
        audio_buffer(new float[AUDIO_BUFFER_SAMPLES]),
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
    char buffers[2][AUDIO_BUFFER_SAMPLES * 4];      // 32-bit float samples, what Unity likes best
    WAVEHDR headers[2] = { {},{} };                 // initialize headers to zeros
    HWAVEIN wi;
};

struct audio* audio_init(audio_state& cfg);
void audio_close(audio* audio);
static void CALLBACK audio_callback_wavedata(HWAVEIN hwi, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2);

#endif