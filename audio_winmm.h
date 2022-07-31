#pragma once

#include <Windows.h>
#include <mmreg.h>
#include <mmsystem.h>
#include <mutex>

#define SAMPLE_RATE 22050
#define CHANNELS 1
#define BYTES_PER_SAMPLE 4

class AudioWinMM
{
public:
    AudioWinMM(float* audio_buffer, std::mutex* audio_buffer_mutex, int* audio_data_length_written, volatile bool* audio_data_ready);
    ~AudioWinMM();

    void init();
    void terminate();
    static const int BUFFER_SAMPLES = 4096;

private:
    // pull the buffers out as globals
    char buffers[2][BUFFER_SAMPLES * 4];      // 4096 32-bit float samples, what Unity likes best
    WAVEHDR headers[2] = { {},{} };           // initialize headers to zeros
    HWAVEIN wi;

    // need to synchronise access to the float buffer between the callback and the main thread
    // main thread will create these but we need to use them
    static std::mutex* audio_buffer_mutex;
    static float* audio_buffer;
    static int* audio_data_length_written;
    static volatile bool* audio_data_ready;

    static void CALLBACK callback_wavedata(HWAVEIN hwi, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2);
};
