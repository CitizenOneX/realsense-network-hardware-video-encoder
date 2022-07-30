#pragma comment(lib,"winmm.lib")

#include <Windows.h>
#include <mmreg.h>
#include <mmsystem.h>
#include <mutex>

#define SAMPLE_RATE 22050
#define CHANNELS 1
#define BYTES_PER_SAMPLE 4
#define BUFFER_SAMPLES 4096

// pull the buffers out as globals
char buffers[2][BUFFER_SAMPLES * 4];      // 4096 32-bit float samples, what Unity likes best
WAVEHDR headers[2] = { {},{} };           // initialize headers to zeros
HWAVEIN wi;

// need to synchronise access to the float buffer between the callback and the main thread
std::mutex audio_buffer_mutex;
float audio_buffer[BUFFER_SAMPLES];

void CALLBACK callback_wavedata(HWAVEIN hwi, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2);

void init_audio()
{
    // Fill the WAVEFORMATEX struct to indicate the format of our recorded audio
    //   For this example we'll use medium quality, ie:  22050 Hz, mono, 32-bit floats
    WAVEFORMATEX wfx = {};
    wfx.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;     // PCM data as floats [-1..1]
    wfx.nChannels = CHANNELS;                    // 1 channel = mono sound
    wfx.nSamplesPerSec = SAMPLE_RATE;            // Samplerate.  22050 Hz
    wfx.wBitsPerSample = 8 * BYTES_PER_SAMPLE;   // 32 since we have 32-bit floats
    // These others are computations:
    wfx.nBlockAlign = wfx.wBitsPerSample * wfx.nChannels / 8;
    wfx.nAvgBytesPerSec = wfx.nBlockAlign * wfx.nSamplesPerSec;

    // Open our 'waveIn' recording device
    waveInOpen(&wi,                 // fill our 'wi' handle
        WAVE_MAPPER,                    // use default device (easiest)
        &wfx,                           // tell it our format
        (DWORD_PTR)callback_wavedata,   // call us back when a buffer is full
        NULL,                           // dwInstance (user data to pass back to the callback)
        CALLBACK_FUNCTION | WAVE_FORMAT_DIRECT   // tell it callback is a function CALLBACK_FUNCTION
    );

    // initialise the headers and buffers
    for (int i = 0; i < 2; ++i)
    {
        headers[i].lpData = buffers[i];                     // give it a pointer to our buffer
        headers[i].dwBufferLength = BUFFER_SAMPLES * 2;      // tell it the size of that buffer in bytes
        // the other parts of the header we don't really care about, and can be left at zero

        // Prepare each header
        waveInPrepareHeader(wi, &headers[i], sizeof(headers[i]));

        // And add it to the queue
        // Once we start recording, queued buffers will get filled with audio data
        waveInAddBuffer(wi, &headers[i], sizeof(headers[i]));
    }

    // start recording
    waveInStart(wi);
}

/// <summary>
/// Callback implementation
/// In the callback thread, lock the audio float buffer and
/// copy the PCM (float) data straight over
/// releasing the lock at the end and releasing the audio buffers back to the WaveIn device
/// </summary>
void CALLBACK callback_wavedata(HWAVEIN hwi, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2)
{
    // only process DATA, no need to do anything with open/close
    if (WIM_DATA == uMsg)
    {
        WAVEHDR* h = (WAVEHDR*)dwParam1;

        {
            std::lock_guard<std::mutex> guard(audio_buffer_mutex);
            memcpy(audio_buffer, h->lpData, h->dwBufferLength);
        }

        // then re-add the buffer to the queue
        h->dwFlags = 0;          // clear the 'done' flag
        h->dwBytesRecorded = 0;  // tell it no bytes have been recorded
        waveInPrepareHeader(hwi, h, sizeof(*h));
        waveInAddBuffer(hwi, h, sizeof(*h));
    }
}

void terminate_audio()
{
    waveInStop(wi);
    for (auto& h : headers)
    {
        waveInUnprepareHeader(wi, &h, sizeof(h));
    }
    waveInClose(wi);
}
