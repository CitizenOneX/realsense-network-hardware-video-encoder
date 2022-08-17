#pragma comment(lib,"winmm.lib")

#include "audio_winmm.h"

struct audio* audio_init(audio_state& cfg)
{
    audio* a = new audio();

    // Fill the WAVEFORMATEX struct to indicate the format of our recorded audio
    //   For this example we'll use medium quality, ie: 24000 Hz, mono, 32-bit floats
    WAVEFORMATEX wfx = {};
    wfx.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;     // PCM data as floats [-1..1]
    wfx.nChannels = CHANNELS;                    // 1 channel = mono sound
    wfx.nSamplesPerSec = SAMPLE_RATE;            // Sample rate
    wfx.wBitsPerSample = 8 * BYTES_PER_SAMPLE;   // 32 since we have 32-bit floats
    // These others are computations:
    wfx.nBlockAlign = wfx.wBitsPerSample * wfx.nChannels / 8;
    wfx.nAvgBytesPerSec = wfx.nBlockAlign * wfx.nSamplesPerSec;

    // Open our 'waveIn' recording device
    waveInOpen(&a->wi,                   // fill our 'wi' handle
        WAVE_MAPPER,                    // use default device (easiest)
        &wfx,                           // tell it our format
        (DWORD_PTR)audio_callback_wavedata,   // call us back when a buffer is full
        (DWORD_PTR)&cfg,                 // dwInstance (user data to pass back to the callback)
        CALLBACK_FUNCTION | WAVE_FORMAT_DIRECT   // tell it callback is a function CALLBACK_FUNCTION
    );

    // initialise the headers and buffers
    for (int i = 0; i < 2; ++i)
    {
        a->headers[i].lpData = a->buffers[i];                         // give it a pointer to our buffer
        a->headers[i].dwBufferLength = AUDIO_BUFFER_SAMPLES * 4;   // tell it the size of that buffer in bytes
        // the other parts of the header we don't really care about, and can be left at zero

        // Prepare each header
        waveInPrepareHeader(a->wi, &a->headers[i], sizeof(a->headers[i]));

        // And add it to the queue
        // Once we start recording, queued buffers will get filled with audio data
        waveInAddBuffer(a->wi, &a->headers[i], sizeof(a->headers[i]));
    }

    // start recording
    waveInStart(a->wi);

    return a;
}

/// <summary>
/// Callback implementation
/// In the callback thread, lock the audio float buffer and
/// copy the PCM (float) data straight over
/// releasing the lock at the end and releasing the audio buffers back to the WaveIn device
/// </summary>
void CALLBACK audio_callback_wavedata(HWAVEIN hwi, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2)
{
    // only process DATA, no need to do anything with open/close
    if (WIM_DATA == uMsg)
    {
        WAVEHDR* h = (WAVEHDR*)dwParam1;
        audio_state* state = (audio_state*)dwInstance;

        {  // use the scope operator to release the lock before notify()
            std::lock_guard<std::mutex> guard(*state->data_mutex);
            memcpy(state->audio_buffer, h->lpData, h->dwBytesRecorded);
            state->audio_data_length_written = h->dwBytesRecorded;
            state->audio_data_ready = true;
            *state->data_ready = true;
        }
        state->cv->notify_one();

        // then re-add the buffer to the queue
        h->dwFlags = 0;          // clear the 'done' flag
        h->dwBytesRecorded = 0;  // tell it no bytes have been recorded
        waveInPrepareHeader(hwi, h, sizeof(*h));
        waveInAddBuffer(hwi, h, sizeof(*h));
    }
}

void audio_close(audio* a)
{
    waveInStop(a->wi);
    for (auto& h : a->headers)
    {
        waveInUnprepareHeader(a->wi, &h, sizeof(h));
    }
    waveInClose(a->wi);
    delete a;
}
