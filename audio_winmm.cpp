#pragma comment(lib,"winmm.lib")

#include <Windows.h>
#include <mmsystem.h>
//#include <fstream>
#include <iostream>
#include <mutex>

// Network Hardware Video Encoder
#include "nhve.h"

#define SAMPLE_RATE 22050
#define CHANNELS 1
#define BYTES_PER_SAMPLE 2
//#define BUFFERS_PER_SECOND 25
#define BUFFER_SAMPLES 4096

// pull the buffers out as globals
//char buffers[2][SAMPLE_RATE * CHANNELS * BYTES_PER_SAMPLE / BUFFERS_PER_SECOND];    // 2 buffers, each 1/25 of a second long
char buffers[2][BUFFER_SAMPLES * 2];      // 4096 16-bit samples, might be what unity likes best
WAVEHDR headers[2] = { {},{} };           // initialize them to zeros
HWAVEIN wi;
//std::ofstream outfile("my_recorded_audio.bin", std::ios_base::out | std::ios_base::binary);

// need to synchronise access to the float buffer between the callback and the main thread
std::mutex float_buffer_mutex;
float float_buffer[BUFFER_SAMPLES];

void CALLBACK callback_wavedata(HWAVEIN hwi, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2);

void init_audio()
{
    // Fill the WAVEFORMATEX struct to indicate the format of our recorded audio
    //   For this example we'll use medium quality, ie:  22050 Hz, mono, 16-bit
    WAVEFORMATEX wfx = {};
    wfx.wFormatTag = WAVE_FORMAT_PCM;            // PCM is standard // TODO can't I use WAVE_FORMAT_IEEE_FLOAT directly here?
    wfx.nChannels = CHANNELS;                    // 1 channel = mono sound
    wfx.nSamplesPerSec = SAMPLE_RATE;            // Samplerate.  22050 Hz
    wfx.wBitsPerSample = 8 * BYTES_PER_SAMPLE;   // 16 bit samples // TODO Would be 32 if we got floats working here
    // These others are computations:
    wfx.nBlockAlign = wfx.wBitsPerSample * wfx.nChannels / 8;
    wfx.nAvgBytesPerSec = wfx.nBlockAlign * wfx.nSamplesPerSec;


    // Open our 'waveIn' recording device
    if (waveInOpen(&wi,                 // fill our 'wi' handle
        WAVE_MAPPER,                // use default device (easiest)
        &wfx,                       // tell it our format
        (DWORD_PTR)callback_wavedata,   // call us back when a buffer is full
        //NULL, // back to no callback
        NULL,      // dwInstance (user data to pass back to the callback)
        CALLBACK_FUNCTION | WAVE_FORMAT_DIRECT   // tell it callback is a function CALLBACK_FUNCTION
    ) != MMSYSERR_NOERROR)
    {
        std::cerr << "Error setting up WaveIn" << std::endl;
    };

    // initialise the headers and buffers
    for (int i = 0; i < 2; ++i)
    {
        headers[i].lpData = buffers[i];             // give it a pointer to our buffer
        //headers[i].dwBufferLength = SAMPLE_RATE * CHANNELS * BYTES_PER_SAMPLE / BUFFERS_PER_SECOND;      // tell it the size of that buffer in bytes
        headers[i].dwBufferLength = BUFFER_SAMPLES * 2;      // tell it the size of that buffer in bytes
        // the other parts of the header we don't really care about for this example, and can be left at zero

        // Prepare each header
        waveInPrepareHeader(wi, &headers[i], sizeof(headers[i]));

        // And add it to the queue
        //  Once we start recording, queued buffers will get filled with audio data
        waveInAddBuffer(wi, &headers[i], sizeof(headers[i]));
    }

    // In this example, I'm just going to dump the audio data to a binary file
    //std::ofstream outfile("my_recorded_audio.bin", std::ios_base::out | std::ios_base::binary);

    // Print some simple directions to the user
    std::cout << "Now recording audio.  Press Escape to stop and exit." << std::endl;

    // start recording!
    waveInStart(wi);
}

// Now that we are recording, keep polling our buffers to see if they have been filled.
//   If they have been, dump their contents to the file and re-add them to the queue so they
//   can get filled again, and again, and again
void process_audio(nhve_frame *auxFrame)
{
    int offset = 0; // in case we need to copy two blocks of data back

    for (auto& h : headers)      // check each header
    {
        if (h.dwFlags & WHDR_DONE)           // is this header done?
        {
            // if yes, copy the data back to the caller via the args
            //std::cout << "Buffer length writing: " << h.dwBufferLength << std::endl;
            auxFrame->linesize[0] = h.dwBufferLength; // + offset?...
            auxFrame->data[0] = (uint8_t*)h.lpData; // TODO buffers are contiguous, but maybe out of order? here I'm overwriting the second time...
            offset = h.dwBufferLength;
            //std::cout << "done copying buffer pointer bytes: " << *linesize << std::endl;

            // TODO will the buffers sometimes fill out of order? I.e. 1 then 0?
            // and when I iterate over them, do I get them in reverse order?

            // then re-add it to the queue
            h.dwFlags = 0;          // clear the 'done' flag
            h.dwBytesRecorded = 0;  // tell it no bytes have been recorded

            // re-add it  (I don't know why you need to prepare it again though...)
            waveInPrepareHeader(wi, &h, sizeof(h));
            waveInAddBuffer(wi, &h, sizeof(h));
        }
    }
}

/// <summary>
/// Callback implementation
/// In the callback thread, lock the audio float buffer and
/// copy the PCM data over, converting to float
/// releasing the lock at the end
/// </summary>
void CALLBACK callback_wavedata(HWAVEIN hwi, UINT uMsg, DWORD_PTR dwInstance, DWORD_PTR dwParam1, DWORD_PTR dwParam2)
{
    // only process DATA, no need to do anything with open/close
    if (WIM_DATA == uMsg)
    {
        std::cout << "called back with audio data" << std::endl;

        WAVEHDR* h = (WAVEHDR*)dwParam1;

        // interpret pcm data as signed, 16-bit ints
        int16_t* pcm_data = (int16_t*)h->lpData;

        // lock it for the whole conversion. If that's too long, 
        // double buffer and just swap during the lock
        // TODO If we can ask the audio system to give us floats instead that's even better
        {
            std::lock_guard<std::mutex> guard(float_buffer_mutex);
            for (size_t i = 0; i < BUFFER_SAMPLES; i++)
            {
                // promote int16_t to float32 and scale back to [-1..1]
                float_buffer[i] = pcm_data[i] / 32768.0f;
            }
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
    // Once the user hits escape, stop recording, and clean up
    waveInStop(wi);
    for (auto& h : headers)
    {
        waveInUnprepareHeader(wi, &h, sizeof(h));
    }
    waveInClose(wi);
    std::cout << "Audio terminating." << std::endl;
    // All done!
}
