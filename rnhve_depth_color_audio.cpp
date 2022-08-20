/*
 * Realsense Network Hardware Video Encoder with Audio
 *
 * Realsense hardware encoded UDP HEVC aligned multi-streaming
 * - depth (Main10) + color (Main) + audio (raw signed 16-bit mono PCM in aux nhve channel)
 *
 * Copyright 2020 (C) Bartosz Meglicki <meglickib@gmail.com>
 * Audio added by CitizenOne
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

// Network Hardware Video Encoder
#include "nhve.h"

#include <fstream>
#include <streambuf> //loading json config
#include <iostream>
#include <Windows.h> // check for Escape press
#include <mutex>

// audio source for Windows
#include "audio_winmm.h"

// depth_video source
#include "depth_video_rs.h"

using namespace std;

int hint_user_on_failure(char *argv[]);
bool main_loop(nhve *streamer, depth_video_state& dv_state, audio_state& a_state, mutex* data_ready_mutex, condition_variable* cv, bool* data_ready);
int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config);

int main(int argc, char* argv[])
{
	//prepare NHVE Network Hardware Video Encoder
	struct nhve_net_config net_config = {0};
	struct nhve_hw_config hw_configs[2] = { {0}, {0} };
	struct nhve *streamer;

	struct input_args user_input = {0};
	user_input.depth_units=0.0001f; //optionally override with user input

	if(process_user_input(argc, argv, &user_input, &net_config, hw_configs) < 0)
		return 1;

	mutex data_ready_mutex;
	condition_variable cv;
	bool data_ready = false;

	audio_state a_state;
	a_state.data_mutex = &data_ready_mutex;
	a_state.data_ready = &data_ready;
	a_state.cv = &cv;
	audio* a = audio_init(a_state);

	depth_video_state dv_state;
	dv_state.data_mutex = &data_ready_mutex;
	dv_state.data_ready = &data_ready;
	dv_state.cv = &cv;
	depth_video* dv = depth_video_init(dv_state, user_input);

	if( (streamer = nhve_init(&net_config, hw_configs, 2, 1)) == NULL )
		return hint_user_on_failure(argv);

	bool status = main_loop(streamer, dv_state, a_state, &data_ready_mutex, &cv, &data_ready);

	nhve_close(streamer);
	depth_video_close(dv);
	audio_close(a);

	if(status)
		cout << "Finished successfully." << endl;

	return 0;
}

//true on success, false on failure
bool main_loop(nhve *streamer, depth_video_state& dv_state, audio_state& a_state, mutex* data_ready_mutex, condition_variable* cv, bool* data_ready)
{
	nhve_frame frame[3] = { {0}, {0}, {0} };
	bool frame_ready = false;

	// keep looping until the user hits escape
	while (!(GetAsyncKeyState(VK_ESCAPE) & 0x8000))
	{
		// wait for notification rather than run hot
		{  // scope here to manage the lifetime of the mutex
			unique_lock<mutex> lk(*data_ready_mutex);
			cv->wait(lk, [&] { return *data_ready; });

			if (dv_state.depth_video_data_ready)
			{
				//supply realsense frame data as ffmpeg frame data
				frame[0].data[0] = dv_state.depth_data;
				frame[0].data[1] = (uint8_t*)dv_state.depth_uv;
				frame[0].linesize[0] = frame[0].linesize[1] = dv_state.depth_stride; //the strides of Y and UV are equal

				frame[1].data[0] = dv_state.color_data;
				frame[1].linesize[0] = dv_state.color_stride;

				dv_state.depth_video_data_ready = false;
				*dv_state.data_ready = false;
				frame_ready = true;
			}
			else
			{
				// zero out subframes 0 and 1 if there's no depth/color frame this time
				frame[0].data[0] = 0;
				frame[0].data[1] = 0;
				frame[0].linesize[0] = 0;
				frame[0].linesize[1] = 0;

				frame[1].data[0] = 0;
				frame[1].linesize[0] = 0;
			}

			if (a_state.audio_data_ready)
			{
				// pass on the audio in subframe 2
				frame[2].data[0] = (uint8_t*)a_state.audio_buffer;
				frame[2].linesize[0] = a_state.audio_data_length_written;
				a_state.audio_data_ready = false;
				*a_state.data_ready = false;
				frame_ready = true;
			}
			else
			{
				// set subframe 2 to empty if there's no audio this time
				frame[2].data[0] = NULL;
				frame[2].linesize[0] = 0;
			}
		} // don't need the mutex anymore

		// only send a frame if at least one of the subframes had data
		if (frame_ready)
		{
			if (nhve_send(streamer, &frame[0], 0) != NHVE_OK)
			{
				cerr << "failed to send depth frame" << endl;
				break;
			}

			if (nhve_send(streamer, &frame[1], 1) != NHVE_OK)
			{
				cerr << "failed to send color frame" << endl;
				break;
			}

			if (nhve_send(streamer, &frame[2], 2) != NHVE_OK)
			{
				cerr << "failed to send aux frame" << endl;
				break;
			}

			frame_ready = false;
		}
	}

	//flush the streamer by sending NULL frame
	nhve_send(streamer, NULL, 0);
	nhve_send(streamer, NULL, 1);
	nhve_send(streamer, NULL, 2);

	delete [] dv_state.depth_uv;

	return true;
}

int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config)
{
	if(argc < 9)
	{
		cerr << "Usage: " << argv[0] << endl
		     << "       <host> <port>" << endl //1, 2
		     << "       <color/depth> # alignment direction" << endl //3
		     << "       <width_depth> <height_depth> <width_color> <height_color>" << endl //4, 5, 6, 7
			  << "       <framerate>" << endl //8
			  << "       [device] [bitrate_depth] [bitrate_color] [depth units] [json]" << endl; //9, 10, 11, 12, 13

		cerr << endl << "examples: " << endl;
		cerr << argv[0] << " 127.0.0.1 9766 color 640 360 640 360 30" << endl;
		cerr << argv[0] << " 127.0.0.1 9766 color 640 360 640 360 30 /dev/dri/renderD128" << endl;
		cerr << argv[0] << " 192.168.0.125 9766 color 640 360 640 360 30 /dev/dri/renderD128 4000000 1000000" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 848 480 848 480 30 /dev/dri/renderD128 8000000 1000000 0.0001" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 848 480 848 480 30 /dev/dri/renderD128 8000000 1000000 0.00005" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 848 480 848 480 30 /dev/dri/renderD128 8000000 1000000 0.000025" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 848 480 848 480 30 /dev/dri/renderD128 8000000 1000000 0.0000125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth 848 480 848 480 30 /dev/dri/renderD128 8000000 1000000 0.0000125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 848 480 848 480 30 /dev/dri/renderD128 8000000 1000000 0.00003125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth 848 480 1280 720 30 /dev/dri/renderD128 8000000 1000000 0.00003125" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 depth 640 480 1280 720 30 /dev/dri/renderD128 8000000 1000000 0.0000390625 my_config.json" << endl;
		cerr << argv[0] << " 192.168.0.100 9768 color 640 480 1280 720 30 /dev/dri/renderD128 8000000 1000000 0.0000390625 my_config.json" << endl;

		return -1;
	}

	net_config->ip = argv[1];
	net_config->port = atoi(argv[2]);

	char c = argv[3][0]; //color, depth
	if(c == 'c') input->align_to = Color;
	else if(c == 'd') input->align_to = Depth;
	else
	{
		cerr << "unnkown alignment target '" << argv[3] <<"', valid targets: 'color', 'depth'" << endl;
		return -1;
	}

	cout << "Aligning to " << ((input->align_to == Color) ? "color" : "depth") << endl;

	//for depth encoding we use 10 bit P010LE pixel format
	//that can be directly matched with Realsense output as P016LE Y plane
	//with precision/range trade-off controlled by Realsense Depth Units
	//for explanation see:
	//https://github.com/bmegli/realsense-depth-to-vaapi-hevc10/wiki/How-it-works

	//native format of Realsense RGB sensor is YUYV (YUY2, YUYV422)
	//see https://github.com/IntelRealSense/librealsense/issues/3042
	//however librealsense is unable to align color with YUYV to depth
	//see https://github.com/IntelRealSense/librealsense/blob/master/src/proc/align.cpp#L123

	//we will match:
	//- Realsense RGB sensor YUYV with VAAPI YUYV422 (same format) when aligning to color
	//- Realsense RGB sensor RGBA8 with VAAPI RGB0 (alpha ignored) when aligning to depth

	input->depth_width = atoi(argv[4]);
	input->depth_height = atoi(argv[5]);
	input->color_width = atoi(argv[6]);
	input->color_height = atoi(argv[7]);

	//DEPTH hardware encoding configuration
	hw_config[Depth].profile = FF_PROFILE_HEVC_MAIN_10;
	hw_config[Depth].pixel_format = "p010le";
	hw_config[Depth].encoder = "hevc_nvenc";

	//output dimensions will match alignment target
	hw_config[Depth].width = (input->align_to == Color) ? input->color_width : input->depth_width;
	hw_config[Depth].height = (input->align_to == Color) ? input->color_height : input->depth_height;

	hw_config[Depth].framerate = input->framerate = atoi(argv[8]);

	hw_config[Depth].device = argv[9]; //NULL as last argv argument, or device path

	if(argc > 10)
		hw_config[Depth].bit_rate = atoi(argv[10]);

	//COLOR hardware encoding configuration
	hw_config[Color].profile = FF_PROFILE_HEVC_MAIN;
	//use RGBA when aligning to color or depth (realsense YUYV doesn't match any of my hevc_nvenc input formats)
	hw_config[Color].pixel_format = "rgb0";
	hw_config[Color].encoder = "hevc_nvenc";

	//output dimensions will match alignment target
	hw_config[Color].width = (input->align_to == Color) ? input->color_width : input->depth_width;
	hw_config[Color].height = (input->align_to == Color) ? input->color_height : input->depth_height;

	hw_config[Color].framerate = input->framerate = atoi(argv[8]);

	hw_config[Color].device = argv[9]; //NULL as last argv argument, or device path

	if(argc > 11)
		hw_config[Color].bit_rate = atoi(argv[11]);

	//set highest quality and slowest encoding
	//this adds around 3 ms and 10% GPU usage on my 2017 KabyLake
	//with 848x480 HEVC Main10 encoding
	hw_config[Depth].compression_level = 1;
	hw_config[Color].compression_level = 0;

	//optionally set qp instead of bit_rate for CQP mode
	//hw_config[].qp = ...

	//optionally set gop_size (determines keyframes period)
	//hw_config[].gop_size = ...;

	if(argc > 12)
		input->depth_units = strtof(argv[12], NULL);

	if(argc > 13)
	{
		ifstream file(argv[13]);
		if(!file)
		{
			cerr << "unable to open file " << argv[13] << endl;
			return -1;
		}

		input->json = string((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
	}

	input->needs_postprocessing = false;

	return 0;
}

int hint_user_on_failure(char *argv[])
{
	cerr << "unable to initalize, try to specify device e.g:" << endl << endl;
	cerr << argv[0] << " 127.0.0.1 9766 color 640 360 640 360 30 /dev/dri/renderD128" << endl;
	cerr << argv[0] << " 127.0.0.1 9766 color 640 360 640 360 30 /dev/dri/renderD129" << endl;
	return -1;
}
