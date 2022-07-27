/*
 * Realsense Network Hardware Video Encoder with Audio
 *
 * Realsense hardware encoded UDP HEVC aligned multi-streaming
 * - depth (Main10) + color (Main) + audio (raw 16-bit mono PCM in aux nhve channel)
 *
 * Copyright 2020 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

// Network Hardware Video Encoder
#include "nhve.h"

// Realsense API
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <fstream>
#include <streambuf> //loading json config
#include <iostream>
#include <math.h>
#include <Windows.h> // check for Escape press

#define BOUNDING_DEPTH 0.5f

using namespace std;

int hint_user_on_failure(char *argv[]);

//encoding index, alignment direction
enum Stream {Depth = 0, Color = 1};

//user supplied input
struct input_args
{
	int depth_width;
	int depth_height;
	int color_width;
	int color_height;
	int framerate;
	float depth_units;
	Stream align_to;
	std::string json;
	bool needs_postprocessing;
};

bool main_loop(const input_args& input, rs2::pipeline& realsense, nhve *streamer);
void process_depth_data(const input_args &input, rs2::depth_frame &depth);

void init_realsense(rs2::pipeline& pipe, input_args& input);
void init_realsense_depth(rs2::pipeline& pipe, const rs2::config &cfg, input_args& input);
void print_intrinsics(const rs2::pipeline_profile& profile, rs2_stream stream);

int process_user_input(int argc, char* argv[], input_args* input, nhve_net_config *net_config, nhve_hw_config *hw_config);

const uint16_t P010LE_MAX = 0xFFC0; //in binary 10 ones followed by 6 zeroes

// implementations in audio_winmm.cpp
void init_audio();
void process_audio(nhve_frame *auxFrame);
void terminate_audio();

int main(int argc, char* argv[])
{
	//prepare NHVE Network Hardware Video Encoder
	struct nhve_net_config net_config = {0};
	struct nhve_hw_config hw_configs[2] = { {0}, {0} };
	struct nhve *streamer;

	struct input_args user_input = {0};
	user_input.depth_units=0.0001f; //optionally override with user input

	rs2::pipeline realsense;

	if(process_user_input(argc, argv, &user_input, &net_config, hw_configs) < 0)
		return 1;

	init_realsense(realsense, user_input);
	init_audio();

	if( (streamer = nhve_init(&net_config, hw_configs, 2, 1)) == NULL )
		return hint_user_on_failure(argv);

	bool status = main_loop(user_input, realsense, streamer);

	nhve_close(streamer);
	terminate_audio();

	if(status)
		cout << "Finished successfully." << endl;

	return 0;
}

inline void update_thresholds(rs2::threshold_filter& filter, float center)
{
	filter.set_option(RS2_OPTION_MIN_DISTANCE, fmaxf(center - BOUNDING_DEPTH, 0.15f));
	filter.set_option(RS2_OPTION_MAX_DISTANCE, fminf(center + BOUNDING_DEPTH, 2.0f));
}

//true on success, false on failure
bool main_loop(const input_args& input, rs2::pipeline& realsense, nhve *streamer)
{
	nhve_frame frame[3] = { {0}, {0}, {0} };

	uint16_t *depth_uv = NULL; //data of dummy color plane for P010LE

	rs2::align aligner( (input.align_to == Color) ? RS2_STREAM_COLOR : RS2_STREAM_DEPTH);
	rs2::threshold_filter thresh_filter;

	while (!(GetAsyncKeyState(VK_ESCAPE) & 0x8000))  // keep looping until the user hits escape
	{
		rs2::frameset frameset = realsense.wait_for_frames();
		frameset = aligner.process(frameset);

		rs2::depth_frame depth = frameset.get_depth_frame();
		// put a bounding volume around the object in the center of the frame, +-0.5m
		update_thresholds(thresh_filter, depth.get_distance(depth.get_width() / 2, depth.get_height() / 2));
		depth = thresh_filter.process(depth);
		rs2::video_frame color = frameset.get_color_frame();

		// TODO do I need to set all color frame pixels to black whose depth=0 in the depth frame?
		// can the threshold_filter tell me which pixels it changed? Or is it easier for me to
		// implement the threshold filter manually and set both depth and color together?

		const int h = depth.get_height();
		const int depth_stride=depth.get_stride_in_bytes();

		//L515 doesn't support setting depth units and clamping
		if(input.needs_postprocessing)
			process_depth_data(input, depth);

		if(!depth_uv)
		{  //prepare dummy color plane for P010LE format, half the size of Y
			//we can't alloc it in advance, this is the first time we know realsense stride
			//the stride will be at least width * 2 (Realsense Z16, VAAPI P010LE)
			depth_uv = new uint16_t[depth_stride/2*h/2];

			for(int i=0;i<depth_stride/2*h/2;++i)
				depth_uv[i] = UINT16_MAX / 2; //dummy middle value for U/V, equals 128 << 8, equals 32768
		}

		//supply realsense frame data as ffmpeg frame data
		frame[0].linesize[0] = frame[0].linesize[1] =  depth_stride; //the strides of Y and UV are equal
		frame[0].data[0] = (uint8_t*) depth.get_data();
		frame[0].data[1] = (uint8_t*) depth_uv;

		frame[1].linesize[0] = color.get_stride_in_bytes();
		frame[1].data[0] = (uint8_t*) color.get_data();

		// copy the audio data into the aux channel directly
		// TODO consider if we need to zero it out or reset a position pointer
		// after sending - otherwise the same frame will be sent every time?
		process_audio(&frame[2]);


		if(nhve_send(streamer, &frame[0], 0) != NHVE_OK)
		{
			cerr << "failed to send depth frame" << endl;
			break;
		}

		if(nhve_send(streamer, &frame[1], 1) != NHVE_OK)
		{
			cerr << "failed to send color frame" << endl;
			break;
		}

		if (nhve_send(streamer, &frame[2], 2) != NHVE_OK)
		{
			cerr << "failed to send aux frame" << endl;
			break;
		}

		// TODO check if this is necessary - I think it is otherwise we'll be sending
		// the same audio data every video frame
		frame[2].data[0] = NULL;
		frame[2].linesize[0] = 0;
	}

	//flush the streamer by sending NULL frame
	nhve_send(streamer, NULL, 0);
	nhve_send(streamer, NULL, 1);
	nhve_send(streamer, NULL, 2);

	delete [] depth_uv;

	return true;
}

void process_depth_data(const input_args &input, rs2::depth_frame &depth)
{
	const int half_stride = depth.get_stride_in_bytes()/2;
	const int height = depth.get_height();

	const float depth_units_set = depth.get_units();
	const float multiplier = depth_units_set / input.depth_units;

	//note - we process data in place rather than making a copy
	uint16_t* data = (uint16_t*)depth.get_data();

	for(int i = 0;i < half_stride * height; ++i)
	{
		uint32_t val = data[i] * multiplier;
		data[i] = val <= P010LE_MAX ? val : 0;
	}
}

void init_realsense(rs2::pipeline& pipe, input_args& input)
{
	rs2::config cfg;
	//use RGBA when aligning to color/depth (Realsense YUYV doesn't match any of my hevc_nvenc input formats)
	cfg.enable_stream(RS2_STREAM_DEPTH, input.depth_width, input.depth_height, RS2_FORMAT_Z16, input.framerate);
	cfg.enable_stream(RS2_STREAM_COLOR, input.color_width, input.color_height, RS2_FORMAT_RGBA8, input.framerate);

	rs2::pipeline_profile profile = pipe.start(cfg);

	init_realsense_depth(pipe, cfg, input);

	if(input.align_to == Color)
		print_intrinsics(profile, RS2_STREAM_COLOR);
	else
		print_intrinsics(profile, RS2_STREAM_DEPTH);
}

void init_realsense_depth(rs2::pipeline& pipe, const rs2::config &cfg, input_args& input)
{
	rs2::pipeline_profile profile = pipe.get_active_profile();

	rs2::depth_sensor depth_sensor = profile.get_device().first<rs2::depth_sensor>();

	if(!input.json.empty())
	{
		cout << "loading settings from json:" << endl << input.json  << endl;
		auto serializable  = profile.get_device().as<rs2::serializable_device>();
		serializable.load_json(input.json);
	}

	bool supports_depth_units = depth_sensor.supports(RS2_OPTION_DEPTH_UNITS) &&
										!depth_sensor.is_option_read_only(RS2_OPTION_DEPTH_UNITS);

	float depth_unit_set = input.depth_units;

	if(supports_depth_units)
	{
		try
		{
			depth_sensor.set_option(RS2_OPTION_DEPTH_UNITS, input.depth_units);
			depth_unit_set = depth_sensor.get_option(RS2_OPTION_DEPTH_UNITS);
			if(depth_unit_set != input.depth_units)
				cerr << "WARNING - device corrected depth units to value: " << depth_unit_set << endl;
		}
		catch(const exception &)
		{
			rs2::option_range range = depth_sensor.get_option_range(RS2_OPTION_DEPTH_UNITS);
			cerr << "failed to set depth units to " << input.depth_units << " (range is " << range.min << "-" << range.max << ")" << endl;
			throw;
		}
	}
	else
	{
		cerr << "WARNING - device doesn't support setting depth units!" << endl;
		input.needs_postprocessing = true;
	}

	cout << (supports_depth_units ? "Setting" : "Simulating") <<
		" realsense depth units: " << depth_unit_set << endl;
	cout << "This will result in:" << endl;
	cout << "-range " << input.depth_units * P010LE_MAX << " m" << endl;
	cout << "-precision " << input.depth_units*64.0f << " m (" << input.depth_units*64.0f*1000 << " mm)" << endl;

	bool supports_advanced_mode = depth_sensor.supports(RS2_CAMERA_INFO_ADVANCED_MODE);

	if(supports_advanced_mode)
	{
		 rs400::advanced_mode advanced = profile.get_device();
		 pipe.stop(); //workaround the problem with setting advanced_mode on running stream
		 STDepthTableControl depth_table = advanced.get_depth_table();
		 depth_table.depthClampMax = P010LE_MAX;
		 advanced.set_depth_table(depth_table);
		 profile = pipe.start(cfg);
	}
	else
	{
		cerr << "WARNING - device doesn't support advanced mode depth clamping!" << endl;
		input.needs_postprocessing = true;
	}
	cout << (supports_advanced_mode ?  "Clamping" : "Simulating clamping") <<
	" range at " << input.depth_units * P010LE_MAX << " m" << endl;
}

void print_intrinsics(const rs2::pipeline_profile& profile, rs2_stream stream)
{
	rs2::video_stream_profile stream_profile = profile.get_stream(stream).as<rs2::video_stream_profile>();
	rs2_intrinsics i = stream_profile.get_intrinsics();

	const float rad2deg = 180.0f / M_PI;
	float hfov = 2 * atan(i.width / (2*i.fx)) * rad2deg;
	float vfov = 2 * atan(i.height / (2*i.fy)) * rad2deg;

	cout << "The camera intrinsics (" << stream << "):" << endl;
	cout << "-width=" << i.width << " height=" << i.height << " hfov=" << hfov << " vfov=" << vfov << endl <<
           "-ppx=" << i.ppx << " ppy=" << i.ppy << " fx=" << i.fx << " fy=" << i.fy << endl;
	cout << "-distortion model " << i.model << " [" <<
		i.coeffs[0] << "," << i.coeffs[2] << "," << i.coeffs[3] << "," << i.coeffs[4] << "]" << endl;
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
