#ifndef DEPTH_VIDEO_RS_H
#define DEPTH_VIDEO_RS_H

// Realsense API
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <iostream>
#include <mutex>
#include <thread>

#define BOUNDING_DEPTH 0.5f

using namespace std;

const uint16_t P010LE_MAX = 0xFFC0; //in binary 10 ones followed by 6 zeroes

//encoding index, alignment direction
enum Stream { Depth = 0, Color = 1 };

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

struct depth_video
{
	rs2::pipeline* realsense;
	thread worker_thread;
	bool volatile keep_working;

	depth_video() :
		realsense(NULL),
		keep_working(true)
	{}
};

// need to synchronise access to the realsense frame data between the worker_thread thread and the main thread
struct depth_video_state
{
	// guards the rest of depth_video_state
	mutex* data_mutex;
	condition_variable* cv;
	uint16_t* depth_uv; //data of dummy color plane for P010LE
	int depth_stride;
	uint8_t* depth_data;
	int color_stride;
	uint8_t* color_data;
	bool depth_video_data_ready;
	bool* data_ready;

	depth_video_state() :
		data_mutex(NULL),
		cv(NULL),
		depth_uv(NULL),
		depth_stride(0),
		depth_data(NULL),
		color_stride(0),
		color_data(NULL),
		depth_video_data_ready(false),
		data_ready(NULL)
	{}
};
// end protected by data_mutex

depth_video* depth_video_init(depth_video_state& dv_state, input_args& user_input);
void depth_video_close(depth_video* dv);
void init_realsense(rs2::pipeline& pipe, input_args& input);
void init_realsense_depth(rs2::pipeline& pipe, const rs2::config& cfg, input_args& input);
void print_intrinsics(const rs2::pipeline_profile& profile, rs2_stream stream);
void process_depth_data(const input_args& input, rs2::depth_frame& depth);
static void realsense_worker_thread(depth_video* dv, depth_video_state& dv_state, input_args& input);

#endif
