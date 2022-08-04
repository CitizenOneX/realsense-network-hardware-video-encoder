#include "depth_video_rs.h"

#ifndef M_PI
#define M_PI           3.14159265358979323846  /* pi */
#endif

using namespace std;

inline void update_thresholds(rs2::threshold_filter& filter, float center)
{
	filter.set_option(RS2_OPTION_MIN_DISTANCE, fmaxf(center - BOUNDING_DEPTH, 0.15f));
	filter.set_option(RS2_OPTION_MAX_DISTANCE, fminf(center + BOUNDING_DEPTH, 2.0f));
}

depth_video* depth_video_init(depth_video_state& dv_state, input_args& user_input)
{
	depth_video* dv = new depth_video();

	dv->keep_working = true;
	dv->realsense = new rs2::pipeline();
	init_realsense(*dv->realsense, user_input);
	dv->worker_thread = thread(realsense_worker_thread, dv, ref(dv_state), ref(user_input));

	return dv;
}

void depth_video_close(depth_video* dv)
{
	// close off the realsense source,
	// stop the realsense worker_thread thread
	dv->keep_working = false;

	if (dv->worker_thread.joinable())
		dv->worker_thread.join();

	delete dv;
}

/// <summary>
/// move the realsense wait-for-frames and processing to another thread
/// and only provide the frames to the main thread when they're ready
/// </summary>
/// <param name="realsense"></param>
/// <param name="input"></param>
static void realsense_worker_thread(depth_video* dv,  depth_video_state & dv_state, input_args& input)
{
	rs2::align aligner((input.align_to == Color) ? RS2_STREAM_COLOR : RS2_STREAM_DEPTH);
	rs2::threshold_filter thresh_filter;

	while (dv->keep_working)
	{
		rs2::frameset frameset = dv->realsense->wait_for_frames();
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
		int local_depth_stride = depth.get_stride_in_bytes();

		//L515 doesn't support setting depth units and clamping
		if (input.needs_postprocessing)
			process_depth_data(input, depth);

		if (!dv_state.depth_uv)
		{  //prepare dummy color plane for P010LE format, half the size of Y
			//we can't alloc it in advance, this is the first time we know realsense stride
			//the stride will be at least width * 2 (Realsense Z16, VAAPI P010LE)
			dv_state.depth_uv = new uint16_t[local_depth_stride / 2 * h / 2];

			for (int i = 0; i < local_depth_stride / 2 * h / 2; ++i)
				dv_state.depth_uv[i] = UINT16_MAX / 2; //dummy middle value for U/V, equals 128 << 8, equals 32768
		}

		{
			// use a mutex to make sure the main thread doesn't read these
			// while we're updating all of them
			// TODO not sure how long these data pointers will last after the frames go out of scope...
			std::lock_guard<std::mutex> guard(dv_state.depth_video_data_mutex);
			dv_state.depth_stride = local_depth_stride;
			dv_state.depth_data = (uint8_t*)depth.get_data();
			dv_state.color_stride = color.get_stride_in_bytes();
			dv_state.color_data = (uint8_t*)color.get_data();
			dv_state.depth_video_data_ready = true;
		}
	}

	dv->realsense->stop();
}

void process_depth_data(const input_args& input, rs2::depth_frame& depth)
{
	const int half_stride = depth.get_stride_in_bytes() / 2;
	const int height = depth.get_height();

	const float depth_units_set = depth.get_units();
	const float multiplier = depth_units_set / input.depth_units;

	//note - we process data in place rather than making a copy
	uint16_t* data = (uint16_t*)depth.get_data();

	for (int i = 0; i < half_stride * height; ++i)
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

	if (input.align_to == Color)
		print_intrinsics(profile, RS2_STREAM_COLOR);
	else
		print_intrinsics(profile, RS2_STREAM_DEPTH);
}

void init_realsense_depth(rs2::pipeline& pipe, const rs2::config& cfg, input_args& input)
{
	rs2::pipeline_profile profile = pipe.get_active_profile();

	rs2::depth_sensor depth_sensor = profile.get_device().first<rs2::depth_sensor>();

	if (!input.json.empty())
	{
		cout << "loading settings from json:" << endl << input.json << endl;
		auto serializable = profile.get_device().as<rs2::serializable_device>();
		serializable.load_json(input.json);
	}

	bool supports_depth_units = depth_sensor.supports(RS2_OPTION_DEPTH_UNITS) &&
		!depth_sensor.is_option_read_only(RS2_OPTION_DEPTH_UNITS);

	float depth_unit_set = input.depth_units;

	if (supports_depth_units)
	{
		try
		{
			depth_sensor.set_option(RS2_OPTION_DEPTH_UNITS, input.depth_units);
			depth_unit_set = depth_sensor.get_option(RS2_OPTION_DEPTH_UNITS);
			if (depth_unit_set != input.depth_units)
				cerr << "WARNING - device corrected depth units to value: " << depth_unit_set << endl;
		}
		catch (const exception&)
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
	cout << "-precision " << input.depth_units * 64.0f << " m (" << input.depth_units * 64.0f * 1000 << " mm)" << endl;

	bool supports_advanced_mode = depth_sensor.supports(RS2_CAMERA_INFO_ADVANCED_MODE);

	if (supports_advanced_mode)
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
	cout << (supports_advanced_mode ? "Clamping" : "Simulating clamping") <<
		" range at " << input.depth_units * P010LE_MAX << " m" << endl;
}

void print_intrinsics(const rs2::pipeline_profile& profile, rs2_stream stream)
{
	rs2::video_stream_profile stream_profile = profile.get_stream(stream).as<rs2::video_stream_profile>();
	rs2_intrinsics i = stream_profile.get_intrinsics();

	const float rad2deg = 180.0f / M_PI;
	float hfov = 2 * atan(i.width / (2 * i.fx)) * rad2deg;
	float vfov = 2 * atan(i.height / (2 * i.fy)) * rad2deg;

	cout << "The camera intrinsics (" << stream << "):" << endl;
	cout << "-width=" << i.width << " height=" << i.height << " hfov=" << hfov << " vfov=" << vfov << endl <<
		"-ppx=" << i.ppx << " ppy=" << i.ppy << " fx=" << i.fx << " fy=" << i.fy << endl;
	cout << "-distortion model " << i.model << " [" <<
		i.coeffs[0] << "," << i.coeffs[2] << "," << i.coeffs[3] << "," << i.coeffs[4] << "]" << endl;
}
