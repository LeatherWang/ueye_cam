/*******************************************************************************
* DO NOT MODIFY - AUTO-GENERATED
*
*
* DISCLAMER:
*
* This project was created within an academic research setting, and thus should
* be considered as EXPERIMENTAL code. There may be bugs and deficiencies in the
* code, so please adjust expectations accordingly. With that said, we are
* intrinsically motivated to ensure its correctness (and often its performance).
* Please use the corresponding web repository tool (e.g. github/bitbucket/etc.)
* to file bugs, suggestions, pull requests; we will do our best to address them
* in a timely manner.
*
*
* SOFTWARE LICENSE AGREEMENT (BSD LICENSE):
*
* Copyright (c) 2013-2016, Anqi Xu and contributors
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the School of Computer Science, McGill University,
*    nor the names of its contributors may be used to endorse or promote
*    products derived from this software without specific prior written
*    permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "ueye_cam/ueye_cam_nodelet.hpp"
#include <cstdlib> // needed for getenv()
#include <ros/package.h>
#include <camera_calibration_parsers/parse.h>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>


//#define DEBUG_PRINTOUT_FRAME_GRAB_RATES


using namespace std;
using namespace sensor_msgs::image_encodings;


namespace ueye_cam {


const std::string UEyeCamNodelet::DEFAULT_FRAME_NAME = "camera";
const std::string UEyeCamNodelet::DEFAULT_CAMERA_NAME = "camera";
const std::string UEyeCamNodelet::DEFAULT_CAMERA_TOPIC = "image_raw";
const std::string UEyeCamNodelet::DEFAULT_TIMEOUT_TOPIC = "timeout_count";
const std::string UEyeCamNodelet::DEFAULT_COLOR_MODE = "";
constexpr int UEyeCamDriver::ANY_CAMERA; // Needed since CMakeLists.txt creates 2 separate libraries: one for non-ROS parent class, and one for ROS child class


// Note that these default settings will be overwritten by queryCamParams() during connectCam()
//! @attention 下面的参数都会被覆盖，所以看看就行，别当真
UEyeCamNodelet::UEyeCamNodelet():
    nodelet::Nodelet(),
    UEyeCamDriver(ANY_CAMERA, DEFAULT_CAMERA_NAME),
    frame_grab_alive_(false),
    ros_cfg_(NULL),
    cfg_sync_requested_(false),
    ros_frame_count_(0),
    timeout_count_(0),
    cam_topic_(DEFAULT_CAMERA_TOPIC),
    timeout_topic_(DEFAULT_TIMEOUT_TOPIC),
    cam_intr_filename_(""),
    cam_params_filename_(""),
    init_clock_tick_(0),
    init_publish_time_(0),
    prev_output_frame_idx_(0)
{
  ros_image_.is_bigendian = (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__); // TODO: what about MS Windows?
  cam_params_.image_width = DEFAULT_IMAGE_WIDTH;
  cam_params_.image_height = DEFAULT_IMAGE_HEIGHT;
  cam_params_.image_left = -1;
  cam_params_.image_top = -1;
  cam_params_.color_mode = DEFAULT_COLOR_MODE;
  cam_params_.subsampling = cam_subsampling_rate_;
  cam_params_.binning = cam_binning_rate_;
  cam_params_.sensor_scaling = cam_sensor_scaling_rate_;
  cam_params_.auto_gain = false;
  cam_params_.master_gain = 0;
  cam_params_.red_gain = 0;
  cam_params_.green_gain = 0;
  cam_params_.blue_gain = 0;
  cam_params_.gain_boost = 0;
  cam_params_.auto_exposure = false;
  cam_params_.exposure = DEFAULT_EXPOSURE;
  cam_params_.auto_white_balance = false;
  cam_params_.white_balance_red_offset = 0;
  cam_params_.white_balance_blue_offset = 0;
  cam_params_.auto_frame_rate = false;
  cam_params_.frame_rate = DEFAULT_FRAME_RATE;
  cam_params_.output_rate = 0; // disable by default
  cam_params_.pixel_clock = DEFAULT_PIXEL_CLOCK;
  cam_params_.ext_trigger_mode = false;
  cam_params_.flash_delay = 0;
  cam_params_.flash_duration = DEFAULT_FLASH_DURATION;
  cam_params_.flip_upd = false;
  cam_params_.flip_lr = false;
  cam_params_.do_imu_sync = true; /// 不要改这里，没有用，该.cfg文件或者在on_init()函数最后调用configCallback之后再进行设置
  cam_params_.adaptive_exposure_mode_ = 2; //2: 主机，1: 从机
  cam_params_.crop_image = false;
  sync_buffer_size_ = 100;
  adaptive_exposure_ms_ = 10;
};


UEyeCamNodelet::~UEyeCamNodelet() {
  disconnectCam();

  // NOTE: sometimes deleting dynamic reconfigure object will lock up
  //       (suspect the scoped lock is not releasing the recursive mutex)
  //
  //if (ros_cfg_ != NULL) {
  //  delete ros_cfg_;
  //  ros_cfg_ = NULL;
  //}
};


void UEyeCamNodelet::onInit() {
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& local_nh = getPrivateNodeHandle();
  image_transport::ImageTransport it(nh);

  // Load camera-agnostic ROS parameters
  local_nh.param<string>("camera_name", cam_name_, DEFAULT_CAMERA_NAME);
  local_nh.param<string>("frame_name", frame_name_, DEFAULT_FRAME_NAME);
  local_nh.param<string>("camera_topic", cam_topic_, DEFAULT_CAMERA_TOPIC);
  local_nh.param<string>("timeout_topic", timeout_topic_, DEFAULT_TIMEOUT_TOPIC);
  local_nh.param<string>("camera_intrinsics_file", cam_intr_filename_, "");
  local_nh.param<int>("camera_id", cam_id_, ANY_CAMERA);
  local_nh.param<string>("camera_parameters_file", cam_params_filename_, "");
  if (cam_id_ < 0) {
    NODELET_WARN_STREAM("Invalid camera ID specified: " << cam_id_ <<
      "; setting to ANY_CAMERA");
    cam_id_ = ANY_CAMERA;
  }
  local_nh.param<bool>("crop_image", cam_params_.crop_image, false);

  loadIntrinsicsFile();

  // Setup dynamic reconfigure server
  ros_cfg_ = new ReconfigureServer(ros_cfg_mutex_, local_nh);
  ReconfigureServer::CallbackType f;
  f = bind(&UEyeCamNodelet::configCallback, this, _1, _2);

  // Setup publishers, subscribers, and services
  ros_cam_pub_ = it.advertiseCamera(cam_name_ + "/" + cam_topic_, 3);
  set_cam_info_srv_ = nh.advertiseService(cam_name_ + "/set_camera_info",
      &UEyeCamNodelet::setCamInfo, this);
  timeout_pub_ = nh.advertise<std_msgs::UInt64>(cam_name_ + "/" + timeout_topic_, 1, true);
  std_msgs::UInt64 timeout_msg; timeout_msg.data = 0; timeout_pub_.publish(timeout_msg);

  // For IMU sync
  ros_rect_pub_ = it.advertise(cam_name_ + "/image_rect", 100); // TODO : not hardcode name
  
  if (cam_params_.crop_image) {
    ros_cropped_pub_ = it.advertise(cam_name_ + "/image_cropped", 3);
  }

  // 告诉从机曝光时间
  ros_exposure_pub_ = nh.advertise<ueye_cam::Exposure>("master_exposure", 1);

  // 订阅IMU发过来的外部触发时间戳，用于同步
//  ros_timestamp_sub_ = nh.subscribe("ddd_mav/cam_imu_sync/cam_imu_stamp", 1,
//					  &UEyeCamNodelet::bufferTimestamp, this);

#ifdef CameraIMUSync
  ros_timestamp_sub_imu_ = nh.subscribe("/xsens/cam_imu_sync_stamp", 1,
                      &UEyeCamNodelet::bufferTimestampIMU, this);
#else
  ros_timestamp_sub_odom_ = nh.subscribe("/slam_car/cam_odom_sync_stamp", 1,
                      &UEyeCamNodelet::bufferTimestampOdometry, this);
#endif
  // 订阅主机发过来的曝光时间
  ros_exposure_sub_ = nh.subscribe("master_exposure", 1,
					  &UEyeCamNodelet::setSlaveExposure, this);

  // 告诉飞控，我相机准备好了?? 可以开始给我发送触发信号了
  //! @attention 参考: ueye_trigger_ready.cpp
  trigger_ready_srv_ = nh.serviceClient<std_srvs::Trigger>(cam_name_ + "/trigger_ready");

	// Initiate camera and start capture
	if (connectCam() != IS_SUCCESS) {
		NODELET_ERROR_STREAM("Failed to initialize UEye camera '" << cam_name_ << "'");
		return;
	}
	if (setStandbyMode() != IS_SUCCESS) {
		NODELET_ERROR_STREAM("Shutting down UEye camera interface at initialization...");
		ros::shutdown();
		return;
	}
	NODELET_INFO_STREAM("Camera " << cam_name_ << " Initialised at standby mode");

    //!@attention 注意这里会调用一次回调函数，会函数内部`cam_params_ = config`会将没有在参数服务器中(由launch文件设置)设置的参数恢复为dynamic_configure中的默认值
  ros_cfg_->setCallback(f); // this will call configCallback, which will configure the camera's parameters

  // Start IMU-camera trigger
  startFrameGrabber();

  NODELET_INFO_STREAM(
      "UEye camera '" << cam_name_ << "' initialized on topic " << ros_cam_pub_.getTopic() << endl <<
      "Width:\t\t\t" << cam_params_.image_width << endl <<
      "Height:\t\t\t" << cam_params_.image_height << endl <<
      "Left Pos.:\t\t" << cam_params_.image_left << endl <<
      "Top Pos.:\t\t" << cam_params_.image_top << endl <<
      "Color Mode:\t\t" << cam_params_.color_mode << endl <<
      "Subsampling:\t\t" << cam_params_.subsampling << endl <<
      "Binning:\t\t" << cam_params_.binning << endl <<
      "Sensor Scaling:\t\t" << cam_params_.sensor_scaling << endl <<
      "Auto Gain:\t\t" << cam_params_.auto_gain << endl <<
      "Master Gain:\t\t" << cam_params_.master_gain << endl <<
      "Red Gain:\t\t" << cam_params_.red_gain << endl <<
      "Green Gain:\t\t" << cam_params_.green_gain << endl <<
      "Blue Gain:\t\t" << cam_params_.blue_gain << endl <<
      "Gain Boost:\t\t" << cam_params_.gain_boost << endl <<
      "Auto Exposure:\t\t" << cam_params_.auto_exposure << endl <<
      "Exposure (ms):\t\t" << cam_params_.exposure << endl <<
      "Auto White Balance:\t" << cam_params_.auto_white_balance << endl <<
      "WB Red Offset:\t\t" << cam_params_.white_balance_red_offset << endl <<
      "WB Blue Offset:\t\t" << cam_params_.white_balance_blue_offset << endl <<
      "Flash Delay (us):\t" << cam_params_.flash_delay << endl <<
      "Flash Duration (us):\t" << cam_params_.flash_duration << endl <<
      "Ext Trigger Mode:\t" << cam_params_.ext_trigger_mode << endl <<
      "Auto Frame Rate:\t" << cam_params_.auto_frame_rate << endl <<
      "Frame Rate (Hz):\t" << cam_params_.frame_rate << endl <<
      "Output Rate (Hz):\t" << cam_params_.output_rate << endl <<
      "Pixel Clock (MHz):\t" << cam_params_.pixel_clock << endl <<
      "Mirror Image Upside Down:\t" << cam_params_.flip_upd << endl <<
      "Mirror Image Left Right:\t" << cam_params_.flip_lr << endl <<
      "Do camera px4 hardware sync:\t" << cam_params_.do_imu_sync << endl <<
      "Do adaptive exposure: \t" << cam_params_.adaptive_exposure_mode_ << endl <<
      "crop_image: " << cam_params_.crop_image << endl
  );
};


INT UEyeCamNodelet::parseROSParams(ros::NodeHandle& local_nh) {
  bool hasNewParams = false;
  ueye_cam::UEyeCamConfig prevCamParams = cam_params_;
  INT is_err = IS_SUCCESS;

  if (local_nh.hasParam("image_width")) {
    local_nh.getParam("image_width", cam_params_.image_width);
    if (cam_params_.image_width != prevCamParams.image_width) {
      if (cam_params_.image_width <= 0) {
        NODELET_WARN_STREAM("Invalid requested image width: " << cam_params_.image_width <<
          "; using current width: " << prevCamParams.image_width);
        cam_params_.image_width = prevCamParams.image_width;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("image_height")) {
    local_nh.getParam("image_height", cam_params_.image_height);
    if (cam_params_.image_height != prevCamParams.image_height) {
      if (cam_params_.image_height <= 0) {
        NODELET_WARN_STREAM("Invalid requested image height: " << cam_params_.image_height <<
          "; using current height: " << prevCamParams.image_height);
        cam_params_.image_height = prevCamParams.image_height;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("image_top")) {
    local_nh.getParam("image_top", cam_params_.image_top);
    if (cam_params_.image_top != prevCamParams.image_top) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("image_left")) {
    local_nh.getParam("image_left", cam_params_.image_left);
    if (cam_params_.image_left != prevCamParams.image_left) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("color_mode")) {
    local_nh.getParam("color_mode", cam_params_.color_mode);
    if (cam_params_.color_mode != prevCamParams.color_mode) {
      if (cam_params_.color_mode.length() > 0) {
        transform(cam_params_.color_mode.begin(),
            cam_params_.color_mode.end(),
            cam_params_.color_mode.begin(),
            ::tolower);

        if (name2colormode(cam_params_.color_mode) != 0) {
          hasNewParams = true;
        } else {
          WARN_STREAM("Invalid requested color mode for [" << cam_name_
            << "]: " << cam_params_.color_mode
            << "; using current mode: " << prevCamParams.color_mode);
          cam_params_.color_mode = prevCamParams.color_mode;
        }
      } else { // Empty requested color mode string
        cam_params_.color_mode = prevCamParams.color_mode;
      }
    }
  }
  if (local_nh.hasParam("subsampling")) {
    local_nh.getParam("subsampling", cam_params_.subsampling);
    if (cam_params_.subsampling != prevCamParams.subsampling) {
      if (!(cam_params_.subsampling == 1 ||
          cam_params_.subsampling == 2 ||
          cam_params_.subsampling == 4 ||
          cam_params_.subsampling == 8 ||
          cam_params_.subsampling == 16)) {
        NODELET_WARN_STREAM("Invalid or unsupported requested subsampling rate: " << cam_params_.subsampling <<
            "; using current rate: " << prevCamParams.subsampling);
        cam_params_.subsampling = prevCamParams.subsampling;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("auto_gain")) {
    local_nh.getParam("auto_gain", cam_params_.auto_gain);
    if (cam_params_.auto_gain != prevCamParams.auto_gain) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("master_gain")) {
    local_nh.getParam("master_gain", cam_params_.master_gain);
    if (cam_params_.master_gain != prevCamParams.master_gain) {
      if (cam_params_.master_gain < 0 || cam_params_.master_gain > 100) {
        NODELET_WARN_STREAM("Invalid master gain: " << cam_params_.master_gain <<
            "; using current master gain: " << prevCamParams.master_gain);
        cam_params_.master_gain = prevCamParams.master_gain;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("red_gain")) {
    local_nh.getParam("red_gain", cam_params_.red_gain);
    if (cam_params_.red_gain != prevCamParams.red_gain) {
      if (cam_params_.red_gain < 0 || cam_params_.red_gain > 100) {
        NODELET_WARN_STREAM("Invalid red gain: " << cam_params_.red_gain <<
            "; using current red gain: " << prevCamParams.red_gain);
        cam_params_.red_gain = prevCamParams.red_gain;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("green_gain")) {
    local_nh.getParam("green_gain", cam_params_.green_gain);
    if (cam_params_.green_gain != prevCamParams.green_gain) {
      if (cam_params_.green_gain < 0 || cam_params_.green_gain > 100) {
        NODELET_WARN_STREAM("Invalid green gain: " << cam_params_.green_gain <<
            "; using current green gain: " << prevCamParams.green_gain);
        cam_params_.green_gain = prevCamParams.green_gain;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("blue_gain")) {
    local_nh.getParam("blue_gain", cam_params_.blue_gain);
    if (cam_params_.blue_gain != prevCamParams.blue_gain) {
      if (cam_params_.blue_gain < 0 || cam_params_.blue_gain > 100) {
        NODELET_WARN_STREAM("Invalid blue gain: " << cam_params_.blue_gain <<
            "; using current blue gain: " << prevCamParams.blue_gain);
        cam_params_.blue_gain = prevCamParams.blue_gain;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("gain_boost")) {
    local_nh.getParam("gain_boost", cam_params_.gain_boost);
    if (cam_params_.gain_boost != prevCamParams.gain_boost) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("auto_exposure")) {
    local_nh.getParam("auto_exposure", cam_params_.auto_exposure);
    if (cam_params_.auto_exposure != prevCamParams.auto_exposure) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("exposure")) {
    local_nh.getParam("exposure", cam_params_.exposure);
    if (cam_params_.exposure != prevCamParams.exposure) {
      if (cam_params_.exposure < 0.0) {
        NODELET_WARN_STREAM("Invalid requested exposure: " << cam_params_.exposure <<
          "; using current exposure: " << prevCamParams.exposure);
        cam_params_.exposure = prevCamParams.exposure;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("auto_white_balance")) {
    local_nh.getParam("auto_white_balance", cam_params_.auto_white_balance);
    if (cam_params_.auto_white_balance != prevCamParams.auto_white_balance) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("white_balance_red_offset")) {
    local_nh.getParam("white_balance_red_offset", cam_params_.white_balance_red_offset);
    if (cam_params_.white_balance_red_offset != prevCamParams.white_balance_red_offset) {
      if (cam_params_.white_balance_red_offset < -50 || cam_params_.white_balance_red_offset > 50) {
        NODELET_WARN_STREAM("Invalid white balance red offset: " << cam_params_.white_balance_red_offset <<
            "; using current white balance red offset: " << prevCamParams.white_balance_red_offset);
        cam_params_.white_balance_red_offset = prevCamParams.white_balance_red_offset;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("white_balance_blue_offset")) {
    local_nh.getParam("white_balance_blue_offset", cam_params_.white_balance_blue_offset);
    if (cam_params_.white_balance_blue_offset != prevCamParams.white_balance_blue_offset) {
      if (cam_params_.white_balance_blue_offset < -50 || cam_params_.white_balance_blue_offset > 50) {
        NODELET_WARN_STREAM("Invalid white balance blue offset: " << cam_params_.white_balance_blue_offset <<
            "; using current white balance blue offset: " << prevCamParams.white_balance_blue_offset);
        cam_params_.white_balance_blue_offset = prevCamParams.white_balance_blue_offset;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("ext_trigger_mode")) {
    local_nh.getParam("ext_trigger_mode", cam_params_.ext_trigger_mode);
    // NOTE: no need to set any parameters, since external trigger / live-run
    //       modes come into effect during frame grab loop, which is assumed
    //       to not having been initialized yet
  }
  if (local_nh.hasParam("flash_delay")) {
    local_nh.getParam("flash_delay", cam_params_.flash_delay);
    // NOTE: no need to set any parameters, since flash delay comes into
    //       effect during frame grab loop, which is assumed to not having been
    //       initialized yet
  }
  if (local_nh.hasParam("flash_duration")) {
    local_nh.getParam("flash_duration", cam_params_.flash_duration);
    if (cam_params_.flash_duration < 0) {
      NODELET_WARN_STREAM("Invalid flash duration: " << cam_params_.flash_duration <<
          "; using current flash duration: " << prevCamParams.flash_duration);
      cam_params_.flash_duration = prevCamParams.flash_duration;
    }
    // NOTE: no need to set any parameters, since flash duration comes into
    //       effect during frame grab loop, which is assumed to not having been
    //       initialized yet
  }
  if (local_nh.hasParam("auto_frame_rate")) {
    local_nh.getParam("auto_frame_rate", cam_params_.auto_frame_rate);
    if (cam_params_.auto_frame_rate != prevCamParams.auto_frame_rate) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("frame_rate")) {
    local_nh.getParam("frame_rate", cam_params_.frame_rate);
    if (cam_params_.frame_rate != prevCamParams.frame_rate) {
      if (cam_params_.frame_rate <= 0.0) {
        NODELET_WARN_STREAM("Invalid requested frame rate: " << cam_params_.frame_rate <<
          "; using current frame rate: " << prevCamParams.frame_rate);
        cam_params_.frame_rate = prevCamParams.frame_rate;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("output_rate")) {
    local_nh.getParam("output_rate", cam_params_.output_rate);
    if (cam_params_.output_rate < 0.0) {
      WARN_STREAM("Invalid requested output rate for [" << cam_name_ << "]: " <<
        cam_params_.output_rate <<
        "; disable publisher throttling by default");
      cam_params_.output_rate = 0;
    } else {
      cam_params_.output_rate = std::min(cam_params_.frame_rate, cam_params_.output_rate);
      // hasNewParams = true; // No need to re-allocate buffer memory or reconfigure camera parameters
    }
  }
  if (local_nh.hasParam("pixel_clock")) {
    local_nh.getParam("pixel_clock", cam_params_.pixel_clock);
    if (cam_params_.pixel_clock != prevCamParams.pixel_clock) {
      if (cam_params_.pixel_clock < 0) {
        NODELET_WARN_STREAM("Invalid requested pixel clock: " << cam_params_.pixel_clock <<
          "; using current pixel clock: " << prevCamParams.pixel_clock);
        cam_params_.pixel_clock = prevCamParams.pixel_clock;
      } else {
        hasNewParams = true;
      }
    }
  }
  if (local_nh.hasParam("flip_upd")) {
    local_nh.getParam("flip_upd", cam_params_.flip_upd);
    if (cam_params_.flip_upd != prevCamParams.flip_upd) {
      hasNewParams = true;
    }
  }
  if (local_nh.hasParam("flip_lr")) {
    local_nh.getParam("flip_lr", cam_params_.flip_lr);
    if (cam_params_.flip_lr != prevCamParams.flip_lr) {
      hasNewParams = true;
    }
  }

  if (local_nh.hasParam("crop_image")) {
    local_nh.getParam("crop_image", cam_params_.crop_image);
    if (cam_params_.crop_image != prevCamParams.crop_image) {
      hasNewParams = true;
    }
  }

  if (hasNewParams) {
    // Configure color mode, resolution, and subsampling rate
    // NOTE: this batch of configurations are mandatory, to ensure proper allocation of local frame buffer
    if ((is_err = setColorMode(cam_params_.color_mode, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setSubsampling(cam_params_.subsampling, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setBinning(cam_params_.binning, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setResolution(cam_params_.image_width, cam_params_.image_height,
        cam_params_.image_left, cam_params_.image_top, false)) != IS_SUCCESS) return is_err;
    if ((is_err = setSensorScaling(cam_params_.sensor_scaling, false)) != IS_SUCCESS) return is_err;

    // Force synchronize settings and re-allocate frame buffer for redundancy
    // NOTE: although this might not be needed, assume that parseROSParams()
    //       is called only once per nodelet, thus ignore cost
    if ((is_err = syncCamConfig()) != IS_SUCCESS) return is_err;

    // Check for mutual exclusivity among requested sensor parameters
    if (!cam_params_.auto_exposure) { // Auto frame rate requires auto shutter
      cam_params_.auto_frame_rate = false;
    }
    if (cam_params_.auto_frame_rate) { // Auto frame rate has precedence over auto gain
      cam_params_.auto_gain = false;
    }

    // Configure camera sensor parameters
    if ((is_err = setGain(cam_params_.auto_gain, cam_params_.master_gain,
        cam_params_.red_gain, cam_params_.green_gain,
        cam_params_.blue_gain, cam_params_.gain_boost)) != IS_SUCCESS) return is_err;
    if ((is_err = setPixelClockRate(cam_params_.pixel_clock)) != IS_SUCCESS) return is_err;
    if ((is_err = setFrameRate(cam_params_.auto_frame_rate, cam_params_.frame_rate)) != IS_SUCCESS) return is_err;
    if ((is_err = setExposure(cam_params_.auto_exposure, cam_params_.exposure)) != IS_SUCCESS) return is_err;
    if ((is_err = setWhiteBalance(cam_params_.auto_white_balance, cam_params_.white_balance_red_offset,
      cam_params_.white_balance_blue_offset)) != IS_SUCCESS) return is_err;

    if ((is_err = setMirrorUpsideDown(cam_params_.flip_upd)) != IS_SUCCESS) return is_err;
    if ((is_err = setMirrorLeftRight(cam_params_.flip_lr)) != IS_SUCCESS) return is_err;
  }

  DEBUG_STREAM("Successfully applied settings from ROS params to [" << cam_name_ << "]");

  return is_err;
};


void UEyeCamNodelet::configCallback(ueye_cam::UEyeCamConfig& config, uint32_t level)
{
  if (!isConnected())
      return;

  // See if frame grabber needs to be restarted
  bool restartFrameGrabber = false;
  bool needToReallocateBuffer = false;
  if (level == RECONFIGURE_STOP && frame_grab_alive_) {
    restartFrameGrabber = true;
    stopFrameGrabber();
  }

  // Configure color mode, resolution, and subsampling rate
  if (config.color_mode != cam_params_.color_mode) {
    needToReallocateBuffer = true;
    if (setColorMode(config.color_mode, false) != IS_SUCCESS) return;
  }

  if (config.image_width != cam_params_.image_width ||
      config.image_height != cam_params_.image_height ||
      config.image_left != cam_params_.image_left ||
      config.image_top != cam_params_.image_top) {
    needToReallocateBuffer = true;
    if (setResolution(config.image_width, config.image_height,
        config.image_left, config.image_top, false) != IS_SUCCESS) {
      // Attempt to restore previous (working) resolution
      config.image_width = cam_params_.image_width;
      config.image_height = cam_params_.image_height;
      config.image_left = cam_params_.image_left;
      config.image_top = cam_params_.image_top;
      if (setResolution(config.image_width, config.image_height,
          config.image_left, config.image_top) != IS_SUCCESS) return;
    }
  }

  if (config.subsampling != cam_params_.subsampling) {
    needToReallocateBuffer = true;
    if (setSubsampling(config.subsampling, false) != IS_SUCCESS) return;
  }

  if (config.binning != cam_params_.binning) {
    needToReallocateBuffer = true;
    if (setBinning(config.binning, false) != IS_SUCCESS) return;
  }

  if (config.sensor_scaling != cam_params_.sensor_scaling) {
    needToReallocateBuffer = true;
    if (setSensorScaling(config.sensor_scaling, false) != IS_SUCCESS) return;
  }

  if (needToReallocateBuffer) {
    if (reallocateCamBuffer() != IS_SUCCESS) return;
    needToReallocateBuffer = false;
  }

  // (Re-)populate ROS image message
  // NOTE: the non-ROS UEye parameters and buffers have been updated by setColorMode(),
  // setResolution(), setSubsampling(), setBinning(), and setSensorScaling()
  ros_image_.header.frame_id = "/" + frame_name_;
  ros_image_.height = config.image_height / (config.sensor_scaling * config.subsampling * config.binning);
  ros_image_.width = config.image_width / (config.sensor_scaling * config.subsampling * config.binning);
  ros_image_.encoding = config.color_mode;
  ros_image_.step = cam_buffer_pitch_;
  ros_image_.is_bigendian = 0;
  ros_image_.data.resize(cam_buffer_size_);

  // Check for mutual exclusivity among requested sensor parameters
  if (!config.auto_exposure) { // Auto frame rate requires auto shutter
    config.auto_frame_rate = false;
  }
  if (config.auto_frame_rate) { // Auto frame rate has precedence over auto gain
    config.auto_gain = false;
  }

  // Configure camera sensor parameters
  if (config.auto_gain != cam_params_.auto_gain ||
      config.master_gain != cam_params_.master_gain ||
      config.red_gain != cam_params_.red_gain ||
      config.green_gain != cam_params_.green_gain ||
      config.blue_gain != cam_params_.blue_gain ||
      config.gain_boost != cam_params_.gain_boost) {
    // If any of the manual gain params change, then automatically toggle off auto_gain
    if (config.master_gain != cam_params_.master_gain ||
        config.red_gain != cam_params_.red_gain ||
        config.green_gain != cam_params_.green_gain ||
        config.blue_gain != cam_params_.blue_gain ||
        config.gain_boost != cam_params_.gain_boost) {
      config.auto_gain = false;
    }

    if (setGain(config.auto_gain, config.master_gain,
        config.red_gain, config.green_gain,
        config.blue_gain, config.gain_boost) != IS_SUCCESS) return;
  }

  if (config.pixel_clock != cam_params_.pixel_clock) {
    if (setPixelClockRate(config.pixel_clock) != IS_SUCCESS) return;
  }

  if (config.auto_frame_rate != cam_params_.auto_frame_rate ||
      config.frame_rate != cam_params_.frame_rate) {
    if (setFrameRate(config.auto_frame_rate, config.frame_rate) != IS_SUCCESS) return;
  }

  if (config.output_rate != cam_params_.output_rate) {
    if (!config.auto_frame_rate) {
      config.output_rate = std::min(config.output_rate, config.frame_rate);
    } // else, auto-fps is enabled, so don't bother checking validity of user-specified config.output_rate

    // Reset reference time for publisher throttle
    output_rate_mutex_.lock();
    init_publish_time_ = ros::Time(0);
    prev_output_frame_idx_ = 0;
    output_rate_mutex_.unlock();
  }

  if (config.auto_exposure != cam_params_.auto_exposure ||
      config.exposure != cam_params_.exposure) {
    if (setExposure(config.auto_exposure, config.exposure) != IS_SUCCESS) return;
  }
  
  // MINE reconfig to LOCK EXPOSURE!!!!!!!!!!!!!!!!!!!!!!!!!!
  /*bool autoExposureToSet = !config.lock_exposure;
  double instantExposure;
  if (is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE, &instantExposure, sizeof(instantExposure)) != IS_SUCCESS) {
    std::cout<< "Failed to query exposure timing for UEye camera" <<std::endl;
    return;
  }
  if (config.lock_exposure) {
    // To lock the camera exposure
    if (setExposure(autoExposureToSet, instantExposure) != IS_SUCCESS) {
      std::cout<< "Failed to LOCK exposure for UEye camera" <<std::endl;
      return;
    } else {
      std::cout<< "Locked exposure to" << instantExposure << std::endl;
    }
  } else {
    // To Unlock camera exposure
    if (setExposure(autoExposureToSet, instantExposure) != IS_SUCCESS) {
      std::cout<< "Failed to UNlock exposure for UEye camera" <<std::endl;
      return;
    } else {
      std::cout<< "unLocked exposure to" << instantExposure << std::endl;
    }
  }*/
  //!!!!!!!!!!!!!!!!!!!!!!!
  
  if (config.auto_white_balance != cam_params_.auto_white_balance ||
      config.white_balance_red_offset != cam_params_.white_balance_red_offset ||
      config.white_balance_blue_offset != cam_params_.white_balance_blue_offset) {
    if (setWhiteBalance(config.auto_white_balance, config.white_balance_red_offset,
        config.white_balance_blue_offset) != IS_SUCCESS) return;
  }

  if (config.flip_upd != cam_params_.flip_upd) {
    if (setMirrorUpsideDown(config.flip_upd) != IS_SUCCESS) return;
  }
  if (config.flip_lr != cam_params_.flip_lr) {
    if (setMirrorLeftRight(config.flip_lr) != IS_SUCCESS) return;
  }

  // NOTE: nothing needs to be done for config.ext_trigger_mode, since frame grabber loop will re-initialize to the right setting

  if (config.flash_delay != cam_params_.flash_delay ||
      config.flash_duration != cam_params_.flash_duration) {
    // NOTE: need to copy flash parameters to local copies since
    //       cam_params_.flash_duration is type int, and also sizeof(int)
    //       may not equal to sizeof(INT) / sizeof(UINT)
    INT flash_delay = config.flash_delay;
    UINT flash_duration = config.flash_duration;
    setFlashParams(flash_delay, flash_duration);
    // Copy back actual flash parameter values that were set
    config.flash_delay = flash_delay;
    config.flash_duration = flash_duration;
  }

  // Update local copy of parameter set to newly updated set
  cam_params_ = config;

  // Restart frame grabber if needed
  cfg_sync_requested_ = true;
  if (restartFrameGrabber) {
    startFrameGrabber();
  }

  DEBUG_STREAM("Successfully applied settings from dyncfg to [" << cam_name_ << "]");
}


INT UEyeCamNodelet::syncCamConfig(string dft_mode_str) {
  INT is_err;

  if ((is_err = UEyeCamDriver::syncCamConfig(dft_mode_str)) != IS_SUCCESS) return is_err;

  // Update ROS color mode string
  cam_params_.color_mode = colormode2name(is_SetColorMode(cam_handle_, IS_GET_COLOR_MODE));
  if (cam_params_.color_mode.empty()) {
    ERROR_STREAM("Force-updating to default color mode for [" << cam_name_ << "]: " <<
      dft_mode_str << "\n(THIS IS A CODING ERROR, PLEASE CONTACT PACKAGE AUTHOR)");
    cam_params_.color_mode = dft_mode_str;
    setColorMode(cam_params_.color_mode);
  }

  // Copy internal settings to ROS dynamic configure settings
  cam_params_.image_width = cam_aoi_.s32Width;   // Technically, these are width and height for the
  cam_params_.image_height = cam_aoi_.s32Height; // sensor's Area of Interest, and not of the image
  if (cam_params_.image_left >= 0) cam_params_.image_left = cam_aoi_.s32X; // TODO: 1 ideally want to ensure that aoi top-left does correspond to centering
  if (cam_params_.image_top >= 0) cam_params_.image_top = cam_aoi_.s32Y;
  cam_params_.subsampling = cam_subsampling_rate_;
  cam_params_.binning = cam_binning_rate_;
  cam_params_.sensor_scaling = cam_sensor_scaling_rate_;
  //cfg_sync_requested_ = true; // WARNING: assume that dyncfg client may want to override current settings

  // (Re-)populate ROS image message
  ros_image_.header.frame_id = "/" + frame_name_;
  // NOTE: .height, .width, .encoding, .step and .data determined in fillImgMsg();
  //       .is_bigendian determined in constructor

  return is_err;
}


INT UEyeCamNodelet::queryCamParams() {
  INT is_err = IS_SUCCESS;
  INT query;
  double pval1, pval2;

  // NOTE: assume that color mode, bits per pixel, area of interest info, resolution,
  //       sensor scaling rate, subsampling rate, and binning rate have already
  //       been synchronized by syncCamConfig()

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_GAIN, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_GAIN, &pval1, &pval2)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query auto gain mode for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_gain = (pval1 != 0);

  cam_params_.master_gain = is_SetHardwareGain(cam_handle_, IS_GET_MASTER_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  cam_params_.red_gain = is_SetHardwareGain(cam_handle_, IS_GET_RED_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  cam_params_.green_gain = is_SetHardwareGain(cam_handle_, IS_GET_GREEN_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
  cam_params_.blue_gain = is_SetHardwareGain(cam_handle_, IS_GET_BLUE_GAIN,
      IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);

  query = is_SetGainBoost(cam_handle_, IS_GET_SUPPORTED_GAINBOOST);
  if(query == IS_SET_GAINBOOST_ON) {
    query = is_SetGainBoost(cam_handle_, IS_GET_GAINBOOST);
    if (query == IS_SET_GAINBOOST_ON) {
      cam_params_.gain_boost = true;
    } else if (query == IS_SET_GAINBOOST_OFF) {
      cam_params_.gain_boost = false;
    } else {
      NODELET_ERROR_STREAM("Failed to query gain boost for UEye camera '" <<
          cam_name_ << "' (" << err2str(query) << ")");
      return query;
    }
  } else {
    cam_params_.gain_boost = false;
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_SHUTTER, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_SHUTTER, &pval1, &pval2)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query auto shutter mode for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_exposure = (pval1 != 0);

  if ((is_err = is_Exposure(cam_handle_, IS_EXPOSURE_CMD_GET_EXPOSURE,
      &cam_params_.exposure, sizeof(cam_params_.exposure))) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query exposure timing for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_WHITEBALANCE, &pval1, &pval2)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query auto white balance mode for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_white_balance = (pval1 != 0);

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_AUTO_WB_OFFSET, &pval1, &pval2)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query auto white balance red/blue channel offsets for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.white_balance_red_offset = pval1;
  cam_params_.white_balance_blue_offset = pval2;

  IO_FLASH_PARAMS currFlashParams;
  if ((is_err = is_IO(cam_handle_, IS_IO_CMD_FLASH_GET_PARAMS,
      (void*) &currFlashParams, sizeof(IO_FLASH_PARAMS))) != IS_SUCCESS) {
    ERROR_STREAM("Could not retrieve current flash parameter info for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.flash_delay = currFlashParams.s32Delay;
  cam_params_.flash_duration = currFlashParams.u32Duration;

  if ((is_err = is_SetAutoParameter(cam_handle_,
      IS_GET_ENABLE_AUTO_SENSOR_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS &&
      (is_err = is_SetAutoParameter(cam_handle_,
          IS_GET_ENABLE_AUTO_FRAMERATE, &pval1, &pval2)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query auto frame rate mode for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.auto_frame_rate = (pval1 != 0);

  if ((is_err = is_SetFrameRate(cam_handle_, IS_GET_FRAMERATE, &cam_params_.frame_rate)) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query frame rate for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }

  UINT currPixelClock;
  if ((is_err = is_PixelClock(cam_handle_, IS_PIXELCLOCK_CMD_GET,
      (void*) &currPixelClock, sizeof(currPixelClock))) != IS_SUCCESS) {
    NODELET_ERROR_STREAM("Failed to query pixel clock rate for UEye camera '" <<
        cam_name_ << "' (" << err2str(is_err) << ")");
    return is_err;
  }
  cam_params_.pixel_clock = currPixelClock;

  INT currROP = is_SetRopEffect(cam_handle_, IS_GET_ROP_EFFECT, 0, 0);
  cam_params_.flip_upd = ((currROP & IS_SET_ROP_MIRROR_UPDOWN) == IS_SET_ROP_MIRROR_UPDOWN);
  cam_params_.flip_lr = ((currROP & IS_SET_ROP_MIRROR_LEFTRIGHT) == IS_SET_ROP_MIRROR_LEFTRIGHT);


  // NOTE: do not need to (re-)populate ROS image message, since assume that
  //       syncCamConfig() was previously called

  DEBUG_STREAM("Successfully queries parameters from [" << cam_name_ << "]");

  return is_err;
};


INT UEyeCamNodelet::connectCam() {
  INT is_err = IS_SUCCESS;

  // 打开相机，打开驱动并建立与相机的连接。该函数在初始化成功后分配相机句柄
  if ((is_err = UEyeCamDriver::connectCam()) != IS_SUCCESS) return is_err;

  // (Attempt to) load UEye camera parameter configuration file
  if (cam_params_filename_.length() <= 0) { // Use default filename
    cam_params_filename_ = string(getenv("HOME")) + "/.ros/camera_conf/" + cam_name_ + ".ini";
  }
  loadCamConfig(cam_params_filename_);

  // Query existing configuration parameters from camera
  //!@attention 查询相机内部已经存在的参数，用于更新cam_params_
  if ((is_err = queryCamParams()) != IS_SUCCESS) return is_err;

  // Parse and load ROS camera settings
  // 注意上一步更改了cam_params_，所以这里只需要修改不一样的参数即可
  if ((is_err = parseROSParams(getPrivateNodeHandle())) != IS_SUCCESS) return is_err;

  return IS_SUCCESS;
};


INT UEyeCamNodelet::disconnectCam() {
  INT is_err = IS_SUCCESS;

  if (isConnected()) {
    stopFrameGrabber();
    is_err = UEyeCamDriver::disconnectCam();
  }

  return is_err;
};


bool UEyeCamNodelet::setCamInfo(sensor_msgs::SetCameraInfo::Request& req,
    sensor_msgs::SetCameraInfo::Response& rsp) {
  ros_cam_info_ = req.camera_info;
  ros_cam_info_.header.frame_id = "/" + frame_name_;
  rsp.success = saveIntrinsicsFile();
  rsp.status_message = (rsp.success) ? "successfully wrote to file" : "failed to write to file";
  return true;
};


void UEyeCamNodelet::frameGrabLoop() {
#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
  ros::Time prevStartGrab = ros::Time::now();
  ros::Time prevGrabbedFrame = ros::Time::now();
  ros::Time currStartGrab;
  ros::Time currGrabbedFrame;
  double startGrabSum = 0;
  double grabbedFrameSum = 0;
  double startGrabSumSqrd = 0;
  double grabbedFrameSumSqrd = 0;
  unsigned int startGrabCount = 0;
  unsigned int grabbedFrameCount = 0;
#endif

  ros::Rate idleDelay(200);

  // For IMU sync case: 
  // Start capturing and set external trigger mode straight away!
  if (cam_params_.do_imu_sync) {
	// Reset reference time to prevent throttling first frame
      	output_rate_mutex_.lock();
      	init_publish_time_ = ros::Time(0);
      	prev_output_frame_idx_ = 0;
      	output_rate_mutex_.unlock();

        //! @attention @attention 强制使能外部触发
	cam_params_.ext_trigger_mode = 1; // force set ext_trigger_mode
        //! @attention 设置软件外部触发 setExtTriggerModeSoftware()
        if (setExtTriggerMode() != IS_SUCCESS) {
        	NODELET_ERROR_STREAM("Shutting down UEye camera interface...");
        	ros::shutdown();
        	return;
        }
	
        NODELET_INFO_STREAM("Camera " << cam_name_ << " set to external trigger mode");

        //! @attention 告诉无人机可以向我发送触发信号了，并将在此之前由于相机缓冲区的图像刷掉
	sendTriggerReady();
  }
  
  // set camera model for rectification
  camera_model_.fromCameraInfo (ros_cam_info_);

  // Grabbing loop
  int prevNumSubscribers = 0;
  int currNumSubscribers = 0;
  int prevNumSubscribers_cropped = 0;
  int currNumSubscribers_cropped = 0;
  
  bool do_imu_sync_monitor = cam_params_.do_imu_sync;
//try{
  while (frame_grab_alive_ && ros::ok()) {
    // check if do_imu_sync flag was changed on the go
    if (do_imu_sync_monitor != cam_params_.do_imu_sync) 
	NODELET_ERROR_STREAM("do_imu_sync cannot be changed on the go. Restart the camera!");
    do_imu_sync_monitor = cam_params_.do_imu_sync;

    // For normal operation withOUT imu sync
    // Initialize live video mode if camera was previously asleep, and ROS image topic has subscribers;
    // and stop live video mode if ROS image topic no longer has any subscribers
    if (!cam_params_.do_imu_sync) {
        //!@attention 睡眠和曝光的切换，只在不进行IMU与相机同步时有效
        // 如果之前是睡眠模式，发现了新的订阅者，重新初始化live模式
        // 如果之前是live模式，现在没有订阅者了，则进入睡眠模式
    	currNumSubscribers = ros_cam_pub_.getNumSubscribers();
      if (cam_params_.crop_image) currNumSubscribers_cropped = ros_cropped_pub_.getNumSubscribers();
      
    	if ((currNumSubscribers > 0 && prevNumSubscribers <= 0) ||
        (currNumSubscribers_cropped > 0 && prevNumSubscribers_cropped <= 0)) {
      		// Reset reference time to prevent throttling first frame
      		output_rate_mutex_.lock();
      		init_publish_time_ = ros::Time(0);
      		prev_output_frame_idx_ = 0;
      		output_rate_mutex_.unlock();

      		if (cam_params_.ext_trigger_mode) {
        		if (setExtTriggerMode() != IS_SUCCESS) {
          			NODELET_ERROR_STREAM("Shutting down UEye camera interface...");
          			ros::shutdown();
          			return;
        		}
        		NODELET_INFO_STREAM("Camera " << cam_name_ << " set to external trigger mode");
      		} else {
        		if (setFreeRunMode() != IS_SUCCESS) {
          			ERROR_STREAM("Shutting down driver nodelet for [" << cam_name_ << "]");
          			ros::shutdown();
          			return;
        		}

        		// NOTE: need to copy flash parameters to local copies since
        		//       cam_params_.flash_duration is type int, and also sizeof(int)
        		//       may not equal to sizeof(INT) / sizeof(UINT)
        		INT flash_delay = cam_params_.flash_delay;
        		UINT flash_duration = cam_params_.flash_duration;
        		setFlashParams(flash_delay, flash_duration);
        		// Copy back actual flash parameter values that were set
        		cam_params_.flash_delay = flash_delay;
        		cam_params_.flash_duration = flash_duration;

        		INFO_STREAM("[" << cam_name_ << "] set to free-run mode");
      		}
    	} else if (currNumSubscribers <= 0 && prevNumSubscribers > 0) {
      		if (setStandbyMode() != IS_SUCCESS) {
			NODELET_ERROR_STREAM("Shutting down UEye camera interface...");
			ros::shutdown();
			return;
		}
		NODELET_INFO_STREAM("Camera " << cam_name_ << " set to standby mode");
	}
    	prevNumSubscribers = currNumSubscribers;
      prevNumSubscribers_cropped = currNumSubscribers_cropped;
    }

    // Send updated dyncfg parameters if previously changed
    // 就是说有些参数可能更改不成功，这样就需要重新刷新一下进行显示喽
    if (cfg_sync_requested_) {
      if (ros_cfg_mutex_.try_lock()) { // Make sure that dynamic reconfigure server or config callback is not active
        ros_cfg_mutex_.unlock();
        ros_cfg_->updateConfig(cam_params_); //! @attention 使用cam_params_更新动态配置参数
        cfg_sync_requested_ = false;
      }
    }


#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
    startGrabCount++;
    currStartGrab = ros::Time::now();
    if (startGrabCount > 1) {
      startGrabSum += (currStartGrab - prevStartGrab).toSec() * 1000.0;
      startGrabSumSqrd += ((currStartGrab - prevStartGrab).toSec() * 1000.0)*((currStartGrab - prevStartGrab).toSec() * 1000.0);
    }
    prevStartGrab = currStartGrab;
#endif

//_____________________________
// start capturing 
    //如果在触发模式下调用is_CaptureVideo()函数，相机将进入连续触发待机状态。每收到一个电子触发信号，相机就会捕捉一张图像，并立即准备就绪等待再次触发
    //! @todo 替换成 isConnected()
    if (isCapturing())
    {
      INT eventTimeout = (cam_params_.auto_frame_rate || cam_params_.ext_trigger_mode) ?
          (INT) 2000 : (INT) (1000.0 / cam_params_.frame_rate * 1.9); // tide strick timeout to avoid skipping frame. 
      //! @attention 阻塞等待
      // `processNextFrame`函数用于按照eventTimeout等待下一帧图像<到来事件>，如果超时，则抛出错误提示
      if (processNextFrame(eventTimeout) != NULL)
      {
        // Initialize shared pointers from member messages for nodelet intraprocess publishing
        sensor_msgs::ImagePtr img_msg_ptr(new sensor_msgs::Image(ros_image_));
        sensor_msgs::CameraInfoPtr cam_info_msg_ptr(new sensor_msgs::CameraInfo(ros_cam_info_));
        
        // Initialize/compute frame timestamp based on clock tick value from camera
        if (init_ros_time_.isZero()) {
            //! @attention 获取相机内部的时间戳
          if(getClockTick(&init_clock_tick_)) {
            init_ros_time_ = getImageTimestamp();

            // Deal with instability in getImageTimestamp due to daylight savings time
            //! @todo
            if (abs((ros::Time::now() - init_ros_time_).toSec()) > abs((ros::Time::now() - (init_ros_time_+ros::Duration(3600,0))).toSec())) { init_ros_time_ += ros::Duration(3600,0); }
            if (abs((ros::Time::now() - init_ros_time_).toSec()) > abs((ros::Time::now() - (init_ros_time_-ros::Duration(3600,0))).toSec())) { init_ros_time_ -= ros::Duration(3600,0); }
          }
        }

        //! @attention 设置时间戳，注意，这是图像刚刚到来的时候的时间，而不是读取完的时间，后面将进行读取
        img_msg_ptr->header.stamp = cam_info_msg_ptr->header.stamp = getImageTickTimestamp();

        // Process new frame
#ifdef DEBUG_PRINTOUT_FRAME_GRAB_RATES
        grabbedFrameCount++;
        currGrabbedFrame = ros::Time::now();
        if (grabbedFrameCount > 1) {
          grabbedFrameSum += (currGrabbedFrame - prevGrabbedFrame).toSec() * 1000.0;
          grabbedFrameSumSqrd += ((currGrabbedFrame - prevGrabbedFrame).toSec() * 1000.0)*((currGrabbedFrame - prevGrabbedFrame).toSec() * 1000.0);
        }
        prevGrabbedFrame = currGrabbedFrame;

        if (grabbedFrameCount > 1) {
          ROS_WARN_STREAM("\nPre-Grab: " << startGrabSum/startGrabCount << " +/- " <<
              sqrt(startGrabSumSqrd/startGrabCount - (startGrabSum/startGrabCount)*(startGrabSum/startGrabCount)) << " ms (" <<
              1000.0*startGrabCount/startGrabSum << "Hz)\n" <<
              "Post-Grab: " << grabbedFrameSum/grabbedFrameCount << " +/- " <<
              sqrt(grabbedFrameSumSqrd/grabbedFrameCount - (grabbedFrameSum/grabbedFrameCount)*(grabbedFrameSum/grabbedFrameCount)) << " ms (" <<
              1000.0*grabbedFrameCount/grabbedFrameSum << "Hz)\n" <<
              "Target: " << cam_params_.frame_rate << "Hz");
        }
#endif

        if (!frame_grab_alive_ || !ros::ok()) break;

        // Throttle publish rate
        bool throttle_curr_frame = false;
        output_rate_mutex_.lock();

        //没有使用，跟设置发布时间间隔有关系
        if (!cam_params_.ext_trigger_mode && cam_params_.output_rate > 0) {
          if (init_publish_time_.is_zero()) { // Set reference time 
            init_publish_time_ = img_msg_ptr->header.stamp;
          } else {
            double time_elapsed = (img_msg_ptr->header.stamp - init_publish_time_).toSec();
            uint64_t curr_output_frame_idx = std::floor(time_elapsed * cam_params_.output_rate);
            if (curr_output_frame_idx <= prev_output_frame_idx_) {
              throttle_curr_frame = true;
            } else {
              prev_output_frame_idx_ = curr_output_frame_idx;
            }
          }
        }
        output_rate_mutex_.unlock();
        if (throttle_curr_frame) continue;

        cam_info_msg_ptr->width = cam_params_.image_width / cam_sensor_scaling_rate_ / cam_binning_rate_;
        cam_info_msg_ptr->height = cam_params_.image_height / cam_sensor_scaling_rate_ / cam_binning_rate_;

        //_________________
        // Copy pixel content from internal frame buffer to ROS image
        // For IMU sync

        if (cam_params_.do_imu_sync)
        {
            // TODO: 9 make ros_image_.data (std::vector) use cam_buffer_ (char*) as underlying buffer,
            // without copy; alternatively after override reallocateCamBuffer() by allocating memory to ros_image_.data,
            // and setting that as internal camera buffer with is_SetAllocatedImageMem (complication is that vector's buffer need to be mlock()-ed)
            int expected_row_stride = cam_info_msg_ptr->width * bits_per_pixel_ / 8;

            if (cam_buffer_pitch_ < expected_row_stride)
            {
                ERROR_STREAM("Camera buffer pitch (" << cam_buffer_pitch_ <<
                             ") is smaller than expected for [" << cam_name_ << "]: " <<
                             "width (" << cam_info_msg_ptr->width << ") * bytes per pixel (" <<
                             bits_per_pixel_ / 8 << ") = " << expected_row_stride);
                continue;

            }
            else if (cam_buffer_pitch_ == expected_row_stride)
            {
                // Content is contiguous, so copy out the entire buffer
                output_rate_mutex_.lock();
                copy((char *) cam_buffer_,
                     ((char *) cam_buffer_) + cam_buffer_size_,
                     img_msg_ptr->data.begin());
                output_rate_mutex_.unlock();

            }
            else
            { // cam_buffer_pitch_ > expected_row_stride
                // Each row contains extra content according to cam_buffer_pitch_, so must copy out each row independently
                output_rate_mutex_.lock();
                std::vector<unsigned char>::iterator ros_image_it = img_msg_ptr->data.begin();
                char *cam_buffer_ptr = cam_buffer_;

                for (unsigned int row = 0; row < cam_info_msg_ptr->height; row++) {
                    ros_image_it = copy(cam_buffer_ptr, cam_buffer_ptr + expected_row_stride, ros_image_it);
                    cam_buffer_ptr += expected_row_stride;
                }

                img_msg_ptr->step = expected_row_stride; // fix the row stepsize/stride value
                output_rate_mutex_.unlock();
            }

            img_msg_ptr->header.seq = cam_info_msg_ptr->header.seq = ros_frame_count_; ros_frame_count_++;
            img_msg_ptr->header.frame_id = cam_info_msg_ptr->header.frame_id;

            if (!frame_grab_alive_ || !ros::ok()) { break; }


            // buffer the image frame and camera info
            output_rate_mutex_.lock();
            image_buffer_.push_back(*img_msg_ptr);
            cinfo_buffer_.push_back(*cam_info_msg_ptr);
            output_rate_mutex_.unlock();

            buffer_mutex_.lock();
            //adaptiveSync();
            if (image_buffer_.size() && timestamp_buffer_.size())
            {
                unsigned int i;
                if(image_buffer_.size() != timestamp_buffer_.size())
                    INFO_STREAM("image_buffer_ size: " << image_buffer_.size() << ", stamp_buffer_ size: " << timestamp_buffer_.size());
                for (i = 0; i < image_buffer_.size() && timestamp_buffer_.size() > 0 ;) {
                    //! @attention 使用触发时间和曝光时间的一半作为图像的时间戳
                    i += stampAndPublishImage(i);
                }
            }
            buffer_mutex_.unlock();

            // Check whether buffer has stale data and if so, throw away oldest
            if (image_buffer_.size() > 100) {
                image_buffer_.erase(image_buffer_.begin(), image_buffer_.begin()+50);
                cinfo_buffer_.erase(cinfo_buffer_.begin(), cinfo_buffer_.begin()+50);
                //ROS_ERROR_THROTTLE(1, "%i: Dropping image", cam_id_);
                INFO_STREAM("[ " << cam_name_ << " ] Dropping half of the image buffer");
            }
        }
        else  // For non sync cases
        {
            if (!fillMsgData(*img_msg_ptr)) {
                ROS_INFO("Skip one image messge filled.");
                continue;
            }

            img_msg_ptr->header.seq = cam_info_msg_ptr->header.seq = ros_frame_count_++;
            img_msg_ptr->header.frame_id = cam_info_msg_ptr->header.frame_id;

            if (!frame_grab_alive_ || !ros::ok()) break;

            ros_cam_pub_.publish(img_msg_ptr, cam_info_msg_ptr);
            // Publish Cropped images
            if (cam_params_.crop_image) publishCroppedImage(*img_msg_ptr);
        }


      }// end if (processNextFrame(eventTimeout) != NULL)
      else
      {
          ROS_WARN("Timeout!!!!");
      }
    }
    else
    {
        init_ros_time_ = ros::Time(0);
        init_clock_tick_ = 0;
    }

    if (!frame_grab_alive_ || !ros::ok()) break;
    idleDelay.sleep();
  }

//} catch (int e) {
//    cout << "An exception occurred. Exception Nr. " << e << '\n';
//}
  setStandbyMode();
  frame_grab_alive_ = false;

  DEBUG_STREAM("Frame grabber loop terminated for [" << cam_name_ << "]");
}


void UEyeCamNodelet::startFrameGrabber() {
  frame_grab_alive_ = true;
  frame_grab_thread_ = thread(bind(&UEyeCamNodelet::frameGrabLoop, this));
};


void UEyeCamNodelet::stopFrameGrabber() {
  frame_grab_alive_ = false;
  if (frame_grab_thread_.joinable()) {
    frame_grab_thread_.join();
  }
  frame_grab_thread_ = thread();
};

const std::map<INT, std::string> UEyeCamNodelet::ENCODING_DICTIONARY = {
    { IS_CM_SENSOR_RAW8, sensor_msgs::image_encodings::BAYER_RGGB8 },
    { IS_CM_SENSOR_RAW10, sensor_msgs::image_encodings::BAYER_RGGB16 },
    { IS_CM_SENSOR_RAW12, sensor_msgs::image_encodings::BAYER_RGGB16 },
    { IS_CM_SENSOR_RAW16, sensor_msgs::image_encodings::BAYER_RGGB16 },
    { IS_CM_MONO8, sensor_msgs::image_encodings::MONO8 },
    { IS_CM_MONO10, sensor_msgs::image_encodings::MONO16 },
    { IS_CM_MONO12, sensor_msgs::image_encodings::MONO16 },
    { IS_CM_MONO16, sensor_msgs::image_encodings::MONO16 },
    { IS_CM_RGB8_PACKED, sensor_msgs::image_encodings::RGB8 },
    { IS_CM_BGR8_PACKED, sensor_msgs::image_encodings::BGR8 },
    { IS_CM_RGB10_PACKED, sensor_msgs::image_encodings::RGB16 },
    { IS_CM_BGR10_PACKED, sensor_msgs::image_encodings::BGR16 },
    { IS_CM_RGB10_UNPACKED, sensor_msgs::image_encodings::RGB16 },
    { IS_CM_BGR10_UNPACKED, sensor_msgs::image_encodings::BGR16 },
    { IS_CM_RGB12_UNPACKED, sensor_msgs::image_encodings::RGB16 },
    { IS_CM_BGR12_UNPACKED, sensor_msgs::image_encodings::BGR16 }
};

bool UEyeCamNodelet::fillMsgData(sensor_msgs::Image& img) const {
  // Copy pixel content from internal frame buffer to img
  // and unpack to proper pixel format
  INT expected_row_stride = cam_aoi_.s32Width * bits_per_pixel_/8;
  if (cam_buffer_pitch_ < expected_row_stride) {
    ERROR_STREAM("Camera buffer pitch (" << cam_buffer_pitch_ <<
        ") is smaller than expected for [" << cam_name_ << "]: " <<
        "width (" << cam_aoi_.s32Width << ") * bytes per pixel (" <<
        bits_per_pixel_/8 << ") = " << expected_row_stride);
    return false;
  }

  // allocate target memory
  img.width = cam_aoi_.s32Width;
  img.height = cam_aoi_.s32Height;
  img.encoding = ENCODING_DICTIONARY.at(color_mode_);
  img.step = img.width * sensor_msgs::image_encodings::numChannels(img.encoding) * sensor_msgs::image_encodings::bitDepth(img.encoding)/8;
  img.data.resize(img.height * img.step);

  DEBUG_STREAM("Allocated ROS image buffer for [" << cam_name_ << "]:" <<
      "\n  size: " << cam_buffer_size_ <<
      "\n  width: " << img.width <<
      "\n  height: " << img.height <<
      "\n  step: " << img.step <<
      "\n  encoding: " << img.encoding);

  const std::function<void*(void*, void*, size_t)> unpackCopy = getUnpackCopyFunc(color_mode_);

  if (cam_buffer_pitch_ == expected_row_stride) {
    // Content is contiguous, so copy out the entire buffer at once
    unpackCopy(img.data.data(), cam_buffer_, img.height*expected_row_stride);
  } else { // cam_buffer_pitch_ > expected_row_stride
    // Each row contains extra buffer according to cam_buffer_pitch_, so must copy out each row independently
    uint8_t* dst_ptr = img.data.data();
    char* cam_buffer_ptr = cam_buffer_;
    for (INT row = 0; row < cam_aoi_.s32Height; row++) {
      unpackCopy(dst_ptr, cam_buffer_ptr, expected_row_stride);
      cam_buffer_ptr += cam_buffer_pitch_;
      dst_ptr += img.step;
    }
  }
  return true;
}


void UEyeCamNodelet::loadIntrinsicsFile() {
  if (cam_intr_filename_.length() <= 0) { // Use default filename
    cam_intr_filename_ = string(getenv("HOME")) + "/.ros/camera_info/" + cam_name_ + ".yaml";
  }

  if (camera_calibration_parsers::readCalibration(cam_intr_filename_, cam_name_, ros_cam_info_)) {
    NODELET_DEBUG_STREAM("Loaded intrinsics parameters for UEye camera " << cam_name_);
  }
  ros_cam_info_.header.frame_id = "/" + frame_name_;
};


bool UEyeCamNodelet::saveIntrinsicsFile() {
  if (camera_calibration_parsers::writeCalibration(cam_intr_filename_, cam_name_, ros_cam_info_)) {
    NODELET_DEBUG_STREAM("Saved intrinsics parameters for UEye camera " << cam_name_ <<
        " to " << cam_intr_filename_);
    return true;
  }
  return false;
}

ros::Time UEyeCamNodelet::getImageTimestamp() {
  UEYETIME utime;
  if(getTimestamp(&utime)) {
    struct tm tm;
    tm.tm_year = utime.wYear - 1900;
    tm.tm_mon = utime.wMonth - 1;
    tm.tm_mday = utime.wDay;
    tm.tm_hour = utime.wHour;
    tm.tm_min = utime.wMinute;
    tm.tm_sec = utime.wSecond;
    return ros::Time(mktime(&tm),utime.wMilliseconds*1e6);
  }
  return ros::Time::now();
}

ros::Time UEyeCamNodelet::getImageTickTimestamp() {
  uint64_t tick;
  if(getClockTick(&tick)) {
    return init_ros_time_ + ros::Duration(double(tick - init_clock_tick_)*1e-7);
  }
  return ros::Time::now();
}
// TODO: 0 bug where nodelet locks and requires SIGTERM when there are still subscribers (need to find where does code hang)


void UEyeCamNodelet::handleTimeout() {
  std_msgs::UInt64 timeout_msg;
  timeout_msg.data = ++timeout_count_;
  timeout_pub_.publish(timeout_msg);
};

//-------------------------
// For IMU sync
void UEyeCamNodelet::setSlaveExposure(const ueye_cam::Exposure &msg)
{
    //! @attention for双目，主从，这里是设置从机的曝光时间跟主机一样，双目嘛，必须保证曝光时间相同
	if(cam_params_.adaptive_exposure_mode_ == 1) { // accept exposure timing from master camera
		// TODO : re-add check for sequence again
		adaptive_exposure_ms_ = msg.exposure_ms;
		bool auto_exposure = false;
		if (setExposure(auto_exposure , adaptive_exposure_ms_) != IS_SUCCESS) {
			ROS_ERROR("Slave adaptive exposure setting failed");
		}
	}
};

// 无人机发过来外部触发开始时间，外部触发使用的硬件上的
//void UEyeCamNodelet::bufferTimestamp(const mavros_msgs::CamIMUStamp &msg)
//{
//	if(cam_params_.do_imu_sync) {
//		buffer_mutex_.lock();
//		timestamp_buffer_.push_back(msg);

//		// Check whether buffer has stale stamp and if so throw away oldest
//		if (timestamp_buffer_.size() > 100) {
//			timestamp_buffer_.erase(timestamp_buffer_.begin(), timestamp_buffer_.begin()+50);
//			//ROS_ERROR_THROTTLE(1, "Dropping timestamp");
//			INFO_STREAM("[ " << cam_name_ << " ] Dropping half of the timestamp buffer.");
//		}
//		buffer_mutex_.unlock();
//    }
//}

#ifdef CameraIMUSync
void UEyeCamNodelet::bufferTimestampIMU(const xsens_driver::CamIMUStamp &msg)
{
    if(cam_params_.do_imu_sync) {
        buffer_mutex_.lock();
        timestamp_buffer_.push_back(msg);
        //cout<<(ros::Time::now()-msg.frame_stamp).toSec()<<endl;

        // Check whether buffer has stale stamp and if so throw away oldest
        if (timestamp_buffer_.size() > 100) {
            timestamp_buffer_.erase(timestamp_buffer_.begin(), timestamp_buffer_.begin()+50);
            //ROS_ERROR_THROTTLE(1, "Dropping timestamp");
            INFO_STREAM("[ " << cam_name_ << " ] Dropping half of the timestamp buffer.");
        }
        buffer_mutex_.unlock();
    }
}
#else
void UEyeCamNodelet::bufferTimestampOdometry(const slam_car::CamOdomStamp &msg)
{
    if(cam_params_.do_imu_sync) {
        buffer_mutex_.lock();
        timestamp_buffer_.push_back(msg);
        cout<<(ros::Time::now()-msg.frame_stamp).toSec()<<endl;

        // Check whether buffer has stale stamp and if so throw away oldest
        if (timestamp_buffer_.size() > 100) {
            timestamp_buffer_.erase(timestamp_buffer_.begin(), timestamp_buffer_.begin()+50);
            //ROS_ERROR_THROTTLE(1, "Dropping timestamp");
            INFO_STREAM("[ " << cam_name_ << " ] Dropping half of the timestamp buffer.");
        }
        buffer_mutex_.unlock();
    }
}
#endif

//! @attention @attention 能够实现同步的关键函数，IMU和相机的计数进行同步，只有二者计数值相等时才表示二者的时间是同步的!!
void UEyeCamNodelet::sendTriggerReady()
{
    //! @todo 飞控的Node一启动，就开始发触发信号!??
	acknTriggerCommander(); // call service: First ackn will STOP px4 triggering

	// set stamp_buffer_offset_ from px4
	ros::Duration(1).sleep(); // wait for timestamp callback from mavros

    // 因为捕获图像的线程和无人机的线程是不同线程，所以在本线程运行到这里之前，可能`frame_seq_id`不是0
    if (timestamp_buffer_.empty())
        stamp_buffer_offset_ = 0;
    else
        stamp_buffer_offset_ = 1 + (uint)(timestamp_buffer_.end()-1)->frame_seq_id;
	stamp_buffer_offset_double_ = (double)stamp_buffer_offset_;

	INFO_STREAM("Detected px4 starting stamp sequence will be: " << stamp_buffer_offset_);

	timestamp_buffer_.clear(); // timestamp_buffer_ should have some elements already from px4 since it is in a different thread.
    ros_frame_count_ = 0;  //清空图像计数器
	int i=0; 

    //! @attention 将缓冲区内的所有未被使用的图像刷掉，因为还没开始触发呢，怎么可能有图像呢
	while (processNextFrame(2000) != NULL) {i++;} // this should flush all the unused frame in the camera buffer
	INFO_STREAM("Flashed " << i << " images from camera buffer prior to start!");
	
	acknTriggerCommander(); // call service: second ackn will RESTART px4 triggering
};

void UEyeCamNodelet::acknTriggerCommander()
{
	std_srvs::Trigger sig;
    //! @attention 注意这里如果失败了，很可能导致不同步
	if (!trigger_ready_srv_.call(sig)) {
		ROS_ERROR("Failed to call ready-for-trigger");
	}
};

// 双目的化，只需要计算一个最优曝光时间即可，然后将计算结果发给从机即可
void UEyeCamNodelet::sendSlaveExposure()
{
	if(cam_params_.adaptive_exposure_mode_ == 2)
	{
		ueye_cam::Exposure msg;
		msg.header.stamp = ros::Time::now();
		//msg.header.seq = ros_frame_count_ + 1; // TODO : This won't work

		msg.exposure_ms = adaptive_exposure_ms_;
		//msg.frame_sequence = ros_frame_count_ + 1;
		//INFO_STREAM("master sent exposure value is: " << adaptive_exposure_ms_);
		ros_exposure_pub_.publish(msg);
	}
};

unsigned int UEyeCamNodelet::stampAndPublishImage(unsigned int index)
{
    // 检查具有index索引的图像在`timestamp_buffer_`里是否有与之对应的元素
	int timestamp_index = findInStampBuffer(index);
//    cout<<"timestamp_index: "<<timestamp_index<<endl;
    if (timestamp_index+1)
    {
		//ROS_INFO("found corresponding image from at index: %i", timestamp_index);
		// Copy corresponding images and time stamps
		sensor_msgs::Image image;
		sensor_msgs::CameraInfo cinfo;
		image = image_buffer_.at(index);
		cinfo = cinfo_buffer_.at(index);

		// copy trigger time// + half of the exposure time
        //! @attention @attention 触发时间+曝光时间的一半
        //adaptive_exposure_ms_ = 15;
		image.header.stamp = timestamp_buffer_.at(timestamp_index).frame_stamp + ros::Duration(adaptive_exposure_ms_/2000.0);
		cinfo.header = image.header;
		
//        NODELET_INFO_STREAM("trigger time nsec: " << timestamp_buffer_.at(timestamp_index).frame_stamp <<
//                            " cam time nsec: " << image.header.stamp);
        if(image_buffer_.size() != timestamp_buffer_.size())
        {
            INFO_STREAM("image size: " << image_buffer_.size() << ", stamp size: " << timestamp_buffer_.size());
        }
		// Publish image in ROS
		ros_cam_pub_.publish(image, cinfo);

    // compute optimal params for next image frame (in any case)
    //! @attention @attention 计算下一帧最有曝光时间，参考:VI-SENSOR论文
    //optimizeCaptureParams(image);
		
		// Publish Cropped images
		if (cam_params_.crop_image) publishCroppedImage(image);

		//INFO_STREAM("image_buffer size: " << image_buffer_.size() << ", cinfo_buffer size: " << cinfo_buffer_.size() << ", timestamp_buffer size: " << timestamp_buffer_.size());
		// Erase published images and used timestamp from buffer
		if (image_buffer_.size()) image_buffer_.erase(image_buffer_.begin() + index);
		if (cinfo_buffer_.size()) cinfo_buffer_.erase(cinfo_buffer_.begin() + index);
		if (timestamp_buffer_.size()) timestamp_buffer_.erase(timestamp_buffer_.begin() + timestamp_index);
		return 0;

	} else {
		return 1;
	}
};

int UEyeCamNodelet::findInStampBuffer(unsigned int index)
{
	// Check whether there is at least one image in image buffer
	if (image_buffer_.empty())
		return -1;

	// Check whether image in image buffer with index "index" has corresponding element in timestamp buffer
    //! @attention @attention 检查具有index索引的图像在`timestamp_buffer_`里是否有与之对应的元素
	unsigned int k = 0;
	
	// sequence based method
	while (k < timestamp_buffer_.size() && ros::ok()) {
		if (image_buffer_.at(index).header.seq == ((uint)timestamp_buffer_.at(k).frame_seq_id - stamp_buffer_offset_)) {
			//INFO_STREAM("Found match k=" << k << ", index=" << index << "! image seq: " << image_buffer_.at(index).header.seq << ", buffer header seq: " << ((uint)timestamp_buffer_.at(k).frame_seq_id));
			return k;

		} else {
			k += 1;
		}
	}

	return -1;
};

void UEyeCamNodelet::publishRectifiedImage(const sensor_msgs::Image &frame)
{
	
	cv_bridge::CvImagePtr cv_ptr;

	try {
		cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::MONO8);

	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	// Rectify image
	cv::Mat frame_rect;
	camera_model_.rectifyImage(cv_ptr->image, frame_rect, cv::INTER_LINEAR);
	
	// crop
	float image_size_width = 752;
	float image_size_height = 480; 
	float percent = 0.4;
	cv::Mat frame_rect_cropped;
	resize(frame_rect (cv::Rect (image_size_width*(1-percent)/2, 
				image_size_height*(1-percent)/2, 
				image_size_width*percent, 
				image_size_height*percent)),
		frame_rect_cropped, 
		cv::Size(image_size_width, image_size_height));

	// Publish rectified image
	//sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(frame.header, frame.encoding, frame_rect).toImageMsg();
	sensor_msgs::ImagePtr rect_msg = cv_bridge::CvImage(frame.header, frame.encoding, frame_rect_cropped).toImageMsg();
	ros_rect_pub_.publish(rect_msg);

};


void UEyeCamNodelet::publishCroppedImage(const sensor_msgs::Image& frame)
{
	
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::MONO8);

	} catch (cv_bridge::Exception &e) {
		NODELET_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	
	// crop
	float image_size_width = 752;
	float image_size_height = 480; 
	frame_cropped_ = cv_ptr->image(cv::Rect ((cam_params_.image_width-image_size_width)/2, (cam_params_.image_height-image_size_height)/2, image_size_width, image_size_height));

	// Publish cropped image
	sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(frame.header, frame.encoding, frame_cropped_).toImageMsg();
	ros_cropped_pub_.publish(image_msg);

};


void UEyeCamNodelet::optimizeCaptureParams(sensor_msgs::Image image)
{
    // 如果是主机，则计算自适应曝光时间
    if(cam_params_.adaptive_exposure_mode_ == 2 && (ros_frame_count_ % 5 == 0) ) //每5帧进行一次
    {
    	// Compute the histogram
    	int histSize = 256;
    	float range[] = { 0, 256 } ;
    	const float *histRange = { range };
    	cv::Mat hist;
    	if(cam_params_.crop_image) {
    		cv::calcHist(&frame_cropped_, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);
    	} else {
    		cv_bridge::CvImagePtr cv_ptr;
    		try {
      			cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::MONO8);

    		} catch (cv_bridge::Exception &e) {
      			NODELET_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}
    		cv::calcHist(&cv_ptr->image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);
    		//cv::cuda::GpuMat src_gpu(cv_ptr->image), hist_gpu;
    	}
		
    	//cv::cuda::histEven(src_gpu, hist_gpu, histSize, (int) range[0], (int) range[1]);
    	//hist_gpu.download(hist);
    	cv::normalize(hist, hist, 1.0, 0, cv::NORM_L1); // TODO : check normalization

    	// Calculate mean sample value
		double j = 0, k = 0;
		double blocksum = 0;
		for (int i = 1; i <= histSize; i++) {
			blocksum += hist.at<float>(i-1);
			if(i % 51 == 0) {
				j += (i/51) * blocksum;
				k += blocksum;
				blocksum = 0;
			}
		}
		double msv = j / k;

		// TODO parameterize this 
        //! @attention 设置参数
		double setpoint = 3.0;//2.4;
		double deadband = 1.0;
        double adaptive_exposure_max_ = 20.0; //ms, to make sure not skipping frames, since there is readout time for image from ueye cam manual.
		double adaptive_exposure_min_ = 0.0; // 0.1
		double rate_max = 1.2;
	
		// Amount of change to the shutter speed or aperture value can be
		// calculated directly from the histogram as the five regions
		// of the histogram represent five f-stops. Each time the
		// shutter time is doubled/halved the image exposure will
		// decrease/increase with one f-stop.

		// Calculate exposure durations
		double error = msv-setpoint;
		if (error > deadband && error < rate_max) {	// overexposed
			adaptive_exposure_ms_ *= 1.0/error;
		
		} else if (error >= rate_max) {
			adaptive_exposure_ms_ *= 1.0/rate_max;

		} else if (error < -deadband && error > -rate_max) {	// underexposed
			adaptive_exposure_ms_ *= -error;

		} else if (error <= -rate_max) {
			adaptive_exposure_ms_ *= rate_max;
		}
		
		// limit exposure timing
		if (adaptive_exposure_ms_ > adaptive_exposure_max_) { 
			adaptive_exposure_ms_ = adaptive_exposure_max_; 
		} else if (adaptive_exposure_ms_ < adaptive_exposure_min_) {
			adaptive_exposure_ms_ = adaptive_exposure_min_;
		}

		//ROS_INFO_STREAM("j = " << j << "k = " << k << "msv = " << msv << ", exposure = " << adaptive_exposure_ms_);
		//ROS_INFO_STREAM("adaptive_exposure_ms_ is " << adaptive_exposure_ms_);

		// Set optimal exposure
		
		bool auto_exposure = false;
		if (setExposure(auto_exposure , adaptive_exposure_ms_) != IS_SUCCESS) {
			ROS_ERROR("Master adaptive exposure setting failed");
		}
		//ROS_WARN("exposure setpoint = %f", adaptive_exposure_ms_);
		// Send exposure message for slave
		sendSlaveExposure();
	}

};


void UEyeCamNodelet::adaptiveSync()
{
	if (image_buffer_.size() && timestamp_buffer_.size()) {
		// adaptive sync
		// frontior shift mean.
		double a = 0.9;
		stamp_buffer_offset_double_ = a*stamp_buffer_offset_double_ + (1-a)*((double)(timestamp_buffer_.end()-1)->frame_seq_id - (double)(image_buffer_.end()-1)->header.seq);
		int correction = (int)stamp_buffer_offset_double_ - stamp_buffer_offset_;
		// adjust the sequence offset accordingly
		if (correction > 0) {
			stamp_buffer_offset_ ++; // gradually increase the offset
			ROS_INFO_STREAM("[ " << cam_name_ << " ] correction is: " << correction << ", " << (timestamp_buffer_.end()-1)->frame_seq_id << ", " << (image_buffer_.end()-1)->header.seq); // tested about -0.05s
			ROS_INFO_STREAM("[ " << cam_name_ << " ] Time sequence shift detected, now trigger stamp starting sequence increase to: " << stamp_buffer_offset_);
			ROS_INFO_STREAM("[ " << cam_name_ << " ] image_buffer size: " << image_buffer_.size() << ", cinfo_buffer size: " << cinfo_buffer_.size() << ", timestamp_buffer size: " << timestamp_buffer_.size());
		} else if (correction < 0) {
			stamp_buffer_offset_ --; // gradually decrease the offset
			ROS_INFO_STREAM("[ " << cam_name_ << " ] correction is: " <<correction); // tested about -0.05s
			ROS_INFO_STREAM("[ " << cam_name_ << " ] Time sequence shift detected, now trigger stamp starting sequence decrease to: " << stamp_buffer_offset_);
			ROS_INFO_STREAM("[ " << cam_name_ << " ] image_buffer size: " << image_buffer_.size() << ", cinfo_buffer size: " << cinfo_buffer_.size() << ", timestamp_buffer size: " << timestamp_buffer_.size());
		}
	}
};

} // namespace ueye_cam


// TODO: 9 bug: when binning (and suspect when subsampling / sensor scaling), white balance / color gains seem to have different effects


#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(ueye_cam, ueye_cam_nodelet, ueye_cam::UEyeCamNodelet, nodelet::Nodelet)
