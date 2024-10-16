#ifndef XIMEA_ROS_CAM_XIMEAROSCAM_HPP
#define XIMEA_ROS_CAM_XIMEAROSCAM_HPP

// ROS2 Stuff
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/fill_image.hpp"

// Camera Stuff
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"

// Ximea API
#include <m3api/xiApi.h>

// Other includes
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <yaml-cpp/yaml.h>
#include <sys/stat.h>
#include <memory>

// OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/core.hpp>

namespace ximea_ros2_cam {

const int kMaxNumberImages = 10;

class XimeaROSCam : public rclcpp::Node, std::enable_shared_from_this<XimeaROSCam> {
 public:
    XimeaROSCam();
    ~XimeaROSCam();

 private:
   /**
    * @brief Initialized all parameters
    * 
    */
   void initParams();
   /**
    * @brief  Initialize all timers for event callbacks.
    */
   void initTimers();

   /**
    * @brief Initializes the camera using API parameters
    *   initializes all required publishers
    */
   void initCam();

   /**
    * @brief Initiates camera capture
    *    
    */
   void openCam();
   
   /**
    * @brief Publishes an image resized with openCV
    * 
    * This helps maintain the camera's FOV and avoids the cropping nature of the
    * built-in ROI features
    * 
    * @param img_buffer : raw image matrix
    * @param xi_img     : Ximea API image stucture
    * @param timestamp  : ROS timestamp
    */
   void publishOpenCVResized(char* img_buffer,
                        XI_IMG xi_img,
                        rclcpp::Time timestamp);

   void publishOpenCVInverted(char* img_buffer,
                        XI_IMG xi_img,
                        rclcpp::Time timestamp);

   // Configuration Maps
   static std::map<std::string, int> ImgFormatMap;  // Maps image format names to their corresponding integer values.
   static std::map<std::string, int> BytesPerPixelMap;  // Maps image format names to the number of bytes per pixel.
   static std::map<std::string, std::string> ImgEncodingMap;  // Maps image format names to their corresponding encoding formats (e.g., JPEG, PNG).
   static std::map<int, int> CamMaxPixelWidth;  // Maps camera IDs to their maximum supported pixel widths.
   static std::map<int, int> CamMaxPixelHeight;  // Maps camera IDs to their maximum supported pixel heights.
   static std::map<std::string, int> DownsamplingMap;  // Maps downsampling factor names to their corresponding integer values.
   static std::map<std::string, int> DownsamplingTypeMap; // Maps downsampling factor names to their corresponding downsampling types.

   // ROS Timers
   rclcpp::TimerBase::SharedPtr xi_open_device_cb_;
   rclcpp::TimerBase::SharedPtr t_frame_cb_;

   // ROS Timer callbacks
   void openDeviceCb();
   void frameCaptureCb();

   bool debug_mode_print_;  // Turn on/off ROS debugs prints

   // ROS  Publishers
   rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;             // Cam info publisher handle
   rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_xi_image_info_pub_; // xiGetImage info publisher handle

   image_transport::CameraPublisher cam_pub_;            // Raw image publisher
   image_transport::CameraPublisher image_resized_pub_;  // Resized Image Publisher
   image_transport::CameraPublisher image_inverted_pub_;  // Inverted Image Publisher
   rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_pub_;

   // Camera context file path
   std::string cam_context_path_;

   // Camera variable list
   uint32_t img_count_;             
   bool cam_info_loaded_;
   std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;

   // OpenCV Resizing and Inverting
   bool cv_resize_enable_, cv_invert_enable_;
   uint32_t cv_height;
   uint32_t cv_width;
   cv::Mat img_mat_invert, img_mat_resize, img_resized, img_inverted;
   std_msgs::msg::Header resized_header, inverted_header;
   
   // compressed image params
   std::string cam_compressed_format_;      // "png" (lossless) or "jpeg"
   int cam_compressed_jpeg_quality_;        // 1-100 (1 = min quality)
   int cam_compressed_png_level_;           // 1-9 (9 = max compression)

   // camera params
   std::string cam_name_;                   // Main topic name for cam
   std::string cam_format_;                 // Camera image format
   int cam_format_int_;                     // Camera image format int val
   std::string cam_encoding_;               // Camera image encoding
   int cam_bytesperpixel_;                  // Camera image bytes per pixel
   std::string cam_serialno_;               // Camera serial no
   std::string cam_frameid_; // The unique ID assigned to the camera frame.
   float poll_time_;            // The time interval for launching cameras in succession (in seconds).
   float poll_time_frame_;                  // The time interval for checking each image buffer (in seconds).
   int cam_model_; // The model number of the camera.
   std::string cam_calib_file_; // The path to the camera calibration file.
   int cam_trigger_mode_; // The trigger mode of the camera (e.g., software, hardware).
   int cam_hw_trigger_edge_; // The edge of the hardware trigger signal (e.g., rising, falling).
   bool cam_autoexposure_; // Whether automatic exposure is enabled.
   int cam_exposure_time_; // The exposure time in microseconds.
   float cam_manualgain_; // The manual gain value.
   int cam_autotime_limit_; // The maximum time allowed for automatic exposure adjustment.
   float cam_autoexposure_priority_; // The priority of automatic exposure adjustment (higher value means higher priority).
   float cam_autogain_limit_; // The maximum gain value allowed for automatic gain adjustment.
   bool cam_binning_en_; // Whether binning is enabled.
   int cam_downsample_factor_; // The downsampling factor for the camera image.
   int cam_roi_left_; // The left coordinate of the region of interest (ROI).
   int cam_roi_top_; // The top coordinate of the ROI.
   int cam_roi_width_; // The width of the ROI.
   int cam_roi_height_; // The height of the ROI.
   bool cam_framerate_control_;   // framerate control - enable or disable
   int cam_framerate_set_;      // framerate control - setting fps
   int cam_img_cap_timeout_;       // max time to wait for img
   std::string cam_downsampling_;  // camera downsampling parameter
   int cam_downsampling_int_;  // camera downsampling correspondent int value
   std::string cam_downsampling_type_;  // camera downsampling type parameter
   int cam_downsampling_type_int_;  // camera downsampling type  correspondent int value

   // white balance mode: 0 - none, 1 - use coeffs, 2 = auto
   int cam_white_balance_mode_;
   float cam_white_balance_coef_r_; // white balance coefficient (rgb)
   float cam_white_balance_coef_g_;
   float cam_white_balance_coef_b_;

   // Bandwidth Limiting
   int cam_num_in_bus_;            // # cameras in a single bus
   float cam_bw_safetyratio_;        // ratio used based on a camera avail bw

   // Active variables
   HANDLE xi_h_;                   // camera xiAPI handle
   bool is_active_;                // camera actively acquiring images?
   float min_fps_;                 // camera calculated min fps
   float max_fps_;                 // camera calculated max fps

   // Focal Parameters
   int lens_mode_;
   float lens_aperture_value_;
   int lens_aperture_index_;
   int lens_focus_movement_value_;
   int lens_focus_move_;
   int lens_focal_length_;

};  // class XimeaROSCam

}  // namespace ximea_ros_cam

#endif  // XIMEA_ROS_CAM_XIMEAROSCAM_HPP
