![logo](docs/resources/ARU_logo_rectangle.png)
# Ximea Camera ROS2 Driver

This repo aims provides a generic [ROS2](https://docs.ros.org/en/foxy/index.html) driver for the XIMEA MQ022CG-CM camera. The repo was adapted from [wavelab's](https://github.com/wavelab/ximea_ros_cam) ROS1 version. And so credit is given to this version for much of the code and convention in this driver. 

![Slide86](docs/resources/ximea.jpeg)

<hr/> 

## Prerequisites

### ROS2

- Tested on [ROS2 foxy](https://docs.ros.org/en/foxy/Installation.html)

- Using [eCAL RWM](https://github.com/eclipse-ecal/rmw_ecal) as an alternative to ROS2 DDS implementations showed significant perfomance improvements
  
### Ximea SDK

Installed and tested on Ubuntu 20.04 LTS.

<b> OPTIONAL: </b>For full installation and setup, you could simply run the bash script: [ximea_install.sh](ximea_ros2_cam/docs/installation/ximea_install.sh). Make sure you have the limits.txt file in the same directory. Also checkout the [docker file](ximea_ros2_cam/docs/docker/XIMEA.Dockerfile) for possible use with ROS2 Humble using the [nvidia-isaac-ros](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common) docker workspace.

Download and extract the most recent software package:
```bash
$ cd ~; mkdir tmp; cd tmp
$ wget https://www.ximea.com/downloads/recent/XIMEA_Linux_SP.tgz
```
```bash
$ tar xzf XIMEA_Linux_SP.tgz
$ cd package
```
Install the package depending on you camera type
```bash
$ ./install -<type>
```
types:

    -cam_usb30
    -pcie

#### Add user to the pugdev group:
```bash
$ sudo gpasswd -a $USER plugdev
```
#### Set the USB FS memory allocation to infinite for sufficient buffering size for high bandwith USB3.0 streams:
```bash
$ echo 0 > /sys/module/usbcore/parameters/usbfs_memory_mb
# TODO: THIS COMMAND NEED SUDO FOR EXECUTION, AVOID THIS OF CONFIGURE. 
# You can put this line to your bashrc file to apply to every new shell or use 
sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null <<<0
```
#### Set realtime priority by putting the following to ``` /etc/security/limits.conf ```:
```bash
*               -       rtprio          0
@realtime       -       rtprio          81
*               -       nice            0
@realtime       -       nice            -16
```
then add the user to realtime
```bash
$ sudo groupadd realtime #if it doesn't exist yet
$ sudo gpasswd -a $USER realtime
```
You may need to reboot your system for some changes to take effect.

## Running the ros package

clone this repo to your ros2 workspace source directory:

```bash
$ cd ~/ros2_ws
$ git clone https://github.com/African-Robotics-Unit/ximea_ROS2_driver.git
$ cd ~/ros2_ws/ximea_ROS2_driver
$ colcon build --packages-select ximea_ros2_cam
```

Modify camera parameters as desired in the [config](ximea_ros2_cam/config/xiCam_config.yaml) and [launch](ximea_ros2_cam/launch/xiCam.launch.xml) files, and run the node using the provided launch file or a custom one. 

### Launching a single camera:
```bash
$ ros2 launch ximea_ros2_cam xiCam.launch.xml

# To view the stream
$ ros2 run rqt_image_view rqt_image_view
```

<b>NOTE:</b> Ximea API's demosaic color defect correction is not optimized for ARM processors, and so performance may degrade significantly. To avoid this you can instead capture raw 8-bits or 16-bits images (by setting the <i>format</i> parameter to <b>RAW8</b> or <b>RAW16</b> in the config file) and post-process later using the API's [offline processing](https://www.ximea.com/support/wiki/apis/XiAPI_Offline_Processing).

### Launch multiple cameras:

```bash
$ ros2 launch ximea_ros2_cam multipleCam.launch.py

# To view the stream
$ ros2 run rqt_image_view rqt_image_view
```

**When launching multiple cameras, be sure to launch them at separate times (~1-2 seconds apart), this is potentially due to USB resource hog issues.**

## Parameter Descriptions:

**Note that these parameters are found in either the config file or launch file**

### General

`serial_no` - Serial number of the Ximea camera (used to locate the proper camera)

`cam_name` - Name of the camera used when saving camera images and snapshots under the directory pointed by `image_directory`

`calib_file` - Calibration file used by the camera

`frame_id` - Frame ID of the camera

`num_cams_in_bus` - Number of USB cameras processed by a single USB controller (That is, if a hub has one controller and 4 ports, with 3 ports plugged with USB 3.0 cameras, then `num_cams_in_bus = 3`) (This will divide the total USB bandwidth by `num_cams_in_bus` to ensure equal bandwidth for each camera)

`cam_downsampling` - Downsampling config, changes image resolution by binning or skipping
```
XI_DWN_1x1 - 1 sensor pixel = 1 image pixel
XI_DWN_2x2 - 2x2 sensor pixels = 1 image pixel
XI_DWN_3x3 - Downsampling 3x3
XI_DWN_4x4 - Downsampling 4x4 (indicated for the used cameras) 
XI_DWN_5x5 - Downsampling 5x5
XI_DWN_6x6 - Downsampling 6x6
XI_DWN_7x7 - Downsampling 7x7
XI_DWN_8x8 - Downsampling 8x8
XI_DWN_9x9 - Downsampling 9x9
XI_DWN_10x10 - Downsampling 10x10
XI_DWN_16x16 - Downsampling 16x16
```

`cam_downsampling_type` - Changes image downsampling type (binning or skipping).
```
XI_BINNING	pixels are interpolated - better image
XI_SKIPPING	pixels are skipped - higher frame rate
```

`bw_safetyratio` - Bandwidth safety ratio, a multiplier to the bandwidth allocated for each camera

`poll_time` - Used to set the duration (in seconds) which the camera is attempted to be opened again. When using multiple cameras, a duration of 2 seconds between each camera is recommended. (i.e. `poll_time=0.0` and `poll_time=2.0` for cameras 1 and 2).

`poll_time_frame` - This is the ROS timer loop period for the ximea camera node. It should generally be set to a rate that is a factor higher than the camera capture rate. For example, if the camera runs at 20Hz (or 0.05s period) and `poll_time_frame` set to 0.001s, then the timer will constantly loop every 0.001 seconds which is faster than the 0.05s period of the camera capture time, with an error that is roughly 0.000 to 0.002 between frames. **Warning: if this value is larger than the period of the camera capture rate, then the frame rate of the camera is capped to the rate of the `poll_time_frame`, for example, if `poll_time_frame` is 0.5s, then the maximum rate that the camera can achieve is 2Hz**

`publish_xi_image_info` - Flag for publishing the extra ximea camera information provided with each image acquisition.

### Image Saving

`image_directory` - Directory used by the flag `save_disk` and `calib_mode` (**Note: Must be a valid directory path with an ABSOLUTE path, otherwise the camera fails to launch or will not create the directory properly. **)

`save_disk` - Save images to disk, under the directory `<image_directory>/stream` 

`calib_mode` - Saves images everytime a trigger is pressed, under the director `<image_directory>/calib`

### Compressed Image Transport Parameters

See [compressed_image_transport](http://wiki.ros.org/compressed_image_transport) for more information.

`image_transport_compressed_format` -  Format for the compressed image - `jpg` or `png`

`image_transport_compressed_jpeg_quality` - 1 to 100 (1 = min quality)

`image_transport_compressed_png_level` - 1 to 9 (9 = max compression)

### Colouring

`format` - Image format: 
```
XI_MONO8 - Grayscale 8 bit
XI_MONO16 - Grayscale 16 bit
XI_RGB24 - BGR 24 bit
XI_RGB32 - BGRA 32 bit
XI_RGB_PLANAR - NOT USED
XI_RAW8 - RAW 8 bit
XI_RAW16 - RAW 16 bit
```

`white_balance_mode` - 0 = none, 1 = use coefficients, 2 = auto

`white_balance_coef_red` - manual white balance red coefficient (0 to 8)

`white_balance_coef_green` - manual white balance green coefficient (0 to 8)

`white_balance_coef_blue` - manual white balance blue coefficient (0 to 8)

### Frame Rate Control

`cam_trigger_mode` - 0 = none, 2 = hardware trigger

`hw_trigger_edge` - For hardware trigger mode, 0 = rising-edge trigger, 1 = falling-edge trigger

`frame_rate_control` - Camera frame rate control (Works if no triggering is enabled)

`frame_rate_set` - FPS limiter (0 for none). This is depending on available bandwidth

`img_capture_timeout` - Timeout for getting an image, in milliseconds

### Exposure Settings

`auto_exposure` - Auto exposure

`auto_exposure_priority` - Auto exposure gain ratio (1 = favour only exposure)

`auto_time_limit` - Auto exposure time limit (microseconds)

`auto_gain_limit` - Auto exposure gain limit

`exposure_time` - Manual exposure time (microseconds)

`manual_gain` - Manual exposure gain

### Region of Interest

`roi_top`, `roi_left` - Top left corner in pixels

`roi_width`, `roi_height` - Width, height in pixels
