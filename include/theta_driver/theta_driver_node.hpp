#ifndef THETA_DERIVER_NODE_HPP
#define THETA_DERIVER_NODE_HPP

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <libuvc/libuvc.h>
#include "thetauvc.h"
#include <opencv2/core.hpp>

namespace theta_driver {

class ThetaDriverNode : public rclcpp::Node
{
public:
    ThetaDriverNode();
    virtual ~ThetaDriverNode();
    bool init();
    bool open();
    bool init_gst_buffer_pool(GstCaps* caps);
    void publishImage(GstMapInfo map);
    void publishCompressedImage(GstMapInfo map);

    bool streaming_ = false;
    uvc_device_handle_t* devh_;
    uvc_stream_ctrl_t ctrl_;
    uvc_context_t* ctx_;
    bool use4k_ = false;
    std::string serial_ = "";
    std::string camera_frame_ = "camera_theta";
    std::string pipeline_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_compressed_;
};

struct gst_src {
    GstElement* pipeline;
    GstElement* appsrc;
    GstBufferPool *pool;

    GMainLoop* loop;
    GTimer* timer;
    int framecount = 0;
    int fps = 30;
    guint id;
    guint bus_watch_id;
    uint32_t dwFrameInterval;
    uint32_t dwClockFrequency;
};

} // namespace theta_driver

#endif // THETA_DERIVER_NODE_HPP
