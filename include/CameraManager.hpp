#ifndef CAMERA_MANAGER_HPP
#define CAMERA_MANAGER_HPP

#include <atomic>
#include <gst/gst.h>
#include <gst/app/app.h>
#include <opencv2/opencv.hpp>
#include <iostream>

class Video {
public:
    Video();
    ~Video();
    
    // Atomic frame access for thread safety
    std::atomic<cv::Mat*> atomicFrame;
    
private:
    // GStreamer pipeline and elements
    GstElement *pipeline;
    GstElement *sink;
    GError *error;
    
    // Callback methods
    GstFlowReturn new_preroll(GstAppSink* appsink, gpointer data);
    GstFlowReturn new_sample(GstAppSink* appsink, gpointer data);
    
    // Bus callback for GStreamer messages
    static gboolean my_bus_callback(GstBus *bus, GstMessage *message, gpointer data);
};

#endif // CAMERA_MANAGER_HPP