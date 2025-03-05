#ifndef CAMERAMANAGER_HPP
#define CAMERAMANAGER_HPP

#include <atomic>
#include <functional>
#include <gst/gst.h>
#include <gst/app/app.h>
#include <opencv2/opencv.hpp>

class Video {
    public:
        std::atomic<cv::Mat*> atomicFrame;
        GError *error;
        GstElement *pipeline;
        GstElement *sink;

        Video();
        ~Video();
        GstFlowReturn new_preroll(GstAppSink* appsink, gpointer data);
        GstFlowReturn new_sample(GstAppSink* appsink, gpointer data);
        static gboolean my_bus_callback(GstBus* bus, GstMessage* message, gpointer data);
};

#endif