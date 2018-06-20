#ifndef WEITU_H
#define WEITU_H

#include <iostream>
#include <chrono>
#include "GenICam/CAPI/SDK.h"
#include <string>
#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../src/finder/finderpatternfinder.h"
#include "../src/edcircle/edcircle.h"

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }
    void out(std::string message = ""){
        double t = elapsed();
        std::cout << message << "  elasped time:" << t << "s" << std::endl;
        reset();
    }
private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

namespace weitu {

class Camera{
public:
    bool open(uint32_t i=0);
    cv::Mat get();
    void close();
    ~Camera()
    {
        close();
    }
private:
    bool open_flag = false;

    GENICAM_Camera *pCamera = NULL;
    GENICAM_StreamSource *pStreamSource = NULL;
};

}

namespace qr_pattern {

std::vector<FinderPattern> find(cv::Mat src);

}

namespace hole_detect {

cv::Point find(cv::Mat src, bool denoise = false);
std::vector<double> find_hole(double z, double timeout = 3, int cam_id = 0);
}


#endif
