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

#include "src/finder/finderpatternfinder.h"
#include "src/edcircle/edcircle.h"

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

cv::Point find(cv::Mat src);

}

#endif
