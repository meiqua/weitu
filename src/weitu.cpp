#include "weitu.h"
#include "seg/seg.h"
namespace weitu
{
#define INFINITE 0xFFFFFFFF // Infinite timeout
#define CREATE_SUSPENDED 0x00000004

static int32_t GENICAM_connect(GENICAM_Camera *pGetCamera)
{
    int32_t isConnectSuccess;

    isConnectSuccess = pGetCamera->connect(pGetCamera, accessPermissionControl);

    if (isConnectSuccess != 0)
    {
        printf("connect cameral failed.\n");
        return -1;
    }

    return 0;
}

static int32_t GENICAM_CreateStreamSource(GENICAM_Camera *pGetCamera, GENICAM_StreamSource **ppStreamSource)
{
    int32_t isCreateStreamSource;
    GENICAM_StreamSourceInfo stStreamSourceInfo;

    stStreamSourceInfo.channelId = 0;
    stStreamSourceInfo.pCamera = pGetCamera;

    isCreateStreamSource = GENICAM_createStreamSource(&stStreamSourceInfo, ppStreamSource);

    if (isCreateStreamSource != 0)
    {
        printf("create stream obj  fail.\r\n");
        return -1;
    }

    return 0;
}

static int32_t GENICAM_startGrabbing(GENICAM_StreamSource *pStreamSource)
{
    int32_t isStartGrabbingSuccess;
    GENICAM_EGrabStrategy eGrabStrategy;

    eGrabStrategy = grabStrartegySequential;
    isStartGrabbingSuccess = pStreamSource->startGrabbing(pStreamSource, 0, eGrabStrategy);

    if (isStartGrabbingSuccess != 0)
    {
        printf("StartGrabbing  fail.\n");
        return -1;
    }

    return 0;
}

static int32_t GENICAM_stopGrabbing(GENICAM_StreamSource *pStreamSource)
{
    int32_t isStopGrabbingSuccess;

    isStopGrabbingSuccess = pStreamSource->stopGrabbing(pStreamSource);
    if (isStopGrabbingSuccess != 0)
    {
        printf("StopGrabbing  fail.\n");
        return -1;
    }

    return 0;
}

static int32_t modifyCamralExposureTime(GENICAM_Camera *pGetCamera)
{
    int32_t isExposureTimeSuccess;
    GENICAM_DoubleNode doubleNode;
    double exposureTimeValue;
    GENICAM_AcquisitionControl *pAcquisitionCtrl = NULL;
    GENICAM_AcquisitionControlInfo acquisitionControlInfo = {0};

    acquisitionControlInfo.pCamera = pGetCamera;

    isExposureTimeSuccess = GENICAM_createAcquisitionControl(&acquisitionControlInfo, &pAcquisitionCtrl);
    if (isExposureTimeSuccess != 0)
    {
        printf("ExposureTime  fail.\n");
        return -1;
    }

    exposureTimeValue = 0.0;
    doubleNode = pAcquisitionCtrl->exposureTime(pAcquisitionCtrl);

    isExposureTimeSuccess = doubleNode.getValue(&doubleNode, &exposureTimeValue);
    if (isExposureTimeSuccess != 0)
    {
        printf("get exposureTime fail.\n");
        return -1;
    }
    else
    {
        printf("before change ,exposureTime is %f\n", exposureTimeValue);
    }

    doubleNode.setValue(&doubleNode, (exposureTimeValue + 2));
    if (isExposureTimeSuccess != 0)
    {
        printf("set exposureTime fail.\n");
        return -1;
    }

    doubleNode.getValue(&doubleNode, &exposureTimeValue);
    if (isExposureTimeSuccess != 0)
    {
        printf("get exposureTime fail.\n");
        return -1;
    }
    else
    {
        printf("after change ,exposureTime is %f\n", exposureTimeValue);
    }

    return 0;
}

static int32_t GENICAM_disconnect(GENICAM_Camera *pGetCamera)
{
    int32_t isDisconnectSuccess;

    isDisconnectSuccess = pGetCamera->disConnect(pGetCamera);
    if (isDisconnectSuccess != 0)
    {
        printf("disconnect fail.\n");
        return -1;
    }

    return 0;
}

bool Camera::open(uint32_t i)
{
    int32_t ret;
    GENICAM_System *pSystem = NULL;
    GENICAM_Camera *pCameraList = NULL;
    uint32_t cameraCnt = 0;

    ret = GENICAM_getSystemInstance(&pSystem);
    if (-1 == ret)
    {
        printf("pSystem is null.\r\n");
        return false;
    }

    ret = pSystem->discovery(pSystem, &pCameraList, &cameraCnt, typeAll);
    if (-1 == ret)
    {
        printf("discovery device fail.\r\n");
        return false;
    }

    if (cameraCnt < i + 1)
    {
        printf("no enough Camera is discovered.\r\n");
        return false;
    }

    pCamera = &pCameraList[i];
    // connect to camera
    //连接设备
    ret = GENICAM_connect(pCamera);
    if (ret != 0)
    {
        printf("connect cameral failed.\n");
        return false;
    }

    // create stream source instance
    //创建流对象
    ret = GENICAM_CreateStreamSource(pCamera, &pStreamSource);
    if (ret != 0)
    {
        printf("create stream obj  fail.\r\n");
        return false;
    }

    ret = GENICAM_startGrabbing(pStreamSource);
    if (ret != 0)
    {
        printf("StartGrabbing  fail.\n");
        return false;
    }
    open_flag = true;
    return true;
}

cv::Mat Camera::get()
{
    if (!open_flag)
    {
        return cv::Mat();
    }

    int32_t ret = -1;
    GENICAM_Frame *pFrame;

    if (NULL == pStreamSource)
    {
        return cv::Mat();
    }

    ret = pStreamSource->getFrame(pStreamSource, &pFrame, 100);
    if (ret < 0)
    {
        printf("getFrame  fail.\n");
        return cv::Mat();
    }

    ret = pFrame->valid(pFrame);
    if (ret < 0)
    {
        printf("frame is invalid!\n");
        pFrame->release(pFrame);
        return cv::Mat();
    }
    //    printf("get frame id = [%ld] successfully!\n", pFrame->getBlockId(pFrame));

    uint32_t rows = pFrame->getImageHeight(pFrame);
    uint32_t cols = pFrame->getImageWidth(pFrame);
    const void *img = pFrame->getImage(pFrame);
    char dest[rows * cols];
    std::memcpy(dest, img, sizeof dest);
    cv::Mat result = cv::Mat(rows, cols, CV_8UC1, dest);
    pFrame->release(pFrame);

    return result;
}

void Camera::close()
{

    if (open_flag)
    {
        // stop grabbing from camera
        GENICAM_stopGrabbing(pStreamSource);

        //close camera
        GENICAM_disconnect(pCamera);

        // close stream
        pStreamSource->release(pStreamSource);
        open_flag = false;
    }
}

} // namespace weitu

namespace qr_pattern
{
std::vector<FinderPattern> find(cv::Mat src)
{
    cv::Mat graySrc;
    if (src.channels() > 1)
    {
        cv::cvtColor(src, graySrc, CV_BGR2GRAY);
    }
    else
    {
        graySrc = src;
    }
    graySrc.convertTo(graySrc, CV_8UC1);

    cv::Mat binarySrc;
    int blockSize = graySrc.rows / 8;
    if (blockSize % 2 == 0)
        blockSize++;

    cv::threshold(graySrc, binarySrc, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    // cv::threshold(graySrc, binarySrc, 127, 255, CV_THRESH_BINARY);
    // cv::adaptiveThreshold(graySrc, binarySrc, 255, cv::ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, blockSize, 0);

    // cv::pyrDown(binarySrc, binarySrc);
    // cv::pyrDown(binarySrc, binarySrc);
    // cv::imshow("binary", binarySrc);
    // cv::waitKey(1);

    FinderPatternFinder finder(binarySrc);
    finder.find();

    std::vector<FinderPattern> pattern = finder.possibleCenters;
    std::sort(pattern.begin(), pattern.end());
    return pattern;
}
} // namespace qr_pattern

namespace hole_detect
{

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v)
{

    // initialize original index locations
    std::vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) { return v[i1].size() > v[i2].size(); });

    return idx;
}

cv::Point find(cv::Mat src, bool denoise, bool hough)
{
    bool vis_result = true;
    bool vis_center = true;

    // GaussianBlur(src, src, cv::Size(3, 3), 3);

    cv::Mat rgb = src.clone();
    if (src.channels() == 3)
    {
        cv::cvtColor(src, src, CV_BGR2GRAY);
    }
    else
    {
        cv::cvtColor(src, rgb, CV_GRAY2BGR);
    }

    if (denoise)
    {
        seg_helper::min_span_tree::Graph graph(src);

        Segmentation seg(src, graph.mst_edges);
        auto lvs = seg.process();

        int i = 0;
        //    for(int i=0;i<lvs.size();i++)
        if (!lvs.empty())
        {
            auto &lv = lvs[i];

            cv::Mat ave_rgb = cv::Mat(rgb.size(), CV_8UC3, cv::Scalar(0));

            for (auto &part : lv)
            {
                cv::Vec3d aveColor(0, 0, 0);
                int count = 0;
                for (int idx : part)
                {
                    int row = idx / int(rgb.cols);
                    int col = idx % int(rgb.cols);
                    aveColor += rgb.at<cv::Vec3b>(row, col);
                    count++;
                }
                aveColor /= count;
                for (int idx : part)
                {
                    int row = idx / int(rgb.cols);
                    int col = idx % int(rgb.cols);
                    ave_rgb.at<cv::Vec3b>(row, col) = aveColor;
                }
            }
            cv::cvtColor(ave_rgb, src, CV_BGR2GRAY);

            for(int i=0; i<3; i++)
            medianBlur(src, src, 3);
            // cv::threshold(src, src, 0, 255, CV_THRESH_OTSU);
//             GaussianBlur(src, src, cv::Size(3, 3), 3);
        }
    }

    std::vector<cv::Point> centers;
    std::vector<float> radiuses;

    if(hough){
         std::vector<cv::Vec3f> circles;
         cv::HoughCircles( src, circles, CV_HOUGH_GRADIENT, 1, src.rows/100,
                           50, 30, 0, src.rows/5);
         for(auto circle: circles){
             cv::Point p = {int(circle[0]), int(circle[1])};
             centers.push_back(p);
             radiuses.push_back(circle[2]);
         }
    }else{
        centers = edcircle::find_circle(src, radiuses);
    }

    if (centers.empty())
    {
        if (vis_result)
        {
            cv::Mat to_show = src;
            cv::resize(to_show, to_show, {to_show.cols * 1024 / to_show.rows, 1024});
            cv::imshow("hole detect", to_show);
        }
        return cv::Point(0, 0);
    }

    cv::Point c = {src.cols / 2, src.rows / 2};
    cv::Point best_p;
    float best_r = 0;
    double closest = std::numeric_limits<double>::max();
    for (int i = 0; i < centers.size(); i++)
    {
        if(radiuses[i]>src.rows/100){
            auto &p = centers[i];
            auto p2c = p - c;
            double dist = p2c.x * p2c.x + p2c.y * p2c.y;
            if (dist < closest)
            {
                closest = dist;
                best_p = p;
                best_r = radiuses[i];
            }
        }
    }

    cv::Point best_p_smallest = best_p;
    float smallest_r = best_r;
    for (int i = 0; i < centers.size(); i++)
    {
        if(radiuses[i]>src.rows/100){
            auto &p = centers[i];
            auto p2c = p - best_p;
            double dist = std::sqrt(p2c.x * p2c.x + p2c.y * p2c.y);
            if (dist < src.rows / 10)
            {
                if(radiuses[i] < smallest_r){
                    smallest_r = radiuses[i];
                    best_p_smallest = p;
                }
            }
        }
    }

    best_p = best_p_smallest;
    best_r = smallest_r;

    if (vis_result)
    {
        cv::Mat to_show;
        cv::cvtColor(src, to_show, CV_GRAY2BGR);
         for (int i = 0; i < centers.size(); i++)
         {
             if(radiuses[i]>src.rows/100)
             cv::circle(to_show, centers[i], radiuses[i], {0, 0, 255}, 2);
         }
         cv::line(to_show, cv::Point(best_p.x, best_p.y - best_r / 2), cv::Point(best_p.x, best_p.y + best_r / 2), cv::Scalar(0, 255, 0), 2);
         cv::line(to_show, cv::Point(best_p.x - best_r / 2, best_p.y), cv::Point(best_p.x + best_r / 2, best_p.y), cv::Scalar(0, 255, 0), 2);

         if (vis_center)
         {
             cv::circle(to_show, {to_show.cols / 2, to_show.rows / 2}, 1, {255, 255, 0}, 2);
             cv::circle(to_show, {to_show.cols / 2, to_show.rows / 2}, best_r / 2, {255, 255, 0}, 2);
             cv::line(to_show, {to_show.cols / 2, to_show.rows / 2}, cv::Point(best_p.x, best_p.y), cv::Scalar(0, 255, 255), 1);
         }
        cv::resize(to_show, to_show, {to_show.cols * 1024 / to_show.rows, 1024});
        cv::imshow("hole detect", to_show);
    }
    return best_p;
}

std::vector<double> find_hole(double z, double timeout, int cam_id)
{
    float K_data[] = {
        5502.067210, 0.000000, 1184.077158,
        0.000000, 5481.872057, 1098.097571,
        0.000000, 0.000000, 1.000000};
    float D_data[] = {
        -0.338621, 0.168072, -0.001953, -0.001642, 0.000000};
    cv::Mat K_pnp = cv::Mat(3, 3, CV_32FC1, K_data);
    cv::Mat D_pnp = cv::Mat(1, 5, CV_32FC1, D_data);

    std::vector<double> xy;
    cv::Point hole_img_point = {0, 0};

    Timer timer;
    weitu::Camera camera;
    camera.open(cam_id);

    int start_cols = 0;
    int start_rows = 0;
    while (timer.elapsed() < timeout)
    {
        cv::Mat rgb = camera.get();
        if (!rgb.empty())
        {
            // cv::undistort(rgb, rgb, K_pnp, D_pnp);
            start_cols = rgb.cols / 4;
            start_rows = rgb.rows / 4;
            cv::Rect roi(start_cols, start_rows, rgb.cols / 2, rgb.rows / 2);
            // cv::pyrDown(rgb, rgb);
            hole_img_point = hole_detect::find(rgb(roi), false);
            if (hole_img_point.x == 0)
                continue;
            break;
        }
        else
        {
            std::cout << "cam no img" << std::endl;
        }
    }
    if (hole_img_point.x > 0)
    {
        cv::Point &p = hole_img_point;
        float img_p_data[] = {float(p.x + start_cols), float(p.y + start_rows), 1.0f};
        cv::Mat img_p = cv::Mat(3, 1, CV_32FC1, img_p_data);
        cv::Mat world_p = K_pnp.inv() * img_p;
        double factor_s = z / world_p.at<float>(2, 0);
        xy.push_back(world_p.at<float>(0, 0) * factor_s);
        xy.push_back(world_p.at<float>(1, 0) * factor_s);
    }
    return xy;
}
} // namespace hole_detect
