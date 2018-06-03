#include "weitu.h"

namespace weitu{
#define INFINITE            0xFFFFFFFF  // Infinite timeout
#define CREATE_SUSPENDED    0x00000004

static int32_t GENICAM_connect(GENICAM_Camera *pGetCamera)
{
	int32_t isConnectSuccess;

	isConnectSuccess = pGetCamera->connect(pGetCamera, accessPermissionControl);

	if( isConnectSuccess != 0)
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
	
	if( isCreateStreamSource != 0)
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

	if( isStartGrabbingSuccess != 0)
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
	if( isStopGrabbingSuccess != 0)
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
	if( isExposureTimeSuccess != 0)
	{
		printf("ExposureTime  fail.\n");
		return -1;
	}
	
	exposureTimeValue = 0.0;
	doubleNode = pAcquisitionCtrl->exposureTime(pAcquisitionCtrl);

	isExposureTimeSuccess = doubleNode.getValue(&doubleNode, &exposureTimeValue);
	if( isExposureTimeSuccess != 0)
	{
		printf("get exposureTime fail.\n");
		return -1;
	}
	else
	{
		printf("before change ,exposureTime is %f\n",exposureTimeValue);
	}	
	
	doubleNode.setValue(&doubleNode, (exposureTimeValue + 2));
	if( isExposureTimeSuccess != 0)
	{
		printf("set exposureTime fail.\n");
		return -1;
	}

	doubleNode.getValue(&doubleNode, &exposureTimeValue);
	if( isExposureTimeSuccess != 0)
	{
		printf("get exposureTime fail.\n");
		return -1;
	}
	else
	{
		printf("after change ,exposureTime is %f\n",exposureTimeValue);
	}

	return 0;
}

static int32_t GENICAM_disconnect(GENICAM_Camera *pGetCamera)
{
	int32_t isDisconnectSuccess;

	isDisconnectSuccess = pGetCamera->disConnect(pGetCamera);
	if( isDisconnectSuccess != 0)
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

    if(cameraCnt < i+1)
    {
        printf("no enough Camera is discovered.\r\n");
        return false;
    }

    pCamera = &pCameraList[i];
    // connect to camera
    //连接设备
    ret = GENICAM_connect(pCamera);
    if(ret != 0)
    {
        printf("connect cameral failed.\n");
        return false;
    }

    // create stream source instance
    //创建流对象
    ret = GENICAM_CreateStreamSource(pCamera, &pStreamSource);
    if(ret != 0)
    {
        printf("create stream obj  fail.\r\n");
        return false;
    }

    ret = GENICAM_startGrabbing(pStreamSource);
    if(ret != 0)
    {
        printf("StartGrabbing  fail.\n");
        return false;
    }
    open_flag = true;
    return true;
}

cv::Mat Camera::get()
{
    if(!open_flag){
        return cv::Mat();
    }

    int32_t ret = -1;
    GENICAM_Frame* pFrame;

    if(NULL == pStreamSource){
        return cv::Mat();
    }

    ret = pStreamSource->getFrame(pStreamSource, &pFrame, 1000);
    if (ret < 0){
        printf("getFrame  fail.\n");
        return cv::Mat();
    }

    ret = pFrame->valid(pFrame);
    if (ret < 0){
        printf("frame is invalid!\n");
        pFrame->release(pFrame);
        return cv::Mat();
    }
//    printf("get frame id = [%ld] successfully!\n", pFrame->getBlockId(pFrame));

    uint32_t rows = pFrame->getImageHeight(pFrame);
    uint32_t cols = pFrame->getImageWidth(pFrame);
    const void* img = pFrame->getImage(pFrame);
    char dest[rows*cols];
    std::memcpy(dest, img, sizeof dest);
    cv::Mat result = cv::Mat(rows, cols, CV_8UC1, dest);
    pFrame->release(pFrame);
    return result;
}

void Camera::close()
{
    if(open_flag){
        // stop grabbing from camera
        GENICAM_stopGrabbing(pStreamSource);

        //close camera
        GENICAM_disconnect(pCamera);

        // close stream
        pStreamSource->release(pStreamSource);
        open_flag = false;
    }
}

}

namespace qr_pattern {
std::vector<FinderPattern> find(cv::Mat src){
    cv::Mat graySrc;
    if(src.channels()>1){
       cv::cvtColor(src,graySrc,CV_BGR2GRAY);
    }else{
        graySrc = src;
    }
    graySrc.convertTo(graySrc,CV_8UC1);

    cv::Mat binarySrc;

//    cv::threshold(graySrc, binarySrc, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    cv::threshold(graySrc, binarySrc, 127, 255, CV_THRESH_BINARY);

    FinderPatternFinder finder(binarySrc);
    finder.find();

    std::vector<FinderPattern> pattern = finder.possibleCenters;
    return pattern;
}
}

namespace hole_detect {

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) {

  // initialize original index locations
  std::vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1].size() > v[i2].size();});

  return idx;
}


cv::Point find(cv::Mat src){
    if(src.channels()==3){
        cv::cvtColor(src, src, CV_BGR2GRAY);
    }

    std::vector<float> radiuses;
    auto centers = edcircle::find_circle(src, radiuses);

    if(centers.empty()) return cv::Point(0,0);

    cv::Point c = {src.cols/2, src.rows/2};
    cv::Point best_p;
    double closest = std::numeric_limits<double>::max();
    for(auto& p: centers){
        auto p2c = p-c;
        double dist = p2c.x*p2c.x + p2c.y*p2c.y;
        if(dist<closest){
            closest = dist;
            best_p = p;
        }
    }
    return best_p;
}
}

