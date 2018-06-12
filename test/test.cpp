#include "weitu.h"

void camera_marker_test(){
    Timer timer;
    weitu::Camera camera;
    camera.open(0);
    timer.out("open camera");
    int FPS = 0;
    while (1) {
       cv::Mat img = camera.get();
       if(!img.empty()){
           cv::pyrDown(img, img);
           std::vector<FinderPattern> pattern = qr_pattern::find(img);
           cv::cvtColor(img, img, CV_GRAY2BGR);

           for(int i=0;i<pattern.size();i++){
               int centerRow=int(pattern[i].getY());
               int centerCol=int(pattern[i].getX());
               cv::line(img, cv::Point(centerCol, centerRow-40), cv::Point(centerCol, centerRow+40)
                        , cv::Scalar(0, 0, 255), 2);
               cv::line(img, cv::Point(centerCol-40, centerRow), cv::Point(centerCol+40, centerRow)
                        , cv::Scalar(0, 0, 255), 2);
           }

           cv::imshow("img", img);
           cv::waitKey(1);
           FPS ++;
       }
       if(timer.elapsed()>1){
           std::cout << "FPS: " << FPS << std::endl;
           FPS = 0;
           timer.reset();
       }
    }
}

void hole_test(){
    using namespace std;
    using namespace cv;

    auto rgb = cv::imread("/home/s/catkin_ws/src/weitu/test/test2.png");

    Timer timer;

//    cv::Rect roi(rgb.rows/4, rgb.cols/4, rgb.rows/2, rgb.cols/2);
//    rgb = rgb(roi);

    auto p = hole_detect::find(rgb);

    timer.out("detect time");

    if(rgb.channels()==1){
        cv::cvtColor(rgb, rgb, CV_GRAY2BGR);
    }
    if(p.x>0){
        cv::line(rgb, cv::Point(p.x, p.y-40), cv::Point(p.x, p.y+40)
                 , cv::Scalar(0, 0, 255), 2);
        cv::line(rgb, cv::Point(p.x-40, p.y), cv::Point(p.x+40, p.y)
                 , cv::Scalar(0, 0, 255), 2);
    }else {
        std::cout << "nothing found" << std::endl;
    }
    imshow("rgb", rgb);
    waitKey(0);
}

void camera_hole_test(){
    Timer timer;
    weitu::Camera camera;
    camera.open(0);
    timer.out("open camera");
    while (1) {
       cv::Mat rgb = camera.get();
       if(!rgb.empty()){

           cv::pyrDown(rgb, rgb);

           timer.reset();
           auto p = hole_detect::find(rgb);
           timer.out("detect time");

           if(rgb.channels()==1){
               cv::cvtColor(rgb, rgb, CV_GRAY2BGR);
           }
           if(p.x>0){
               cv::line(rgb, cv::Point(p.x, p.y-40), cv::Point(p.x, p.y+40)
                        , cv::Scalar(0, 0, 255), 2);
               cv::line(rgb, cv::Point(p.x-40, p.y), cv::Point(p.x+40, p.y)
                        , cv::Scalar(0, 0, 255), 2);
           }

           imshow("rgb", rgb);
           cv::waitKey(100);
       }
    }
}


int main()
{	
    // camera_marker_test();
    camera_hole_test();
//    hole_test();

    return 0;
}
