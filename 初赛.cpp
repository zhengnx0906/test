#include "topics.hpp"
#include<vector>
using namespace cv;
//#define showresult

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string robotName = "maxwell";
    if (argc > 1) {
        robotName = std::string(argv[1]);
    }
    auto playerNode = std::make_shared<rclcpp::Node>(robotName + "_player");
    common::msg::BodyTask btask;
    common::msg::HeadTask htask;
    common::msg::GameData gameData;
    btask.type = btask.TASK_WALK;
    btask.count = 2;
    btask.step = 0.03;
    htask.yaw = 0.0;
    htask.pitch = 45.0;
    auto bodyTaskNode = std::make_shared<BodyTaskPublisher>(robotName);
    auto headTaskNode = std::make_shared<HeadTaskPublisher>(robotName);
    auto imageSubscriber = std::make_shared<ImageSubscriber>(robotName);
    auto imuSubscriber = std::make_shared<ImuDataSubscriber>(robotName);
    auto headSubscriber = std::make_shared<HeadAngleSubscriber>(robotName);
    auto resImgPublisher = std::make_shared<ResultImagePublisher>(robotName);
    auto gameSubscriber = std::make_shared<GameDataSubscriber>(robotName);
    rclcpp::WallRate loop_rate(10.0);

    while (rclcpp::ok()) {
        rclcpp::spin_some(bodyTaskNode);
        rclcpp::spin_some(headTaskNode);
        rclcpp::spin_some(imageSubscriber);
        rclcpp::spin_some(imuSubscriber);
        rclcpp::spin_some(headSubscriber);
        rclcpp::spin_some(resImgPublisher);
        auto imuData = imuSubscriber->GetData();
        auto src = imageSubscriber->GetImage().clone();
        auto headAngle = headSubscriber->GetData();
        Point center;
        size_t size;
       // RCLCPP_INFO(playerNode->get_logger(),"yaws: %f",imuData.yaw);
        if (!src.empty()) 
        {
         // 在这里写图像处理
            Mat cimg;
            medianBlur(src, src, 5);
            cvtColor(src,cimg,COLOR_BGR2GRAY);
            threshold(cimg, cimg, 230, 255, THRESH_BINARY);
            std::vector<Vec3f> circles;
            HoughCircles(cimg, circles, HOUGH_GRADIENT, 1, 600,100, 15, 20, 60  );
            //RCLCPP_INFO(playerNode->get_logger(),"R: %u",circles.size());
        for (size_t i = 0; i < circles.size(); i++)
        {
            if(i==0)
            {
                center.x=cvRound(circles[i][0]);
                center.y=cvRound(circles[i][1]);
            }
            Point center1(cvRound(circles[i][0]), cvRound(circles[i][1]));
            #ifdef showresult
            int radius = cvRound(circles[i][2]);
            //绘制圆心  
            circle(src, center1, 3, Scalar(0, 255, 0), -1, 8, 0);
            //绘制圆轮廓  
            circle(src, center1, radius, Scalar(155, 50, 255), 3, 8, 0);
            #endif
        }
            size=circles.size();
            //Mat test;
            //test=convertTo3Channels(Bina);
            resImgPublisher->Publish(src); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看
        }
        // write your code here
        if (size>0)
        {

            if(center.x<=310)
            {
                btask.step=0.01;
                btask.lateral=0.02;//left
            }
            if(center.x>=330)
            {
                btask.step=0.01;
                btask.lateral=-0.02;//right
            }
            if (310<center.x && center.x<330)
            {
                btask.step=0.07;
                btask.lateral=0;//right
            }
        }
        if(-177<imuData.yaw&&imuData.yaw<0)
        {
           btask.turn=-180-imuData.yaw;
        }
        if(0<imuData.yaw&&imuData.yaw<177)
        {
            btask.turn=180-imuData.yaw;
        }
        if(imuData.yaw>=177||imuData.yaw<=-177)
        {
            btask.turn=0;
        }
        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);
        loop_rate.sleep();
    }
    return 0;
}