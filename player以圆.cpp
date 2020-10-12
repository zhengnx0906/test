#include "topics.hpp"
#include<vector>
#include <opencv4/opencv2/core/core.hpp>
#include<opencv4/opencv2/highgui/highgui.hpp>
using namespace cv;
/*isin dectect if the ball is in sight ,in sight return 1 else return 0
then if !isin 
*/
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
        //RCLCPP_INFO(playerNode->get_logger(),"Hey! I'm %s",argv[1]);
        if (!src.empty()) {
            // 在这里写图像处理
            Mat cimg;
            medianBlur(src, src, 5);
            cvtColor(src,cimg,COLOR_BGR2GRAY);
            GaussianBlur(cimg, cimg, Size(9, 9), 2, 2);
            //   medianBlur(cimg, cimg, 5);
            Canny(cimg,cimg,10,250,5);
            std::vector<Vec3f> circles;
            HoughCircles(cimg, circles, HOUGH_GRADIENT, 1, 30,100, 28, 10, 100  );
        for (size_t i = 0; i < circles.size(); i++)
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            //绘制圆心  
            circle(src, center, 3, Scalar(0, 255, 0), -1, 8, 0);
            //绘制圆轮廓  
            circle(src, center, radius, Scalar(155, 50, 255), 3, 8, 0);
        }
            resImgPublisher->Publish(src); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看
        }
        // write your code here
        if (robotName.back() == '1')
        {
            btask.step = -1; // 1 号机器人后退
        }

        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);
        loop_rate.sleep();
    }
    return 0;
}
