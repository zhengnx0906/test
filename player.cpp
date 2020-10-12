#include "topics.hpp"
using namespace cv;
using namespace std;
#include <opencv2/imgproc.hpp>
/**/
Mat convertTo3Channels(const Mat& binImg)
{
    Mat three_channel = Mat::zeros(binImg.rows,binImg.cols,CV_8UC3);
    vector<Mat> channels;
    for (int i=0;i<3;i++)
    {
        channels.push_back(binImg);
    }
    merge(channels,three_channel);
    return three_channel;
}

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
            Mat gray,Bina;
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            cvtColor( src, gray, COLOR_BGR2GRAY );//huidu
            blur( gray, gray, Size(3,3) );
            threshold(gray, Bina, 230, 255, THRESH_BINARY);
            findContours(Bina, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

            /// 绘出轮廓
            Mat drawing = Mat::zeros(src.size(), CV_8UC3);
            for (int i = 0; i < contours.size(); i++)
            {
                Scalar color = Scalar(255,255,0);
                drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, Point());
            }
            //Mat test;
            //test=convertTo3Channels(Bina);
            resImgPublisher->Publish(drawing); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看
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


