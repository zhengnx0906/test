#include "topics.hpp"
using namespace cv;
using namespace std;
#include <opencv2/imgproc.hpp>
//#define show

/**/
int piict_x=320;
int pict_y=240;
float last_l=0;

static inline bool ContoursSortFun(vector<cv::Point> contour1,vector<cv::Point> contour2)  
{  
    return (cv::contourArea(contour1) > cv::contourArea(contour2));  
} 

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
    btask.step = 0.01;
    
    htask.yaw = 0.0;//
    htask.pitch = 45.0;//fu yan
    auto bodyTaskNode = std::make_shared<BodyTaskPublisher>(robotName);
    auto headTaskNode = std::make_shared<HeadTaskPublisher>(robotName);
    auto imageSubscriber = std::make_shared<ImageSubscriber>(robotName);
    auto imuSubscriber = std::make_shared<ImuDataSubscriber>(robotName);
    auto headSubscriber = std::make_shared<HeadAngleSubscriber>(robotName);
    auto resImgPublisher = std::make_shared<ResultImagePublisher>(robotName);
    rclcpp::WallRate loop_rate(10.0);
    while (rclcpp::ok()) 
    {
        rclcpp::spin_some(bodyTaskNode);
        rclcpp::spin_some(headTaskNode);
        rclcpp::spin_some(imageSubscriber);
        rclcpp::spin_some(imuSubscriber);
        rclcpp::spin_some(headSubscriber);
        rclcpp::spin_some(resImgPublisher);
        auto imuData = imuSubscriber->GetData();
        auto src = imageSubscriber->GetImage().clone();
        auto headAngle = headSubscriber->GetData();
        btask.count = 1;
        Point2f center;
        int size;
        //RCLCPP_INFO(playerNode->get_logger(),"rows: %d,cols: %d",src.rows,src.cols);
        if (!src.empty()) 
        {
            Mat gray,Bina;
            vector<vector<Point> > contours;

            vector<Vec4i> hierarchy;
            cvtColor( src, gray, COLOR_BGR2GRAY );//huidu
            blur( gray, gray, Size(3,3) );
            threshold(gray, Bina, 240, 255, THRESH_BINARY);
            findContours(Bina, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
            std::sort(contours.begin(),contours.end(),ContoursSortFun);//按面积把轮廓大小排序
            size=contours.size();
            if(size>6){size=6;}//我的想法是找一块一块的轮廓，如果大于3就让他只找三个最大轮廓，实际未操作
            /// 绘出轮廓
            Mat drawing = Mat::zeros(src.size(), CV_8UC3);
            vector<Moments> mom(contours.size());
	        vector<Point2f> m(contours.size());
            float x=0;
            float y=0;
            for (int i = 0; i < contours.size(); i++)
            {
                mom[i] = moments(contours[i], false);
		        m[i] = Point(static_cast<float>(mom[i].m10 / mom[i].m00), static_cast<float>(mom[i].m01 / mom[i].m00));
		        //中心点坐标
                #ifdef show
                Scalar color = Scalar(255,255,0);
                drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, Point());
                circle(drawing, m[i], 3, (0, 255, 255), -1);
                #endif
            }
            if (size >0)
            {
                for (int i = 0; i < size; i++)
                {
                    x=m[i].x+x;
                    y=m[i].y+y;
                }
            x=x/size;
            y=y/size;
            center.x=x;
            center.y=y;
            #ifdef show
            char label[1024]={0};
            int font_face = FONT_HERSHEY_COMPLEX; 
	        double font_scale = 0.5;
	        int thickness = 2;
            circle(drawing, center, 10, (255, 0, 255), -1);
            sprintf(label, "(%f, %f)", x, y);
            putText(drawing, label, Point2i(int(x-30),int(y-30)), font_face, font_scale, Scalar(0, 255, 255), thickness, 8, 0);
            #endif
            }

            //Mat test;
            //test=convertTo3Channels(Bina);
            resImgPublisher->Publish(drawing); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看
        }
        // write your code here
        if (size>0)
        {

            if(center.x<=310)
            {
                btask.step=0.04;
                btask.lateral=0.05;//left
            }
            if(center.x>=330)
            {
                btask.step=0.04;
                btask.lateral=-0.05;//right
            }
            if (310<center.x && center.x<330)
            {
                btask.step=0.05;
                btask.lateral=0;//right
            }
        }
        if(size<=0)
        {
            btask.step=-0.05;
            btask.lateral=last_l;
            btask.count = 2;
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
        last_l=btask.lateral;
        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);
        loop_rate.sleep();
    }
    return 0;
}
