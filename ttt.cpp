#include "topics.hpp"
#include <common/srv/get_color.hpp>
#include <opencv2/imgproc.hpp>
using namespace cv;
using namespace std;

enum RobotColor {
    COLOR_INVALID,
    COLOR_RED,
    COLOR_BLUE
};
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
RobotColor GetRobotColor(const std::string &name)
{
    if (name.find("red") != std::string::npos) {
        return COLOR_RED;
    } else if (name.find("blue") != std::string::npos) {
        return COLOR_BLUE;
    } else {
        return COLOR_INVALID;
    }
}
Mat hsv_image;//rgb->hsv more accurate
Point FIgureColor(RobotColor color,Mat image)//image==main:src
{
    
    Point robo;
    //int row=image.rows;
    int count=0;
    int x=0,y=0;//que ding heng zuo biao
    //int col=image.cols;
    int H=0,S=0,V=0;//be used for figure
    cvtColor(image,hsv_image,COLOR_BGR2HSV);
    for(int i=0;i<480;i++)
    {
        for(int j=0;j<640;j++)
        {
            H=hsv_image.at<Vec3b>(i,j)[0];//get H,S,V
            S=hsv_image.at<Vec3b>(i,j)[1];
            V=hsv_image.at<Vec3b>(i,j)[2];
            if(color==COLOR_BLUE)
            {
                //if( (((H>0)&&(H<20))&& ((S>43)&&(S<255)) && ((V>46)&&(V<255))) || (((H>156)&&(H<180))&&((S>43)&&(S<255))&&((V>46)&&(V<255))))
                if(((H>=0)&&(H<=5))&&((S>43)&&(S<255))&&((V>46)&&(V<255)))
                {
                    hsv_image.at<Vec3b>(i,j)[0]=0;
                    hsv_image.at<Vec3b>(i,j)[1]=1;
                    hsv_image.at<Vec3b>(i,j)[2]=0;
                    x=j+x;
                    count++;
                    if(x==0)
                    {
                        y=i+10;
                    }
                }
                else
                {
                    hsv_image.at<Vec3b>(i,j)[0]=180;
                    hsv_image.at<Vec3b>(i,j)[1]=255;
                    hsv_image.at<Vec3b>(i,j)[2]=255;
                }
                
            }
            if(color==COLOR_RED)
            {
                if((H>90)&&(H<130) && ((S>43)&&(S<255)) &&((V>46)&&(V<255)))
                {
                    hsv_image.at<Vec3b>(i,j)[0]=0;
                    hsv_image.at<Vec3b>(i,j)[1]=1;
                    hsv_image.at<Vec3b>(i,j)[2]=0;
                    x=j+x;
                    count++;
                    if(x==0)
                    {
                        y=i+10;
                    }
                }
                else
                {
                    hsv_image.at<Vec3b>(i,j)[0]=180;
                    hsv_image.at<Vec3b>(i,j)[1]=255;
                    hsv_image.at<Vec3b>(i,j)[2]=255;
                }
                
            }
        }
    }
    if(count==0){count=1;}
    robo.x=x/count;
    robo.y=y;
    return robo;
}
Point FIgureGray(Mat image,RobotColor color)//image==main:src
{
    
    Point gate;
    //int row=image.rows;
    int count=0;
    int x=0,y=0;//que ding heng zuo biao
    //int col=image.cols;
    int H=0,S=0,V=0;//be used for figure
    cvtColor(image,hsv_image,COLOR_BGR2HSV);
    for(int i=0;i<480;i++)
    {
        for(int j=0;j<640;j++)
        {
            H=hsv_image.at<Vec3b>(i,j)[0];//get H,S,V
            S=hsv_image.at<Vec3b>(i,j)[1];
            V=hsv_image.at<Vec3b>(i,j)[2];
             if(color==COLOR_BLUE)
            {
            if((H>0)&&(H<180) && ((S>0)&&(S<43)) &&((V>182)&&(V<=190)))
            {
                hsv_image.at<Vec3b>(i,j)[0]=120;
                hsv_image.at<Vec3b>(i,j)[1]=255;
                hsv_image.at<Vec3b>(i,j)[2]=255;
                x=j+x;
                count++;
                if(x==0)
                {
                    y=i+10;
                }
            }
            }
            if(color==COLOR_RED)
             {
            if((H>0)&&(H<60) && ((S>0)&&(S<43)) &&((V>175)&&(V<177)))
            {
                hsv_image.at<Vec3b>(i,j)[0]=120;
                hsv_image.at<Vec3b>(i,j)[1]=255;
                hsv_image.at<Vec3b>(i,j)[2]=255;
                x=j+x;
                count++;
                if(x==0)
                {
                    y=i+10;
                }
            }
            }
        }
    }
    if(count==0){count=1;}
    x=x+10;
    gate.x=x/count;
    gate.y=y;
    return gate;
}
int GetRobotId(const std::string &name)
{
    return static_cast<int>(name.back() - '0');
}

static inline bool ContoursSortFun(vector<cv::Point> contour1,vector<cv::Point> contour2)  
{  
    return (cv::contourArea(contour1) > cv::contourArea(contour2));  
} 

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string robotName = "maxwell";

    std::string tmp = std::string(argv[1]);
    std::string team = tmp.substr(0, tmp.find_last_of('_'));

    auto playerNode = std::make_shared<rclcpp::Node>(tmp + "_player");
    rclcpp::Client<common::srv::GetColor>::SharedPtr client =
        playerNode->create_client<common::srv::GetColor>("gamectrl/get_color");
    while (!client->wait_for_service(std::chrono::duration<long long>(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(playerNode->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(playerNode->get_logger(), "service not available, waiting again...");
    }
    auto request = std::make_shared<common::srv::GetColor::Request>();
    request.get()->team = team;
    auto result = client->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(playerNode, result);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        if (result.get()->color == "invalid") {
            RCLCPP_ERROR(playerNode->get_logger(), "Not supportted team name. Exiting.");
            return 0;
        }
        robotName = result.get()->color + tmp.substr(tmp.find_last_of('_'));
        RCLCPP_INFO(playerNode->get_logger(), "robotName: %s", robotName.c_str());
    } else {
        RCLCPP_ERROR(playerNode->get_logger(), "Exiting.");
        return 0;
    }

    RobotColor myColor = GetRobotColor(robotName);
    int myId = GetRobotId(robotName);
    common::msg::BodyTask btask;
    common::msg::HeadTask htask;
    common::msg::GameData gameData;
    btask.type = btask.TASK_WALK;
    btask.count = 2;
    btask.step = 0.03;
    htask.yaw = 0.0;
    htask.pitch = 10.0;
    auto bodyTaskNode = std::make_shared<BodyTaskPublisher>(robotName);
    auto headTaskNode = std::make_shared<HeadTaskPublisher>(robotName);
    auto imageSubscriber = std::make_shared<ImageSubscriber>(robotName);
    auto imuSubscriber = std::make_shared<ImuDataSubscriber>(robotName);
    auto headSubscriber = std::make_shared<HeadAngleSubscriber>(robotName);
    auto resImgPublisher = std::make_shared<ResultImagePublisher>(robotName);
    auto gameSubscriber = std::make_shared<GameDataSubscriber>(robotName);
    rclcpp::WallRate loop_rate(10.0);
    float initYaw = 0.0;
    Point robot;//dui fang robot dian
    Point Gate; 
    int counts=0;
    while (rclcpp::ok()) {
        rclcpp::spin_some(bodyTaskNode);
        rclcpp::spin_some(headTaskNode);
        rclcpp::spin_some(imageSubscriber);
        rclcpp::spin_some(imuSubscriber);
        rclcpp::spin_some(headSubscriber);
        rclcpp::spin_some(resImgPublisher);
        rclcpp::spin_some(gameSubscriber);
        gameData = gameSubscriber->GetData();
        auto imuData = imuSubscriber->GetData();
        auto src = imageSubscriber->GetImage().clone();
        auto headAngle = headSubscriber->GetData();
        Point ball_center;//yuan xin
        size_t size=0;  
        // ----------------- 可以修改的部分 begin--------------------
        // 郑重声明：以下说明仅供参考，实际情况以实际为准
        // 下面提供的是一些可能会用到的相关信息接口说明和示例，可以根据自己的需要去使用，不必要纠结于现有的代码结构
        if (gameData.state == gameData.STATE_INIT){
            // 每次开球时都会进入这里
            // 例如：RCLCPP_INFO(playerNode->get_logger(), "%d", gameData.blue_score);
            // INII状态时，1号机器人位于己方这边的中圈上，2号机器人位于己方的禁区线上
            initYaw = imuData.yaw; // 获取init状态时，机器人的朝向，此时的方向即为自己进攻的方向
        }

        // 将机器人的当前朝向减去init时的朝向，即可得到机器人相对于初始位置的方向，
        // 这样可以保证自己不管是红色或者蓝色，其yaw都是在面朝对方球门时为0
        // 下面提供这种转换的方法，需要该转换的可以把注释符号去掉
        // imuData.yaw = imuData.yaw - initYaw; 

        // IMU数据里的yaw信息是统一的，属于绝对方向，但是红色机器人和蓝色机器人的进球方向是不一样的，
        // 假如红色的进球方向是0度，则蓝色的进球方向就是180度
        // 比赛时使用什么颜色的机器人是现场决定的，所以对于两种颜色的机器人，都需要考虑如何
        // 如果使用了上面的转换方法，则不需要再关心不同颜色机器人的方向问题
        if (myColor == COLOR_RED) {
            // 使用的红色的机器人
        } else if (myColor == COLOR_BLUE) {
            // 使用的蓝色的机器人
        }

        // 复赛每个组使用两台机器人，需要根据不同的机器人id制定策略的队伍，可以利用myId进行区分
        // 机器人出界后，会将出界的机器人放回己方半场靠近中线的约0.5米处，此时该机器人会有30s的惩罚
        // 由于有两台机器人，每一天机器人重生的位置都是在己方固定的某一边，并不是从哪边出去就从哪边重生，
        // 具体哪一台从哪边重生，请自行运行仿真查看
        if (myId == 1) {
            // 1号机器人
 
        } else if (myId == 2) {
            // 2号机器人
        }

        if (!src.empty()) {
            // 在这里写图像处理
            Mat gray,Bina;
            vector<vector<Point> > contours,c_robot;

            vector<Vec4i> hierarchy,h_robot;
            cvtColor( src, gray, COLOR_BGR2GRAY );//huidu
            blur( gray, gray, Size(3,3) );
            threshold(gray, Bina, 0, 20, THRESH_BINARY_INV);
            findContours(Bina, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
            std::sort(contours.begin(),contours.end(),ContoursSortFun);//按面积把轮廓大小排序
            size=contours.size();
            if(size>6){size=6;}
            Mat drawing = Mat::zeros(src.size(), CV_8UC3);
            vector<Moments> mom(contours.size());
	        vector<Point2f> m(contours.size());
            //hua dui fang robot 
            if(myColor==COLOR_RED)
            {
                robot=FIgureColor(COLOR_BLUE,src);//get robot point
                cvtColor(hsv_image,hsv_image,COLOR_HSV2BGR);
                //Gate=FIgureGray(src,COLOR_BLUE);
                //RCLCPP_INFO(playerNode->get_logger(),"row::::%d",robot.x);

            }
            if(myColor==COLOR_BLUE)
            {
                robot=FIgureColor(COLOR_RED,src);
                cvtColor(hsv_image,hsv_image,COLOR_HSV2BGR);
               //RCLCPP_INFO(playerNode->get_logger(),"row::::%f",robot.x);
               //Gate=FIgureGray(src,COLOR_RED);
            } 
            /*---------------------------------------------------------*/
            cvtColor( hsv_image, hsv_image, COLOR_BGR2GRAY );//huidu
            blur( hsv_image, hsv_image, Size(3,3) );
            threshold(hsv_image, hsv_image, 0, 100, THRESH_BINARY_INV);
            findContours(hsv_image, c_robot, h_robot, RETR_EXTERNAL, CHAIN_APPROX_NONE); //查找轮廓
            vector<Rect> boundRect(c_robot.size()); //定义外接矩形集合
            int x0=0, y0=0, w0=0, h0=0;
            Mat d_robot = Mat::zeros(src.size(), CV_8UC3);
            for(int i=0; i<c_robot.size(); i++)
            {
                boundRect[i] = boundingRect((Mat)c_robot[i]); //查找每个轮廓的外接矩形
                drawContours(d_robot,c_robot, i, Scalar(0, 0, 255), 2, 8);  //绘制轮廓
                x0 = boundRect[i].x;  //获得第i个外接矩形的左上角的x坐标
                y0 = boundRect[i].y; //获得第i个外接矩形的左上角的y坐标
                w0 = boundRect[i].width; //获得第i个外接矩形的宽度
                h0 = boundRect[i].height; //获得第i个外接矩形的高度
                rectangle(d_robot, Point(x0, y0), Point(x0+w0, y0+h0), Scalar(0, 255, 0), 2, 8); //绘制第i个外接矩形
            }

            //RCLCPP_INFO(playerNode->get_logger(),"num of robot::%d",c_robot.size());

            /*---------------------------------------------------------*/
            float x=0;
            float y=0;
            for (int i = 0; i < contours.size(); i++)
            {
                mom[i] = moments(contours[i], false);
		        m[i] = Point(static_cast<float>(mom[i].m10 / mom[i].m00), static_cast<float>(mom[i].m01 / mom[i].m00));
                Scalar color = Scalar(255,255,0);
                drawContours(drawing, contours, i, color, 1, 8, hierarchy, 0, Point());
                circle(drawing, m[i], 3, (0, 255, 255), -1);
            }
            if (size >0)
            {
                for (int i = 0; i < size; i++)
                {
                    x=m[i].x+x;
                    y=m[i].y+y;
                }
            x=float(x/size);
            y=float(y/size);
            ball_center.x=x;
            ball_center.y=y;
            RCLCPP_INFO(playerNode->get_logger(),"ball::%f",x);
            char label[1024]={0};
            int font_face = FONT_HERSHEY_COMPLEX; 
	        double font_scale = 0.5;
	        int thickness = 2;
            circle(drawing, ball_center, 10, (255, 0, 255), -1);
            sprintf(label, "(%f, %f)", x, y);
            putText(drawing, label, Point2i(int(x-30),int(y-30)), font_face, font_scale, Scalar(0, 255, 255), thickness, 8, 0);
            }
            //Mat test;
            //test=convertTo3Channels(hsv_image);
            resImgPublisher->Publish(drawing); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看

        }
        // write your code here
        // 郑重声明：以上说明仅供参考，实际情况以实际为准
        // ----------------- 可以修改的部分 end--------------------
        if (robotName.back() == '1')
        {
        // if (size>=0)
        // {
        //     if(center.x<=378)
        //     {
        //         counts++;
        //         btask.step=0.01;
        //         btask.lateral=0.01;//left
        //         if(counts>200)
        //         {
        //             htask.pitch=10;
                    
        //             for(int i=0;i<10;i++)
        //             {
        //             Gate=FIgureWhite(src);
        //             if(Gate.x<=310)
        //             {
        //                 btask.step=0.0;
        //                 btask.turn=1;
        //                 btask.lateral=-0.01;
        //             }
        //             if(Gate.x>=330)
        //             {
        //                 btask.step=0.0;
        //                 btask.turn=-1;
        //                 btask.lateral=0.01;
        //             }
        //             if (310<Gate.x && Gate.x<330)
        //             {
        //                 btask.step=0.0;
        //                 btask.lateral=0.0;
        //             }
        //             }
        //             counts=0;
        //             htask.pitch=60;
        //         }
        //     }
        //     if(center.x>=385)
        //     {
        //         counts++;
        //         btask.step=0.01;
        //         btask.lateral=-0.01;//right
        //         if(counts>200)
        //         {
        //             htask.pitch=10;
                    
        //             for(int i=0;i<10;i++)
        //             {
        //             Gate=FIgureGray(src);
        //             if(Gate.x<=310)
        //             {
        //                 btask.step=0.0;
        //                 btask.turn=1;
        //                 btask.lateral=-0.01;
        //             }
        //             if(Gate.x>=330)
        //             {
        //                 btask.step=0.0;
        //                 btask.turn=-1;
        //                 btask.lateral=0.01;
        //             }
        //             if (310<Gate.x && Gate.x<330)
        //             {
        //                 btask.step=0.0;
        //                 btask.lateral=0.0;
        //             }
        //             }
        //             counts=0;
        //             htask.pitch=60;
        //         }
        //     }
        //     if (378<center.x && center.x<385)
        //     {
        //         counts++;
        //         btask.step=0.03;
        //         btask.lateral=0;//right
        //         if(counts>200)
        //         {
        //             htask.pitch=-10;
        //             for(int i=0;i<10;i++)
        //             {
        //             Gate=FIgureGray(src);
        //             if(Gate.x<=310)
        //             {
        //                 btask.step=0.0;
        //                 btask.turn=1;
        //                 btask.lateral=-0.01;
        //             }
        //             if(Gate.x>=330)
        //             {
        //                 btask.step=0.0;
        //                 btask.turn=-1;
        //                 btask.lateral=0.01;
        //             }
        //             if (310<Gate.x && Gate.x<330)
        //             {
        //                 btask.step=0.0;
        //                 btask.lateral=0.0;
        //             }
        //             }
        //             counts=0;
        //             htask.pitch=60;
        //         }
        //     }
        //     //RCLCPP_INFO(playerNode->get_logger(),"count::::%d",Gate.x);
        //     //RCLCPP_INFO(playerNode->get_logger(),"count::::%d",counts);
        // }
         }
         /*if(-177<imuData.yaw&&imuData.yaw<0)
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
        }*/
         if (robotName.back() == '2')
         {
             btask.turn=-3;
             btask.count=2;
             btask.lateral=0.05;
         }





        // 下面的代码不能动
        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);
        loop_rate.sleep();
    }
    return 0;
}

