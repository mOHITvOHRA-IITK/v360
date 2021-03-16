#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "webcam_topic_publish");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle n;

    ros::Publisher marker_pub = n.advertise<sensor_msgs::Image>("/webcam_images", 10);
    sensor_msgs::Image image_data;

    cv::VideoCapture capture(0);

    cv::Mat frame;

    while (ros::ok())
    {
        if (capture.isOpened() == false)
        {
            std::cout << "webcam is not available\n";
            break;
        }

        //Create image frames from capture
        capture >> frame;

        if (!frame.empty())
        {
            cv::imshow("frame", frame);
            cv::waitKey(1);

            //convertinh opencv image to sensor_msg::image format
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            image_data = *img_msg;

            marker_pub.publish(image_data);

        }
    }
}
