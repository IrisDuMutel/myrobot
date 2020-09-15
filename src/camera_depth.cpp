// #include <ros/ros.h>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/core/mat.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <string>
// #include <iostream>

// const std::string wn = "OCV_window";
/////converts images to CV
// class ImageConverter
// {
//     ros::NodeHandle nh_;
//     image_transport::ImageTransport it_;
//     image_transport::Subscriber image_sub_;
//     image_transport::Publisher image_pub_;

//     public:
//     ImageConverter()
//         : it_(nh_)
//     {
//         image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
//         image_pub_ = it_.advertise("/image_editor/output_image", 1);

//         cv::namedWindow(wn);
//     }

//     ~ImageConverter()
//     {
//         cv::destroyWindow(wn);
//     }

//     void imageCb(const sensor_msgs::ImageConstPtr& incoming_message)
//     {
//         cv_bridge::CvImagePtr cvi;
//         try
//         {
//             cvi = cv_bridge::toCvCopy(incoming_message, sensor_msgs::image_encodings::RGB8);
//         }
//         catch (cv_bridge::Exception& e)
//         {
//             ROS_ERROR("CV_Bridge Exception: %s", e.what());
//             return;
//         }

//         cv::imshow(wn, cvi->image);
//         cv::waitKey(3);

//         image_pub_.publish(cvi->toImageMsg());

//     }

// };

// int main(int argc, char** argv)
// {
//     ros::init(argc,argv, "Image_Converter");
//     ImageConverter ic;
//     ros::spin();
//     return(0);
// }


////NOT WORKING PROPERLY
// #include <iostream>

// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/image_encodings.h>
// #include <cv_bridge/cv_bridge.h>

// #include <opencv2/highgui/highgui.hpp>

// static const uint32_t MY_ROS_QUEUE_SIZE = 1000;

// void imgcb(const sensor_msgs::Image::ConstPtr& msg)
// {
//     // The message's data is a raw buffer. While the type is uint8_t*, the
//     // data itself is an array of floats (for depth data), the value of
//     // the float being the distance in meters.
//     std::cout << "Top-left corner: " << *reinterpret_cast<const float*>(&msg->data[0]) << "m" << std::endl;

//     try {
//         cv_bridge::CvImageConstPtr cv_ptr;
//         cv_ptr = cv_bridge::toCvShare(msg);

//         // imshow expects a float value to lie in [0,1], so we need to normalize
//         // for visualization purposes.
//         double max = 0.0;
//         cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
//         cv::Mat normalized;
//         cv_ptr->image.convertTo(normalized, CV_32F, 1.0/max, 0)  ;

//         cv::imshow("foo", normalized);
//         cv::waitKey(1);
//     } catch (const cv_bridge::Exception& e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//     }
// }

// int main(int argc, char* argv[])
// {
//     ros::init(argc, argv, "foo");

//     std::cout << "Oh hai there!" << std::endl;

//     ros::NodeHandle nh;
//     ros::Subscriber sub = nh.subscribe("camera/depth/image", MY_ROS_QUEUE_SIZE, imgcb);

//     cv::namedWindow("foo");
//     ros::spin();
//     cv::destroyWindow("foo");

//     std::cout << "byebye my friend" << std::endl;

//     return 0;
// } 

////////////////////////////////////////////////////////////////////////////////////

#include <image_transport/image_transport.h>

typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image)
{
    // If position is invalid
    if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width))
        return -1;
    int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depth_image->step/depth_image->width) == 4) {
        U_FloatConvert depth_data;
        int i, endian_check = 1;
        // If big endian
        if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) ||  // Both big endian
           ((!depth_image->is_bigendian) && (*(char*)&endian_check == 1))) { // Both lil endian
            for (i = 0; i < 4; i++)
                depth_data.byte_data[i] = depth_image->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depth_data.float_data == depth_data.float_data)
                return int(depth_data.float_data*1000);
            return -1;  // If depth data invalid
        }
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++) 
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return int(depth_data.float_data*1000);
        return -1;  // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
   int temp_val;
   // If big endian
   if (depth_image->is_bigendian)
       temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
   // If little endian
   else
       temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);
   // Make sure data is valid (check if NaN)
   if (temp_val == temp_val)
       return temp_val;
   return -1;  // If depth data invalid
}

// Image Callback
void imageCallback(const sensor_msgs::ImageConstPtr& image) {
    int depth = ReadDepthData(240, 320, image); // Width = 640, Height = 480
    ROS_INFO("Depth: %d", depth);
} 

//MAIN
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imagegrab");
    ros::NodeHandle n;
    printf("READY to get image\n");
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, imageCallback);
    ros::spin();
    return 0;
} 