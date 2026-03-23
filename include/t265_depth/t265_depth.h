#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

namespace t265_depth
{

    // stereo parameters
    constexpr int kPreFilterCap = 11;
    constexpr int kSADWindowSize = 11;
    constexpr int kMinDisparity = 0;
    constexpr int kNumDisparities = 64;
    constexpr int kUniquenessRatio = 0;
    constexpr int kSpeckleRange = 3;
    constexpr int kSpeckleWindowSize = 500;
    // bm parameters
    constexpr int kTextureThreshold = 0;
    const std::string kPreFilterType = "xsobel";
    constexpr int kPreFilterSize = 3;
    // sgbm parameters
    constexpr bool kUseSGBM = false;
    constexpr int kP1 = 120;
    constexpr int kP2 = 240;
    constexpr int kDisp12MaxDiff = -1;
    constexpr int kSGBMMode = 0;

    constexpr bool kDoMedianBlur = true;

    class t265Depth : public rclcpp::Node
    {
    public:
        explicit t265Depth(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~t265Depth();

        void initializeRectificationMapping(std::string param_file_path);
        void publishCameraInfo(cv::Mat K1, cv::Mat K2,
                               cv::Mat P1, cv::Mat P2,
                               cv::Mat R1, cv::Mat R2);
        void elaborateImages(const std_msgs::msg::Header &header_msg);
        void computePointcloud(const cv::Mat &input_disparity,
                               sensor_msgs::msg::PointCloud2 &pointcloud);

    private:
        void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg_left,
                          const sensor_msgs::msg::Image::ConstSharedPtr &image_msg_right);

        image_transport::Publisher pub_img_left_rect_;
        image_transport::Publisher pub_img_right_rect_;
        image_transport::Publisher pub_disparity_;

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_left_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_camera_info_right_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;

        std::string input_topic_left_;
        std::string input_topic_right_;
        std::string output_frame_id_;
        std::string input_transport_;

        sensor_msgs::msg::CameraInfo output_camera_info_left_;
        sensor_msgs::msg::CameraInfo output_camera_info_right_;

        cv::Mat image_left_;
        cv::Mat undist_image_left_;
        cv::Mat image_right_;
        cv::Mat undist_image_right_;

        cv::Mat1f lmapx_;
        cv::Mat1f lmapy_;
        cv::Mat1f rmapx_;
        cv::Mat1f rmapy_;

        std::string param_file_path_;
        int process_every_nth_frame_ = 1;
        int frame_counter_ = 0;
        double scale_ = 1.0;

        // stereo parameters
        int pre_filter_cap_;
        int sad_window_size_;
        int min_disparity_;
        int num_disparities_;
        int uniqueness_ratio_;
        int speckle_range_;
        int speckle_window_size_;

        // bm parameters
        int texture_threshold_;
        int pre_filter_type_;
        int pre_filter_size_;

        // sgbm parameters
        bool use_sgbm_;
        int sgbm_mode_;
        int p1_;
        int p2_;
        int disp_12_max_diff_;

        bool do_median_blur_;

        // output stereo
        float stereo_cx;
        float stereo_cy;
        float focal_length;
        float baseline = 0.064;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                               sensor_msgs::msg::Image>
            MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;

        image_transport::SubscriberFilter image_sub_L_;
        image_transport::SubscriberFilter image_sub_R_;
        std::shared_ptr<Sync> sync_;
    };

} // namespace t265_depth
