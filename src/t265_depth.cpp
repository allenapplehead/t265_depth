#include <t265_depth/t265_depth.h>

namespace t265_depth
{
    t265Depth::t265Depth(const rclcpp::NodeOptions & options)
        : rclcpp::Node("t265_depth", options)
    {
        // Declare parameters with defaults
        declare_parameter("scale", 1.0);
        declare_parameter("process_every_nth_frame", 1);
        declare_parameter("input_topic_left", std::string("/camera/fisheye1/image_raw"));
        declare_parameter("input_topic_right", std::string("/camera/fisheye2/image_raw"));
        declare_parameter("input_transport", std::string("raw"));
        declare_parameter("output_frame_id", std::string("t265_depth"));
        declare_parameter("param_file_path", std::string(""));
        // stereo parameters
        declare_parameter("pre_filter_type", kPreFilterType);
        declare_parameter("pre_filter_cap", kPreFilterCap);
        declare_parameter("sad_window_size", kSADWindowSize);
        declare_parameter("min_disparity", kMinDisparity);
        declare_parameter("num_disparities", kNumDisparities);
        declare_parameter("uniqueness_ratio", kUniquenessRatio);
        declare_parameter("speckle_range", kSpeckleRange);
        declare_parameter("speckle_window_size", kSpeckleWindowSize);
        // bm parameters
        declare_parameter("texture_threshold", kTextureThreshold);
        declare_parameter("pre_filter_size", kPreFilterSize);
        // sgbm parameters
        declare_parameter("use_sgbm", kUseSGBM);
        declare_parameter("sgbm_mode", kSGBMMode);
        declare_parameter("p1", kP1);
        declare_parameter("p2", kP2);
        declare_parameter("disp_12_max_diff", kDisp12MaxDiff);

        declare_parameter("do_median_blur", kDoMedianBlur);

        // Get parameters
        scale_                   = get_parameter("scale").as_double();
        process_every_nth_frame_ = get_parameter("process_every_nth_frame").as_int();
        input_topic_left_        = get_parameter("input_topic_left").as_string();
        input_topic_right_       = get_parameter("input_topic_right").as_string();
        input_transport_         = get_parameter("input_transport").as_string();
        output_frame_id_         = get_parameter("output_frame_id").as_string();
        param_file_path_         = get_parameter("param_file_path").as_string();

        RCLCPP_INFO(get_logger(), "Reading Intrinsics from: %s", param_file_path_.c_str());
        RCLCPP_INFO(get_logger(), "Scale set to: %f", scale_);

        std::string pre_filter_type_string = get_parameter("pre_filter_type").as_string();
        if (pre_filter_type_string == "xsobel")
        {
            pre_filter_type_ = cv::StereoBM::PREFILTER_XSOBEL;
        }
        else if (pre_filter_type_string == "normalized_response")
        {
            pre_filter_type_ = cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE;
        }
        else
        {
            throw std::runtime_error(
                "Unrecognized prefilter type, choices are 'xsobel' or "
                "'normalized_response'");
        }

        pre_filter_cap_     = get_parameter("pre_filter_cap").as_int();
        sad_window_size_    = get_parameter("sad_window_size").as_int();
        min_disparity_      = get_parameter("min_disparity").as_int();
        num_disparities_    = get_parameter("num_disparities").as_int();
        uniqueness_ratio_   = get_parameter("uniqueness_ratio").as_int();
        speckle_range_      = get_parameter("speckle_range").as_int();
        speckle_window_size_= get_parameter("speckle_window_size").as_int();
        texture_threshold_  = get_parameter("texture_threshold").as_int();
        pre_filter_size_    = get_parameter("pre_filter_size").as_int();
        use_sgbm_           = get_parameter("use_sgbm").as_bool();
        sgbm_mode_          = get_parameter("sgbm_mode").as_int();
        p1_                 = get_parameter("p1").as_int();
        p2_                 = get_parameter("p2").as_int();
        disp_12_max_diff_   = get_parameter("disp_12_max_diff").as_int();
        do_median_blur_     = get_parameter("do_median_blur").as_bool();

        initializeRectificationMapping(param_file_path_);

        // Publishers
        pub_img_left_rect_  = image_transport::create_publisher(this, input_topic_left_ + "/rectified");
        pub_img_right_rect_ = image_transport::create_publisher(this, input_topic_right_ + "/rectified");
        pub_camera_info_left_  = create_publisher<sensor_msgs::msg::CameraInfo>(
            input_topic_left_ + "/rectified/info", 10);
        pub_camera_info_right_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            input_topic_right_ + "/rectified/info", 10);
        pub_disparity_  = image_transport::create_publisher(this, "/disparity");
        pub_pointcloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/points2", 10);

        // Synchronized image subscribers
        RCLCPP_INFO(get_logger(), "Image transport: %s", input_transport_.c_str());
        image_sub_L_.subscribe(this, input_topic_left_,  input_transport_, rmw_qos_profile_sensor_data);
        image_sub_R_.subscribe(this, input_topic_right_, input_transport_, rmw_qos_profile_sensor_data);

        sync_ = std::make_shared<Sync>(MySyncPolicy(10), image_sub_L_, image_sub_R_);
        sync_->registerCallback(
            std::bind(&t265Depth::syncCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

    t265Depth::~t265Depth()
    {
    }

    void t265Depth::syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg_left,
                                 const sensor_msgs::msg::Image::ConstSharedPtr &image_msg_right)
    {
        if (++frame_counter_ < process_every_nth_frame_)
        {
            return;
        }
        frame_counter_ = 0;

        image_left_  = cv_bridge::toCvCopy(image_msg_left,  "mono8")->image;
        image_right_ = cv_bridge::toCvCopy(image_msg_right, "mono8")->image;

        std_msgs::msg::Header header_out = image_msg_left->header;

        if (scale_ < 1)
        {
            resize(image_left_,  image_left_,  cv::Size(image_left_.cols  * scale_, image_left_.rows  * scale_));
            resize(image_right_, image_right_, cv::Size(image_right_.cols * scale_, image_right_.rows * scale_));
        }

        elaborateImages(header_out);

        sensor_msgs::msg::Image::SharedPtr out_img_msg_left;
        sensor_msgs::msg::Image::SharedPtr out_img_msg_right;
        try
        {
            out_img_msg_left  = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", undist_image_left_).toImageMsg();
            out_img_msg_right = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", undist_image_right_).toImageMsg();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Could not convert from '%s' to 'mono8'.",
                         image_msg_left->encoding.c_str());
            return;
        }

        // left
        out_img_msg_left->header          = image_msg_left->header;
        out_img_msg_left->header.frame_id = output_frame_id_;
        output_camera_info_left_.header          = image_msg_left->header;
        output_camera_info_left_.header.frame_id = output_frame_id_;

        if (pub_img_left_rect_.getNumSubscribers() > 0)
        {
            pub_img_left_rect_.publish(out_img_msg_left);
        }
        if (pub_camera_info_left_->get_subscription_count() > 0)
        {
            pub_camera_info_left_->publish(output_camera_info_left_);
        }

        // right
        out_img_msg_right->header          = image_msg_right->header;
        out_img_msg_right->header.frame_id = output_frame_id_;
        output_camera_info_right_.header          = image_msg_right->header;
        output_camera_info_right_.header.frame_id = output_frame_id_;

        if (pub_img_right_rect_.getNumSubscribers() > 0)
        {
            pub_img_right_rect_.publish(out_img_msg_right);
        }
        if (pub_camera_info_right_->get_subscription_count() > 0)
        {
            pub_camera_info_right_->publish(output_camera_info_right_);
        }
    }

    void t265Depth::publishCameraInfo(cv::Mat K1, cv::Mat K2, cv::Mat P1, cv::Mat P2, cv::Mat R1, cv::Mat R2)
    {
        for (int i = 0; i < 9; i++)
        {
            output_camera_info_left_.k[i]  = K1.at<float>(i);
            output_camera_info_right_.k[i] = K2.at<float>(i);
            output_camera_info_left_.r[i]  = R1.at<float>(i);
            output_camera_info_right_.r[i] = R2.at<float>(i);
        }
        for (int i = 0; i < 12; i++)
        {
            output_camera_info_left_.p[i]  = P1.at<float>(i);
            output_camera_info_right_.p[i] = P2.at<float>(i);
        }
    }

    void t265Depth::initializeRectificationMapping(std::string param_file_path)
    {
        cv::Mat1f Q, P1, P2;
        cv::Mat1f R1, R2, K1, K2, D1, D2, R;
        cv::Vec3f T;
        cv::Vec2f size_input, size_output;

        cv::FileStorage param_file = cv::FileStorage(param_file_path, cv::FileStorage::READ);

        param_file["K1"] >> K1;
        param_file["K2"] >> K2;
        param_file["D1"] >> D1;
        param_file["D2"] >> D2;
        param_file["R"]  >> R;
        param_file["T"]  >> T;

        param_file["input"]  >> size_input;
        param_file["output"] >> size_output;

        cv::Size input_img_size(size_input[0],  size_input[1]);
        cv::Size output_img_size(size_output[0], size_output[1]);

        output_camera_info_left_.width   = size_output[0];
        output_camera_info_left_.height  = size_output[1];
        output_camera_info_left_.d       = std::vector<double>(5, 0);

        output_camera_info_right_.width  = size_output[0];
        output_camera_info_right_.height = size_output[1];
        output_camera_info_right_.d      = std::vector<double>(5, 0);

        int max_disp = min_disparity_ + num_disparities_;
        float stereo_fov_rad    = 90 * (M_PI / 180);
        float stereo_width_px   = size_output[0] + max_disp;
        float stereo_height_px  = size_output[1];
        float stereo_focal_px   = stereo_height_px / 2 / tan(stereo_fov_rad / 2);

        cv::Mat R_left  = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat R_right = R;

        cv::Size stereo_size = cv::Size(stereo_width_px, stereo_height_px);

        stereo_cx = (stereo_width_px  - 1) / 2 + max_disp;
        stereo_cy = (stereo_height_px - 1) / 2;

        cout << "stereo_focal_px: "  << stereo_focal_px  << "\n"
             << "stereo_height_px: " << stereo_height_px << "\n"
             << "stereo_width_px: "  << stereo_width_px  << "\n"
             << "stereo_cx: "        << stereo_cx        << "\n"
             << "stereo_cy: "        << stereo_cy        << "\n"
             << "max_disp: "         << max_disp         << "\n";

        cv::Mat_<float> P_left(3, 4);
        P_left << stereo_focal_px, 0, stereo_cx, 0,
                  0, stereo_focal_px, stereo_cy, 0,
                  0, 0, 1, 0;

        cv::Mat_<float> P_right(3, 4);
        P_left.copyTo(P_right);
        P_right.at<float>(0, 3) = T[0] * stereo_focal_px;

        Q = (cv::Mat_<float>(4, 4) << 1, 0, 0, -(stereo_cx - max_disp),
             0, 1, 0, -stereo_cy,
             0, 0, 0, stereo_focal_px,
             0, 0, (-1 / T[0]), 0);

        cv::fisheye::initUndistortRectifyMap(K1, D1, R_left,  P_left,  stereo_size, CV_32FC1, lmapx_, lmapy_);
        cv::fisheye::initUndistortRectifyMap(K2, D2, R_right, P_right, stereo_size, CV_32FC1, rmapx_, rmapy_);
        publishCameraInfo(K1, K2, P_left, P_right, R_left, R_right);
        focal_length = P_left.at<float>(0, 0);
        baseline     = std::abs(T[0]);

        cout << "focal_length: " << focal_length << "\n"
             << "baseline: "     << baseline     << "\n";

        RCLCPP_INFO(get_logger(), "[t265Depth] Initialization complete");
    }

    void t265Depth::computePointcloud(const cv::Mat &input_disparity,
                                      sensor_msgs::msg::PointCloud2 &pointcloud)
    {
        int side_bound = sad_window_size_ / 2;
        sensor_msgs::PointCloud2Modifier modifier(pointcloud);
        pointcloud.width  = input_disparity.cols - side_bound;
        pointcloud.height = input_disparity.rows - side_bound;
        modifier.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> pointcloud_x(pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float> pointcloud_y(pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float> pointcloud_z(pointcloud, "z");

        for (int y_pixels = side_bound; y_pixels < input_disparity.rows - side_bound;
             ++y_pixels)
        {
            for (int x_pixels = side_bound + min_disparity_ + num_disparities_;
                 x_pixels < input_disparity.cols - side_bound; ++x_pixels)
            {
                const int16_t &input_value = input_disparity.at<int16_t>(y_pixels, x_pixels);
                double disparity_value = static_cast<double>(input_value);

                // OpenCV stores disparity maps as 16x the true values
                if (disparity_value > 50 && disparity_value < 800)
                {
                    *pointcloud_z = (16 * focal_length * baseline) / disparity_value;
                    *pointcloud_x = *pointcloud_z * (x_pixels - stereo_cx) / focal_length;
                    *pointcloud_y = *pointcloud_z * (y_pixels - stereo_cy) / focal_length;
                }

                ++pointcloud_x;
                ++pointcloud_y;
                ++pointcloud_z;
            }
        }
    }

    void t265Depth::elaborateImages(const std_msgs::msg::Header &header_msg)
    {
        cv::Mat left_disp, left_disp8u;

        if (image_left_.rows > 0 && image_right_.rows > 0)
        {
            cv::remap(image_left_,  undist_image_left_,  lmapx_, lmapy_, cv::INTER_LINEAR);
            cv::remap(image_right_, undist_image_right_, rmapx_, rmapy_, cv::INTER_LINEAR);

            if (use_sgbm_)
            {
                int mode;
                if (sgbm_mode_ == 1)
                {
                    mode = cv::StereoSGBM::MODE_HH;
                }
                else if (sgbm_mode_ == 2)
                {
                    mode = cv::StereoSGBM::MODE_SGBM_3WAY;
                }
                else
                {
                    mode = cv::StereoSGBM::MODE_SGBM;
                }

                cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
                    min_disparity_,
                    num_disparities_,
                    sad_window_size_,
                    p1_,
                    p2_,
                    disp_12_max_diff_,
                    pre_filter_cap_,
                    uniqueness_ratio_,
                    speckle_window_size_,
                    speckle_range_,
                    mode);
                left_matcher->compute(undist_image_left_, undist_image_right_, left_disp);
            }
            else
            {
                cv::Ptr<cv::StereoBM> left_matcher =
                    cv::StereoBM::create(num_disparities_, sad_window_size_);

                left_matcher->setPreFilterType(pre_filter_type_);
                left_matcher->setPreFilterSize(pre_filter_size_);
                left_matcher->setPreFilterCap(pre_filter_cap_);
                left_matcher->setMinDisparity(min_disparity_);
                left_matcher->setTextureThreshold(texture_threshold_);
                left_matcher->setUniquenessRatio(uniqueness_ratio_);
                left_matcher->setSpeckleRange(speckle_range_);
                left_matcher->setSpeckleWindowSize(speckle_window_size_);
                left_matcher->compute(undist_image_left_, undist_image_right_, left_disp);
            }

            if (do_median_blur_)
            {
                cv::medianBlur(left_disp, left_disp, 3);
            }

            if (pub_disparity_.getNumSubscribers() > 0)
            {
                cv::normalize(left_disp, left_disp8u, 0, 255, cv::NORM_MINMAX, CV_8U);
                auto out_disparity_msg =
                    cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", left_disp8u).toImageMsg();
                pub_disparity_.publish(out_disparity_msg);
            }

            sensor_msgs::msg::PointCloud2 pointcloud_msg;
            computePointcloud(left_disp, pointcloud_msg);
            pointcloud_msg.header.stamp    = header_msg.stamp;
            pointcloud_msg.header.frame_id = output_frame_id_;

            if (pub_pointcloud_->get_subscription_count() > 0)
            {
                pub_pointcloud_->publish(pointcloud_msg);
            }
        }
    }

} // namespace t265_depth
