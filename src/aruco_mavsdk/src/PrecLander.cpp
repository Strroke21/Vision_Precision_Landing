#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavlink/common/mavlink.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip> 
#include <cmath>
#include <atomic>  // For thread-safe flag
#include <vector>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace mavsdk;

//data stream function
bool enable_data_stream(System& system, uint8_t stream_id, uint16_t rate) {
    MavlinkPassthrough mavlink_passthrough(system);

    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,
        system.get_system_id(),
        1, // target component
        stream_id,
        rate,
        1  // start streaming (1 = start, 0 = stop)
    );

    return mavlink_passthrough.send_message(msg) == MavlinkPassthrough::Result::Success;
}


void send_land_msg(System& system, float ang_x, float ang_y)
{
    MavlinkPassthrough mavlink_passthrough{system};

    mavlink_message_t msg;

    mavlink_msg_landing_target_pack(
        mavlink_passthrough.get_our_sysid(),
        mavlink_passthrough.get_our_compid(),
        &msg,

        0,                          // time_usec
        0,                          // target_num
        MAV_FRAME_BODY_OFFSET_NED,         // frame
        ang_x,                      // angle_x (radians)
        ang_y,                      // angle_y (radians)
        0.0f,                       // distance (meters)
        0.0f,                       // size_x
        0.0f,                       // size_y
        0.0f,                       // x
        0.0f,                       // y
        0.0f,                       // z
        nullptr,                    // q[4] quaternion
        LANDING_TARGET_TYPE_VISION_FIDUCIAL,
        0                           // position_valid
    );

    auto result = mavlink_passthrough.send_message(msg);
    std::cout << "LANDING_TARGET message sent with ang_x: " << ang_x << " and ang_y: " << ang_y << std::endl;

    if (result != MavlinkPassthrough::Result::Success) {
        std::cout << "LANDING_TARGET send failed\n";
    }
}


void aruco_lander(System& system, cv::Mat& frame, int id_to_find, float marker_size, cv::Mat cameraMatrix, cv::Mat cameraDistortion, float hfov_rad, float vfov_rad, int image_width, int image_height)
{
    static cv::aruco::Dictionary aruco_dict =
        cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_ARUCO_ORIGINAL
        );

    static cv::aruco::DetectorParameters parameters;

    static cv::aruco::ArucoDetector detector(
        aruco_dict,
        parameters
    );

    cv::Mat gray;

    // -----------------------------
    // Preprocessing
    // -----------------------------
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(
        gray,
        gray,
        cv::Size(5,5),
        0
    );

    cv::equalizeHist(gray, gray);

    // -----------------------------
    // Detection
    // -----------------------------
    std::vector<int> ids;

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;

    detector.detectMarkers(
        gray,
        corners,
        ids,
        rejected
    );

    // Draw rejected candidates
    for (auto &r : rejected)
    {
        for (int i = 0; i < 4; i++)
        {
            cv::line(
                frame,
                r[i],
                r[(i+1)%4],
                cv::Scalar(255,0,0),
                1
            );
        }
    }

    if (!ids.empty())
    {
        cv::aruco::drawDetectedMarkers(
            frame,
            corners,
            ids
        );

        std::vector<cv::Vec3d> rvecs, tvecs;

        cv::aruco::estimatePoseSingleMarkers(
            corners,
            marker_size,
            cameraMatrix,
            cameraDistortion,
            rvecs,
            tvecs
        );

        for (size_t i = 0; i < ids.size(); i++)
        {
            std::cout
                << "Detected ID: "
                << ids[i]
                << std::endl;

            if (ids[i] == id_to_find)
            {
                cv::Vec3d rvec = rvecs[i];
                cv::Vec3d tvec = tvecs[i];

                // Rotation matrix
                cv::Mat R;

                cv::Rodrigues(rvec, R);

                // Yaw
                double yaw =
                    fmod(
                        atan2(
                            R.at<double>(1,0),
                            R.at<double>(0,0)
                        ) * 180.0 / CV_PI + 360.0,
                        360.0
                    );

                // Roll
                double roll =
                    fmod(
                        atan2(
                            R.at<double>(2,1),
                            R.at<double>(2,2)
                        ) * 180.0 / CV_PI + 360.0,
                        360.0
                    );

                // Pitch
                double pitch =
                    fmod(
                        atan2(
                            -R.at<double>(2,0),
                            sqrt(
                                pow(R.at<double>(2,1),2) +
                                pow(R.at<double>(2,2),2)
                            )
                        ) * 180.0 / CV_PI + 360.0,
                        360.0
                    );

                float x_sum = corners[0][0].x + corners[0][1].x + corners[0][2].x + corners[0][3].x;
                float y_sum = corners[0][0].y + corners[0][1].y + corners[0][2].y + corners[0][3].y;
                float x_avg = x_sum / 4.0f;
                float y_avg = y_sum / 4.0f;
                float x_ang = (x_avg - image_width / 2.0f) * (hfov_rad / image_width);
                float y_ang = (y_avg - image_height / 2.0f) * (vfov_rad / image_height);
                send_land_msg(system, x_ang, y_ang);

                std::cout
                    << "Yaw: " << yaw
                    << " Roll: " << roll
                    << " Pitch: " << pitch
                    << " X: " << tvec[0]
                    << " Y: " << tvec[1]
                    << " Z: " << tvec[2]
                    << " x_ang: " << x_ang
                    << " y_ang: " << y_ang
                    << std::endl;

                cv::drawFrameAxes(
                    frame,
                    cameraMatrix,
                    cameraDistortion,
                    rvec,
                    tvec,
                    0.1
                );

                cv::putText(
                    frame,
                    "Z: " + std::to_string(tvec[2]),
                    cv::Point(10,30),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(0,255,0),
                    2
                );
            }
        }
    }
    else
    {
        std::cout
            << "ARUCO "
            << id_to_find
            << " NOT FOUND"
            << std::endl;
    }
}


int main(int argc, char **argv){
    std::string fcu_address = "udpin://127.0.0.1:14550"; // FCU address
    Mavsdk::Configuration config{ComponentType::GroundStation};
    Mavsdk mavsdk(config);

    ConnectionResult connection_result = mavsdk.add_any_connection(fcu_address);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << std::endl;
        return 1;
    }

    std::shared_ptr<System> system;
    while (true) {
        auto systems = mavsdk.systems();
        if (!systems.empty()) {
            system = systems.at(0);
            break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    enable_data_stream(*system, MAV_DATA_STREAM_ALL,100);
    std::cout << "Data stream enabled." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    Telemetry telemetry{system}; //initialize telemetry
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("aruco_lander");

    // -----------------------------
    // Variables
    // -----------------------------
    cv::Mat latest_frame;

    cv::Mat cameraMatrix;
    cv::Mat cameraDistortion;

    bool camera_info_received = false;

    int id_to_find = 72;

    float marker_size = 30.0f;
    float hfov_rad = 90.0f * CV_PI / 180.0f; 
    float vfov_rad = 65.0f * CV_PI / 180.0f;
    int image_width = 640;
    int image_height = 480;

    // -----------------------------
    // Camera Info Subscriber
    // -----------------------------
    auto camera_info_sub =
        node->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera1/camera_info",
            1,
            [&](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
            {
                cameraMatrix = (cv::Mat_<double>(3,3) <<
                    msg->k[0], msg->k[1], msg->k[2],
                    msg->k[3], msg->k[4], msg->k[5],
                    msg->k[6], msg->k[7], msg->k[8]);

                cameraDistortion = cv::Mat(msg->d).clone();

                camera_info_received = true;
            });

    // -----------------------------
    // Image Subscriber
    // -----------------------------
    auto image_sub =
        node->create_subscription<sensor_msgs::msg::Image>(
            "/camera1/image_raw",
            1,
            [&](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                if (!camera_info_received)
                {
                    return;
                }

                try
                {
                    latest_frame = cv_bridge::toCvCopy(msg,"bgr8")->image;

                    aruco_lander(*system, latest_frame,id_to_find,marker_size,cameraMatrix, cameraDistortion,hfov_rad,
                        vfov_rad,
                        image_width,
                        image_height);

                    //cv::imshow("frame", latest_frame);

                    //cv::waitKey(1);
                }
                catch (cv_bridge::Exception &e)
                {
                    std::cout
                        << "cv_bridge exception: "
                        << e.what()
                        << std::endl;
                }
            });

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;

}