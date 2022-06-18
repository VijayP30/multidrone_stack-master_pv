#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>

class CamInfoPublisher {
private:
    ros::Subscriber normalCameraInfoSubscriber;
    ros::Publisher fixedCameraInfoSubscriber;
    
    sensor_msgs::CameraInfo actualCameraInfo;

    void receiveMsg(sensor_msgs::CameraInfo cameraInfo) {
        cameraInfo.D = actualCameraInfo.D;
        cameraInfo.K = actualCameraInfo.K;
        cameraInfo.R = actualCameraInfo.R;
        cameraInfo.P = actualCameraInfo.P;
        cameraInfo.width = actualCameraInfo.width;
        cameraInfo.height = actualCameraInfo.height;

        fixedCameraInfoSubscriber.publish(cameraInfo);
    }

public:

    CamInfoPublisher(ros::NodeHandle nh) {
        actualCameraInfo.width = 1920;
        actualCameraInfo.height = 1080;

        actualCameraInfo.D = {0.0008863891559004586, 0.001116468317310022, 0.0027715126904969026, 0.0025984212919268742, 0.0};
        actualCameraInfo.K = {940.8746119321808, 0.0, 966.6962208261726, 0.0, 942.20071151189, 548.3741522912533, 0.0, 0.0, 1.0};
        actualCameraInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        actualCameraInfo.P = {946.546875, 0.0, 974.8538871221244, 0.0, 0.0, 949.81640625, 553.4697523161303, 0.0, 0.0, 0.0, 1.0, 0.0};

        normalCameraInfoSubscriber = nh.subscribe("old_camera_info", 1, &CamInfoPublisher::receiveMsg, this);
        fixedCameraInfoSubscriber = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "broadcast_actual_camera_info");
    ros::NodeHandle nh;

    CamInfoPublisher broadcaster(nh);

    ros::spin();

    return 0;
}