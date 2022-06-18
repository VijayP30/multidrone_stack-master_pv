#include <ros/ros.h>

#include <aprilslam/Apriltags.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Geometry>

std::ostream& operator<<(std::ostream& out, const tf2::Transform trans) {
    return out << "qw: " << trans.getRotation().getW() << ", qx: " << trans.getRotation().getX() << ", qy: " << trans.getRotation().getY() << ", qz: " << trans.getRotation().getZ() << std::endl
           << "x: " << trans.getOrigin().getX() << ", y: " << trans.getOrigin().getY() << ", z: " << trans.getOrigin().getZ() << std::endl;
}

std::ostream& operator<<(std::ostream& out, const Eigen::Quaterniond quat) {
    return out << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z();
}

Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond> &quats) {
    Eigen::Matrix4d q_qt = Eigen::Matrix4d::Zero();
    for (auto &quat : quats) {
        Eigen::Vector4d quatAsVec { quat.w(), quat.x() , quat.y(), quat.z() };
        q_qt += quatAsVec * quatAsVec.transpose();
    }
    q_qt /= quats.size();
    
    // Compute the SVD of this 4x4 matrix
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(q_qt, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::VectorXd singularValues = svd.singularValues();
	Eigen::MatrixXd U = svd.matrixU();

	// find the eigen vector corresponding to the largest eigen value
	int largestEigenValueIndex;
	double largestEigenValue;
	bool first = true;

	for (int i=0; i<singularValues.rows(); ++i)
	{
		if (first)
		{
			largestEigenValue = singularValues(i);
			largestEigenValueIndex = i;
			first = false;
		}
		else if (singularValues(i) > largestEigenValue)
		{
			largestEigenValue = singularValues(i);
			largestEigenValueIndex = i;
		}
	}

	Eigen::Quaterniond average;
	average.w() = U(0, largestEigenValueIndex);
	average.x() = U(1, largestEigenValueIndex);
	average.y() = U(2, largestEigenValueIndex);
	average.z() = U(3, largestEigenValueIndex);

    average.normalize();

    return average;
}


Eigen::Vector3d averageVectors(const std::vector<Eigen::Vector3d> &vecs) {
    Eigen::Vector3d ret = Eigen::Vector3d::Zero();

    for (auto &vec: vecs) {
        ret += vec;
    }

    ret /= vecs.size();

    return ret;
}
class Node {
private:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Subscriber apriltagSubscriber;

    std::vector<Eigen::Quaterniond> quats;
    std::vector<Eigen::Vector3d> vecs;

    void onReceiveApriltag(aprilslam::Apriltags apriltags) {
        tf2::Transform drone_Camera_transform;

        try {
            // Step 1: get transforms
            {
                auto drone_Tag = tfBuffer.lookupTransform("tracker1", "measured_camera_frame", ros::Time(0)).transform;
                tf2::fromMsg(drone_Tag, drone_Camera_transform);
            }

            // std::cout << "Transform:\n" << drone_Camera_transform << std::endl;
            quats.emplace_back(drone_Camera_transform.getRotation().getW(), drone_Camera_transform.getRotation().getX(), drone_Camera_transform.getRotation().getY(), drone_Camera_transform.getRotation().getZ());
            vecs.emplace_back(drone_Camera_transform.getOrigin().getX(), drone_Camera_transform.getOrigin().getY(), drone_Camera_transform.getOrigin().getZ());

            std::cout << "this transform\n" << drone_Camera_transform << std::endl;

            std::cout << "Average quat: " << averageQuaternions(quats) << std::endl;
            auto avg = averageVectors(vecs);
            std::cout << "Avegare location: " << avg.x() << " " << avg.y() << " " << avg.z() << std::endl << std::endl;
        } catch (...) {
            std::cout << "oogh" << std::endl;
        } 
    }

public:
    Node(ros::NodeHandle nh) : tfListener(tfBuffer) {
        apriltagSubscriber = nh.subscribe<aprilslam::Apriltags>("/camera/apriltags", 1, &Node::onReceiveApriltag, this);
    }

};

class PublishApriltagTransform {
private:
    tf2_ros::TransformBroadcaster tfBroadcaster;

    ros::Subscriber apriltagSubscriber;

    void onReceiveApriltag(aprilslam::Apriltags apriltags) {
        // Publish where we are measuring the camera frame to be
        {
            auto tag_pose = apriltags.apriltags.at(0).pose;
            tf2::Transform camera_Tag_transform;
            tf2::fromMsg(tag_pose, camera_Tag_transform);
            
            tf2::Transform tag_Camera_transform = camera_Tag_transform.inverse();

            geometry_msgs::TransformStamped sendToTf;
            sendToTf.transform = tf2::toMsg(tag_Camera_transform);
            sendToTf.child_frame_id = "measured_camera_frame";
            sendToTf.header.frame_id = "gt_tag_zup";
            sendToTf.header.seq = apriltags.header.seq;
            sendToTf.header.stamp = apriltags.header.stamp;

            tfBroadcaster.sendTransform(sendToTf);
        }

        // Publish where we are measuring the tag to be
        {
            auto tag_pose = apriltags.apriltags.at(0).pose;
            tf2::Transform camera_Tag_transform;
            tf2::fromMsg(tag_pose, camera_Tag_transform);

            geometry_msgs::TransformStamped sendToTf;
            sendToTf.transform = tf2::toMsg(camera_Tag_transform);
            sendToTf.child_frame_id = "measured_tag";
            sendToTf.header.frame_id = "camera_frame";
            sendToTf.header.seq = apriltags.header.seq;
            sendToTf.header.stamp = apriltags.header.stamp;

            tfBroadcaster.sendTransform(sendToTf);
        }
    }
public:
    PublishApriltagTransform(ros::NodeHandle nh) {
        apriltagSubscriber = nh.subscribe<aprilslam::Apriltags>("/camera/apriltags", 1, &PublishApriltagTransform::onReceiveApriltag, this);
    }

};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "find_camera_to_drone_transform");
    ros::NodeHandle nh;

    PublishApriltagTransform node(nh);

    // ros::Duration(5.0).sleep();
    Node node2(nh);


    ros::spin();

    return 0;
}