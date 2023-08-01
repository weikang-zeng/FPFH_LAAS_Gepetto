#ifndef EXTRACTOR_TMPL_H
#define EXTRACTOR_TMPL_H

#include "extractor.h"

namespace multiscale_fpfh{

    template <typename CloudSaveT>
    void Extractor::post_process(CloudSaveT cloud, float radius, Eigen::Isometry3d pose){
        ROS_INFO("Got Pose: x:=%f y:=%f z:=%f", pose.translation().x(), pose.translation().y(), pose.translation().z());
        Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();
        if(config_.for_model){
            //! for model => expressed in object frame
            //******* Sensor in obj ********//
            transformation = config_.objInWorld.inverse().matrix() * pose.matrix();
            //******** Cloud in obj ********//
            pcl::transformPointCloud(cloud, cloud, transformation.matrix().cast<float>());
        } else {
            //! not for model => expressed in sensor frame, transform is pose of the object in sensor frame
            //******* Obj in sensor ********//
            transformation = pose.inverse().matrix() * config_.objInWorld.matrix();
        }
        ROS_INFO("computed transformation: x:=%f y:=%f z:=%f", transformation.translation().x(), transformation.translation().y(), transformation.translation().z());
        DebugPublishCloud(cloud, "object");
        //********* Save cloud *********//
        if(!saveCloud(cloud, radius, transformation)){
            ROS_INFO_ONCE("Be careful, a cloud hasn't been saved. (This message will only show once).");
        }
        if(!saveCloudWoInfos(cloud, radius, transformation)){
            ROS_INFO_ONCE("Be careful, a cloud hasn't been saved. (This message will only show once).");
        }
    }

    template <typename CloudSaveT>
    void Extractor::DebugPublishCloud(CloudSaveT cloud, std::string frame){
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = frame;
        debug_cloud_pub_.publish(msg);
    }

    template <typename CloudSaveT>
    bool Extractor::saveCloud(CloudSaveT cloud, float radius, Eigen::Isometry3d sensorPose){
        Eigen::Isometry3f sensorPoseF = sensorPose.cast<float>();
        //! above needed because sensorPoseF.rotation()can't be accessed as quaternion
        Eigen::Quaternionf qSensorRotF (sensorPoseF.rotation());

        std::string filename = defineFilename(radius, sensorPoseF, qSensorRotF, false);
        ROS_INFO("Saving cloud <%s>...", filename.c_str());

        cloud.sensor_orientation_.x() = qSensorRotF.x();
        cloud.sensor_orientation_.y() = qSensorRotF.y();
        cloud.sensor_orientation_.z() = qSensorRotF.z();
        cloud.sensor_orientation_.w() = qSensorRotF.w();
        cloud.sensor_origin_.x() = sensorPoseF.translation()(0);
        cloud.sensor_origin_.y() = sensorPoseF.translation()(1);
        cloud.sensor_origin_.z() = sensorPoseF.translation()(2);

        if(!pcl::io::savePCDFile(filename, cloud)){
            ROS_INFO("Saved.");
            return true;
        } else {
            ROS_ERROR("Error while saving <%s>.", filename.c_str());
            return false;
        }
    }
    template <typename CloudSaveT>
    bool Extractor::saveCloudWoInfos(CloudSaveT cloud, float radius, Eigen::Isometry3d sensorPose){
        Eigen::Isometry3f sensorPoseF = sensorPose.cast<float>();
        //! above needed because sensorPoseF.rotation()can't be accessed as quaternion
        Eigen::Quaternionf qSensorRotF (sensorPoseF.rotation());

        std::string filename = defineFilename(radius, sensorPoseF, qSensorRotF, true);
        ROS_INFO("Saving cloud <%s>...", filename.c_str());

        cloud.sensor_orientation_.x() = 0.;
        cloud.sensor_orientation_.y() = 0.;
        cloud.sensor_orientation_.z() = 0.;
        cloud.sensor_orientation_.w() = 1.;
        cloud.sensor_origin_.x() = 0.;
        cloud.sensor_origin_.y() = 0.;
        cloud.sensor_origin_.z() = 0.;

        if(!pcl::io::savePCDFile(filename, cloud)){
            ROS_INFO("Saved.");
            return true;
        } else {
            ROS_ERROR("Error while saving <%s>.", filename.c_str());
            return false;
        }
    }


} // namespace multiscale_fpfh

#endif