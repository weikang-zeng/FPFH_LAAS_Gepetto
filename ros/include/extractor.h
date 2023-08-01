#ifndef APP_H
#define APP_H

#include <iostream>
#include <vector>
#include <limits>
#include <math.h>

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <eigen_conversions/eigen_msg.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>

//PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d_omp.h>

//Boost
#include <boost/thread/thread.hpp>
#include <boost/system/linux_error.hpp>

//! Base value for the output dir, the class enables it modification in the constructors.
#define BASE_DIR_OUTPUT "/local/users/tlasguigne/work/output/mfpfh"

namespace multiscale_fpfh{

    struct ExtractorConfig{
        bool for_model = true;
        Eigen::Isometry3d objInWorld = Eigen::Isometry3d::Identity();
        std::string objectName = "";
        std::string outputFolder = "";
        int resolution = -1;
        int th_resolution = -1;
        int ph_resolution = -1;
        bool save_normals = false;
    };

    /**
     * @brief Extractor class. Runs the cloud extraction for the multiscale fpfh problem.
     */
    class Extractor{
    private:
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> CloudT;
        typedef pcl::PointNormal PointNormT;
        typedef pcl::PointCloud<PointNormT> CloudPointNormT;
        typedef pcl::Normal NormalT;
        typedef pcl::PointCloud<NormalT> CloudNormalT;

        ExtractorConfig config_;
        ros::NodeHandle* nh_;
        bool isFinished_;
        ros::ServiceClient setModelClient_;
        ros::Publisher debug_cloud_pub_;

        pcl::NormalEstimationOMP<PointT, NormalT> normalEstimation_;

        /**
         * @brief Check if the path given exist. Otherwise, create the directories needed. It also create the last directory (given the object name).
         * 
         */
        void checkPathAndDir();

        /**
         * @brief Set the new pose of the sensor
         * 
         * @param sensorPose pose to be set.
         * @return if the pose has been set. //!not implemented yet, returning true
         */
        bool setNewPose(Eigen::Isometry3d sensorPose);

        /**
         * @brief change a spherical coordinate set in a Eigen::Isometry3d
         * 
         * @param radius radial distance.
         * @param theta polar angle (between radial segment and z-axis)
         * @param phi azimuth angle (to x-axis on the xy-plane)
         * @return Eigen::Isometry3d 
         */
        Eigen::Isometry3d spherical2cartesian(float radius, float theta, float phi);

        std::string defineFilename(float radius, Eigen::Isometry3f pose, Eigen::Quaternionf q, bool wo);

        /**
         * @brief Save the given point cloud
         * 
         * @param cloud cloud to be saved.
         * @param radius used in the name.
         * @param sensorPose used in the name and in the file.
         * @return if the cloud has been successfully saved.
         */
        template <typename CloudSaveT>
        bool saveCloud(CloudSaveT cloud, float radius, Eigen::Isometry3d sensorPose);
        template <typename CloudSaveT>
        bool saveCloudWoInfos(CloudSaveT cloud, float radius, Eigen::Isometry3d sensorPose);

        template <typename CloudSaveT>
        void DebugPublishCloud(CloudSaveT cloud, std::string frame);
        
        template <typename CloudSaveT>
        void post_process(CloudSaveT cloud, float radius, Eigen::Isometry3d pose);

    public:
        /**
         * @brief Construct a new Extractor object
         * 
         * @warning a ros::NodeHandle object is needed. See the other constructors.
         */
        Extractor();
        /**
         * @brief Construct a new Extractor object
         * 
         * @param nh  Node Handle (mandatory)
         * @param config Configuration structure
         */
        Extractor(ros::NodeHandle* nh, ExtractorConfig config=ExtractorConfig());
        /**
         * @brief Destroy the Extractor object
         */
        ~Extractor();
        
        /**
         * @brief Run the pipeline.
         */
        void run(void);
    };

} // namespace multiscale_fpfh

#include "impl/extractor_tmpl.hpp"

#endif