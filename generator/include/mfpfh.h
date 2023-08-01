#ifndef MFPFH_H
#define MFPFH_H

#include <iostream>
#include <vector>
#include <limits>
#include <math.h>

//PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

//Boost
#include <boost/thread/thread.hpp>
#include <boost/system/linux_error.hpp>

//! Base value for the output dir, the class enables it modification in the constructors.
#define BASE_DIR_OUTPUT "/local/users/tlasguigne/work/output/mfpfh"

namespace multiscale_fpfh{

    /**
     * @brief MFPFHEstimation class. Runs the definition of the descriptors.
     */
    class MFPFHEstimation{
    private:
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> CloudT;
        typedef pcl::PointNormal PointWNormT;
        typedef pcl::PointCloud<PointWNormT> CloudWNormT;
        typedef pcl::Normal NormalT;
        typedef pcl::PointCloud<NormalT> CloudNormT;
        typedef pcl::FPFHSignature33 SignatureT;
        typedef pcl::PointCloud<SignatureT> CloudSignT;


        std::string objName_;
        std::string savePath_;
        bool isFinished_;
        Eigen::Isometry3d transformObjInWorld_;

        float highScale_, lowScale_;

        pcl::NormalEstimation<PointT, NormalT> normalEstimation_;
        pcl::FPFHEstimationOMP<PointT, NormalT, SignatureT> fpfhEstimation_;
        pcl::FPFHEstimationOMP<PointWNormT, PointWNormT, SignatureT> fpfhEstimationWNorm_;

        /**
         * @brief Check if the path given exist. Otherwise, create the directories needed. It also create the last directory (given the object name).
         * 
         */
        void checkPathAndDir();

        /**
         * @brief load a point cloud
         * 
         * @param filepath [in] the path object to the file
         * @param cloud [out] the cloud extracted
         * @return true cloud extracted
         * @return false cloud not extracted (a message will be displayed on the terminal)
         */
        template <typename CloudInT>
        bool loadCloud(boost::filesystem::path filepath, CloudInT& cloud);

    public:
        /**
         * @brief Construct a new MFPFHEstimation object
         * 
         * @warning a ros::NodeHandle object is needed. See the other constructors.
         */
        MFPFHEstimation();
        /**
         * @brief Construct a new MFPFHEstimation object
         * 
         * @param objName [in] Name of the object. Will be used to define a specific output directory and to name the files.
         */
        MFPFHEstimation(std::string objName);
        /**
         * @brief Construct a new MFPFHEstimation object
         * 
         * @param objInWorld [in] Transformation applied to the object (if done in the running simulation).
         * @note if the directory (and its path) does not exist, the system will try to create it.
         */
        MFPFHEstimation(Eigen::Isometry3d objInWorld);
        /**
         * @brief Construct a new MFPFHEstimation object
         * 
         * @param objName [in] Name of the object. Will be used to define a specific output directory and to name the files.
         * @param savePath [in] Path to the wanted output directory.
         * @note if the directory (and its path) does not exist, the system will try to create it.
         */
        MFPFHEstimation(std::string objName, std::string savePath);
        /**
         * @brief Construct a new MFPFHEstimation object
         * 
         * @param objName [in] Name of the object. Will be used to define a specific output directory and to name the files.
         * @param savePath [in] Path to the wanted output directory.
         * @param objInWorld [in] Transformation applied to the object (if done in the running simulation).
         * @note if the directory (and its path) does not exist, the system will try to create it.
         */
        MFPFHEstimation(std::string objName, Eigen::Isometry3d objInWorld);
        /**
         * @brief Construct a new MFPFHEstimation object
         * 
         * @param objName [in] Name of the object. Will be used to define a specific output directory and to name the files.
         * @param savePath [in] Path to the wanted output directory.
         * @param objInWorld [in] Transformation applied to the object (if done in the running simulation).
         * @note if the directory (and its path) does not exist, the system will try to create it.
         */
        MFPFHEstimation(std::string objName, std::string savePath, Eigen::Isometry3d objInWorld);
        /**
         * @brief Destroy the MFPFHEstimation object
         */
        ~MFPFHEstimation();
        
        /**
         * @brief Run the pipeline.
         */
        void run(void);

        /**
         * @brief Run the pipeline with normals in files.
         */
        void runWithNormals(void);

        /**
         * @brief Set the Limits for the scales
         * 
         * @param high [in] Higher scale
         * @param low [in] lower boundary
         * @note the set of scales will be defined as $S_r = \{s_i | s_{i+1} = s_i / 2, s_i > low, s_0 = high\}$
         */
        void setScalesLimits(float high, float low){highScale_ = high; lowScale_ = low;}

        /**
         * @brief Get the Limits for the scales
         * 
         * @param high [out] Higher scale
         * @param low [out] lower boundary
         */
        void getScalesLimits(float& high, float& low){high = highScale_; low = lowScale_;}
    };

} // namespace multiscale_fpfh


#endif