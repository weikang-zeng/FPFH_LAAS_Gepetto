#ifndef OBJECTSEGMENTATIONPLANE_H
#define OBJECTSEGMENTATIONPLANE_H

// *************** Class to do clustering of Objects on a Plane *********************************
// * Extracted from Localisation system developped by Gopi Krishna Erabati
// * Improved by Thibaud Lasguignes

#include <pcl/common/common.h>
#include <pcl/common/time.h>
//for getting plane indices and coeff using sac segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
//for extracting pc from pc using indices
#include <pcl/filters/extract_indices.h>
//for euclidean clusters
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
//for projecting
#include <pcl/filters/project_inliers.h>
//for convex hull
#include <pcl/surface/convex_hull.h>
//to get polygonal data
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>

namespace multiscale_fpfh{

    //class to segment objects on top of table and return the clusters of objects
    /**
     * @brief Class to segment objects on top of a plane and return the clusters of objects
     * 
     * @tparam PointT PCL point type of the data.
     */
    template<typename PointT = pcl::PointXYZ>
    class ObjectSegmentationPlane{
    private:
        typedef pcl::PointCloud<PointT> CloudT;

        // SACSegmentation object
        pcl::SACSegmentation<PointT> sacSegmenter_;

        // Extract Indices Object
        pcl::ExtractIndices<PointT> indicesExtractor_;

        // cluster extraction object
        pcl::EuclideanClusterExtraction<PointT> euclideanClusterExtractor_;

        // projet inliers object
        pcl::ProjectInliers<PointT> inliersProjector_;
    public:
        /**
         * @brief Construct a new Object Segmentation Plane object
         */
        ObjectSegmentationPlane(){}

        /**
         * @brief function to do planar segmentation basing on pcl::SACSegmentation and return the indices of plane and coeff
         * 
         * @param p_cloudInput pointer to the input point cloud
         * @param p_indices [out] indices of the plane's points in the input point cloud
         * @param p_modelCoeff [out] coefficients of the detected plane ([0]x+[1]y+[3]z+[4]=0)
         * @return true a plane has been detected
         * @return false no plane has been detected (or error occurred)
         */
        bool getPlaneIndicesAndCoeffSAC(typename CloudT::Ptr p_cloudInput, pcl::PointIndices::Ptr p_indices, pcl::ModelCoefficients::Ptr p_modelCoeff);

        /**
         * @brief function to extract plane and non-plane cloud
         * 
         * @param p_cloudInput pointer to the input point cloud
         * @param p_indices indices of the plane's points in the input point cloud
         * @param p_cloudPlane Points of the plane
         * @param p_cloudNotPlane points not of the plane
         */
        void getPlaneAndNonPlaneCloud(typename CloudT::Ptr p_cloudInput, pcl::PointIndices::Ptr p_indices, typename CloudT::Ptr p_cloudPlane, typename CloudT::Ptr p_cloudNotPlane);

        /**
         * @brief function to perform euclidean clustering and return cluster indices
         * 
         * @param p_cloudInput pointer to the input point cloud
         * @return std::vector<pcl::PointIndices> vector of indices of the objects
         */
        std::vector<pcl::PointIndices> getClusters(typename CloudT::Ptr p_cloudInput);

        /**
         * @brief function to project inliers into plane
         * 
         * @param p_cloudInput pointer to the input point cloud
         * @param p_indices indices of the plane's points in the input point cloud
         * @param p_modelCoeff coefficients of the detected plane ([0]x+[1]y+[3]z+[4]=0)
         * @return typename CloudT::Ptr projected cloud
         */
        void getProjectedCloud(typename CloudT::Ptr p_cloudInput, pcl::PointIndices::Ptr p_indices, pcl::ModelCoefficients::Ptr p_modelCoeff, typename CloudT::Ptr& p_cloudOutput);

        /**
         * @brief function to create convex hull
         * 
         * @param p_cloudInput pointer to the input point cloud
         * @return typename CloudT::Ptr convex hull
         */
        // typename CloudT::Ptr getConvexHull(typename CloudT::Ptr p_cloudInput);
        void getConvexHull(typename CloudT::Ptr p_cloudInput, typename CloudT::Ptr& p_cloudOutput);

        /**
         * @brief function to take and point cloud and return vector of pointcloud clusters using ConvexHull
         * 
         * @param p_cloudInput pointer to the input point cloud
         * @param cloudClusterVector vector of point clouds clustered
         * @param p_cloudPlane plane's point cloud
         * @return true segmentation succeeded
         * @return false segmentation failed
         */
        bool getSegmentedObjectsOnPlane(typename CloudT::Ptr p_cloudInput, std::vector<typename CloudT::Ptr> &cloudClusterVector, typename CloudT::Ptr &p_cloudPlane);

        bool getReducedCloud(typename CloudT::Ptr p_cloudIn, typename CloudT::Ptr& p_cloudOut, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
        bool getReducedCloud(typename CloudT::Ptr p_cloudIn, typename CloudT::Ptr& p_cloudOut, std::vector<float> limits);
    };

} // namespace multiscale_fpfh

#endif // OBJECTSEGMENTATIONPLANE_H
