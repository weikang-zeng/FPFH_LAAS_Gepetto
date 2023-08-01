#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <math.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/ml/kmeans.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>

//Boost
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/system/linux_error.hpp>

//local
#include "objectsegmentationplane.h"

//! Base value for the output dir, the class enables it modification in the constructors.
#define BASE_DIR_OUTPUT "/local/users/tlasguigne/work/output/mfpfh"

//! value to get inf or NaN numbers obtained by k-means to an out of possible range value that will not infer the computation.
#define OUT_OF_COMPUTATION_FPFH 1000

namespace multiscale_fpfh{

    /**
     * @brief MFPFHLocalizer class. Used to localize an object using the MFPFH descriptor
     */
    class MFPFHLocalizer{
    private:
        //* Typedefs
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> CloudT;

        typedef pcl::PointNormal PointWNormT;
        typedef pcl::PointCloud<PointWNormT> CloudWNormT;

        typedef pcl::Normal NormalT;
        typedef pcl::PointCloud<NormalT> CloudNormT;

        typedef pcl::FPFHSignature33 SignatureT;
        typedef pcl::PointCloud<SignatureT> CloudSignT;

        typedef struct{
            float scale;
            CloudSignT::Ptr features;
            int k_compo;
            // k_compo
        } ModelFeaturesT;
        typedef struct{
            std::string partName;
            CloudT::Ptr points;
            std::vector<ModelFeaturesT> scales;
            //std::vector<ModelFeaturesT> k_value;
        } ModelCompleteT; // model for 1 view
        friend std::ostream& operator<<(std::ostream& os, std::vector<ModelCompleteT> model);

        struct ResultCompareT{
            int indexCluster;
            int indexInModel;
            float fitness;
            Eigen::Isometry3d pose;
            bool operator<(const ResultCompareT &b){
                return fitness < b.fitness;
            }
            bool operator>(const ResultCompareT &b){
                return fitness > b.fitness;
            }
        };
        typedef struct ResultCompareT ResultCompareT;
        friend std::ostream& operator<<(std::ostream& os, ResultCompareT result);
        /**
         * @brief compare two ResultCompareT by their fitnesses
         * 
         * @param a first object
         * @param b second object
         * @return true a < b
         * @return false a > b
         */
        bool compareComparisonResults(const ResultCompareT &a, const ResultCompareT &b);

        struct ResultSurpriseT{
            int indexReducedScene;
            int indexCompleteScene;
            int indexkClusters;
            float distance;
            bool operator<(const ResultSurpriseT &b){
                return distance < b.distance;
            }
            bool operator>(const ResultSurpriseT &b){
                return distance > b.distance;
            }
        };
        typedef struct ResultSurpriseT ResultSurpriseT;

        //* Generic variables
        std::string execDateStr_, execHourStr_;
        std::string objName_;               //? Object name
        std::string sceneFile_;             //? Path to the scene cloud
        std::string inputPath_;             //? Path given as input
        std::string outputPath_;            //? Path given as ouput
        std::string objectPath_;            //? constructed path for the object (after checked)
        std::string savePath_;              //? constructed path for the save directory (after checked)
        std::ofstream logFile_;
        Eigen::Isometry3d initialGuess_;

        //* Model variables
        std::vector<float> scalesObj_;
        std::vector<ModelCompleteT> modelObject_;
        std::vector<ModelCompleteT> swap_modelObject_;
        //std::vector<int>kvaluesObj_;
        std::vector<ModelFeaturesT>Allinone_;

        //* Scene point clouds
        CloudT::Ptr scenePoints_;
        CloudNormT::Ptr sceneNormals_;
        CloudSignT::Ptr sceneFeatures_;
        CloudT::Ptr planePoints_;
        std::vector<CloudT::Ptr> clustersPoints_;
        std::vector<CloudNormT::Ptr> clustersNormals_;
        std::vector<CloudSignT::Ptr> clustersFeatures_;

        std::vector<pcl::visualization::PCLVisualizer::Ptr> viewers_;

        //* Computation instances
        pcl::NormalEstimationOMP<PointT, NormalT> normal_estimator_;
        pcl::FPFHEstimationOMP<PointT, NormalT, SignatureT> fpfh_estimator_;
        pcl::SampleConsensusInitialAlignment<PointT, PointT, SignatureT> aligner_scia_;
        pcl::SampleConsensusPrerejective<PointT, PointT, SignatureT> aligner_scp_;
        ObjectSegmentationPlane<PointT> object_plane_segmenter_;

        

        /**
         * @brief generate a string containing the date of execution of the system (for logs)
         */
        void generateExecDate();

        /**
         * @brief extract the scale in the filename.
         * 
         * @param filename the name of the file analysed
         * @return float the scale extracted
         */
        float extractScaleFromFilename(std::string filename);

        /**
         * @brief extract the set of scales for each object.
         * 
         * @return true the set of scales of each objects has been extracted.
         * @return false the set of scales could not be extracted (explained on the console).
         */
        bool extractScales();


        /**
         * @brief extract the k_value in the filename.
         * 
         * @param filename the name of the file analysed
         * @return float the k_value extracted
         */
        float extractKvalueFromFilename(std::string filename);

        /**
         * @brief extract the k for each object.
         * 
         * @return true the  k of each objects has been extracted.
         */
        bool extract_value_k(std::vector<ModelCompleteT>& vectorOut, std::string folderObject);

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

        /**
         * @brief save a point cloud
         * 
         * @param filepath [in] the path object to the file
         * @param cloud [in] the cloud to save
         * @return true cloud saved
         * @return false cloud not saves (a message will be displayed on the terminal)
         */
        template <typename CloudInT>
        bool saveCloud(boost::filesystem::path filepath, CloudInT cloud);

        /**
         * @brief extract points clouds of a folder and put them in a vector
         * 
         * @param vectorOut [out] vector to complete
         * @param folderObject path to the folders
         * @return true extraction complete
         * @return false extraction not complete (a message will be displayed on the terminal)
         */
        bool extractClouds(std::vector<ModelCompleteT>& vectorOut, std::string folderObject);

        /**
         * @brief init the model vector with the scales that should be provided
         * 
         * @param vectorOut model vector
         * @param vectorScales vector of scales sizes
         */
        void initScales(std::vector<ModelCompleteT>& vectorOut, std::vector<float> vectorScales);


        /**
         * @brief extract features all in one of a folder and put them in the vector
         * 
         * @param vectorOut [out] vector to complete
         * @param folderObject path to the folders
         * @param vectorScales vector of scales sizes
         * @return true extraction complete
         * @return false extraction not complete (a message will be displayed on the terminal)
         */
        bool extractFeatures_allinone(std::vector<ModelFeaturesT>& vectorOut, std::string folderObject, std::vector<float> vectorScales);

        /**
         * @brief extract features sets of a folder and put them in the vector
         * 
         * @param vectorOut [out] vector to complete
         * @param folderObject path to the folders
         * @return true extraction complete
         * @return false extraction not complete (a message will be displayed on the terminal)
         */
        bool extractFeatures(std::vector<ModelCompleteT>& vectorOut, std::string folderObject);

        /**
         * @brief Give simple statistics on the model on the console.
         * 
         * @param model the model to observe.
         */
        void modelStatistics(std::vector<ModelCompleteT> model);

        /**
         * @brief Extract the object and complete the vectors
         * 
         * @return true extraction complete
         * @return false extraction not complete (a message will be displayed on the terminal)
         */
        bool extractObject();

        /**
         * @brief Extract the scene cloud and preprocess it (normals computation)
         * 
         * @return true complete
         * @return false not complete (a message will be displayed on the terminal)
         */
        bool extractScene();

        /**
         * @brief Preprocess the entry cloud
         * 
         * @return true if the preprocessing is done
         * @return false if the preprocessing has an error (information will be on the terminal)
         */
        bool preProcessing();

        /**
         * @brief Process the clusters to generate normals and features
         * 
         * @return true complete
         * @return false not complete (a message will be displayed on the terminal)
         */
        bool generateClustersFeatures();

        /**
         * @brief compare to the known scene
         * 
         * @param sourcePoints point cloud to compare
         * @param sourceFeatures features linked to the point cloud
         * @param isoOut (out) optimal pose found
         * @return float fitness resulting for the optimal pose
         */
        float compare2Scene(CloudT::Ptr sourcePoints, CloudSignT::Ptr sourceFeatures, Eigen::Isometry3d &isoOut);

        /**
         * @brief compare to the given cluster
         * 
         * @param sourcePoints point cloud to compare
         * @param sourceFeatures features linked to the point cloud
         * @param indexCluster index of the cluster to compare
         * @param isoOut (out) optimal pose found
         * @return float fitness resulting for the optimal pose
         */
        float compare2Cluster(CloudT::Ptr sourcePoints, CloudSignT::Ptr sourceFeatures, size_t indexCluster, Eigen::Isometry3d &isoOut);

        /**
         * @brief Find in the model the closest scale to the input
         * 
         * @param scale scale requested
         * @return float closest scale
         */
        float findClosestScale(float scale);


        /**
         * @brief Select the closest candidates in the scene, given a scale
         * 
         * @param vec_out Vector of the candidates
         * @param pc_candidates Point cloud of the candidates
         * @param scale The scale to study
         * @param kmean_nb The number of clusters
         */
        void selectCandidates(std::vector<ResultSurpriseT> &vec_out, CloudT::Ptr& pc_candidates, const CloudT::Ptr pc_source, const CloudNormT::Ptr pc_normals, const float scale, const int kmean_nb);
        void selectCandidates(std::vector<ResultSurpriseT> &vec_out, CloudT::Ptr& pc_candidates, const float scale, const int kmean_nb){
            selectCandidates(vec_out, pc_candidates, scenePoints_, sceneNormals_, scale, kmean_nb);
        }

        /**
         * @brief Extract a part of the scene corresponding to the acknowledged candidates.
         * 
         * @param vec_candidates Vector of the candidates results (considering that we will extract for all candidates in the vector -> it has already been reduced)
         * @param pc_candidates Point cloud of the candidates
         * @param pc_out Point cloud of the extracted zone
         * @param scale Scale of the candidates (used to define the zone to extract)
         */
        void extractCandidatesZone(const std::vector<ResultSurpriseT> vec_candidates, CloudT::Ptr pc_candidates,
                                   CloudT::Ptr &pc_points_out, CloudNormT::Ptr &pc_normals_out, const float scale,
                                   CloudT::Ptr &debug_not_selected);
        
        void formCandidatesVisualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out, const CloudT::Ptr pc_in, std::vector<ResultSurpriseT> vec_in); 
        
        /**
         * @brief 
         * 
         * @param window_name 
         * @param pc_candidates 
         * @param pc_global 
         * @param pc_zone 
         * @param pc_not_zone 
         * @return int the index of the created viewer in the vector
         */
        int visualize(const std::string window_name, 
                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_candidates, const CloudT::Ptr pc_global, 
                      const CloudT::Ptr pc_zone, const CloudT::Ptr pc_not_zone);

        /**
         * @brief Find the next scale in descending order.
         * 
         * @param prev the previous scale
         * @return float the next scale. (-1 if previous is smallest, -2 if previous not known)
         */
        float findNextScaleDes(float prev);
        
        /**
         * @brief Find the index of the scale in the scales vectors.
         * 
         * @param scale the scale to find
         * @return int the index (-1 if not in the vector.)
         */
        int findScaleIndex(const float scale);

        /**
         * @brief Check the existence of the given paths
         */
        void checkPathAndDir();

    public:
        /**
         * @brief Construct a new MFPFHLocalizer object
         */
        MFPFHLocalizer();
        /**
         * @brief Construct a new MFPFHLocalizer object
         * 
         * @param object name of the object
         */
        MFPFHLocalizer(std::string object);
        /**
         * @brief Construct a new MFPFHLocalizer object
         * 
         * @param object name of the object
         * @param path path to use as input and output
         */
        MFPFHLocalizer(std::string object, std::string path);
        /**
         * @brief Construct a new MFPFHLocalizer object
         * 
         * @param object name of the object
         * @param pathIn path to use as input
         * @param pathOut path to use as output
         */
        MFPFHLocalizer(std::string object, std::string pathIn, std::string pathOut);
        /**
         * @brief Construct a new MFPFHLocalizer object
         * 
         * @param object name of the object
         * @param pathIn path to use as input
         * @param pathOut path to use as output
         * @param fileScene direct path to the scene file
         */
        MFPFHLocalizer(std::string object, std::string pathIn, std::string pathOut, std::string fileScene);

        /**
         * @brief Destroy the MFPFHLocalizer object
         */
        ~MFPFHLocalizer();

        /**
         * @brief Run the pipeline on clusters localisation
         */
        void runClusters();
        /**
         * @brief Run the pipeline on clusters localisation with an initial guess
         * 
         * @param initialGuess the initial guess
         */
        void runClusters(Eigen::Isometry3d initialGuess);

        /**
         * @brief Run the pipeline on "surprise" localisation
         */
        void runSurprise(float scale_start, float scale_limit, int kmean_num);
        /**
         * @brief Run the pipeline on "surprise" localisation with an initial guess
         * 
         * @param initialGuess the initial guess
         */
        void runSurprise(float scale_start, float scale_limit, Eigen::Isometry3d initialGuess, int kmean_num);


        /**
         * @brief Run the pipeline on k-means localisation
         */
        void runkmeans();
        /**
         * @brief Run the pipeline on k-means localisation with an initial guess
         * 
         * @param initialGuess the initial guess
         */
        void runkmeans(Eigen::Isometry3d initialGuess);
    };

    /**
     * @brief All's in the name
     * 
     */
    void PressEnterToContinue();

    /**
     * @brief All's in the name bis.
     * 
     * @param question to be answered
     * @return true 
     * @return false 
     */
    bool askUserValidation(std::string question);

} // namespace multiscale_fpfh

#include "impl/localizer_tmpl.hpp"

#endif