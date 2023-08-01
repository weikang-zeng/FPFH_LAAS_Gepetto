#include "localizer.h"

namespace multiscale_fpfh
{
    bool MFPFHLocalizer::preProcessing(){
        float xMin(-5), xMax(0), yMin(-20), yMax(20), zMin(-10), zMax(10); //? all that's in front :)

        CloudT::Ptr sceneRecucedPC (new CloudT);
        object_plane_segmenter_.getReducedCloud(scenePoints_, sceneRecucedPC, xMin, xMax, yMin, yMax, zMin, zMax);

        object_plane_segmenter_.getSegmentedObjectsOnPlane(sceneRecucedPC, clustersPoints_, planePoints_);

        // pcl::visualization::PCLVisualizer viewer("Clustering results");
        // viewer.setBackgroundColor(0.6, 0.6, 0.6);
        // viewer.addCoordinateSystem(0.01);
        // viewer.addPointCloud(scenePoints_, "scene");
        // viewer.addPointCloud(sceneRecucedPC, "plane");
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0.2, "plane");

        // for(size_t indexCluster = 0; indexCluster < clustersPoints_.size(); indexCluster++){
        //     viewer.addPointCloud(clustersPoints_[indexCluster], "cluster"+std::to_string(indexCluster));
        //     float r, g, b;
        //     r = (((indexCluster>>2)&0x01)==0x01)?0.8:0.2;
        //     g = (((indexCluster>>1)&0x01)==0x01)?0.8:0.2;
        //     b = ((indexCluster&0x01)==0x01)?0.8:0.2;
        //     viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "cluster"+std::to_string(indexCluster));
        // }

        // while(!viewer.wasStopped())
        //     viewer.spinOnce();
        return true;
    }

    bool MFPFHLocalizer::generateClustersFeatures(){
        pcl::ScopeTime timer("Clusters' features' generation");

        clustersNormals_.resize(clustersPoints_.size());
        clustersFeatures_.resize(clustersPoints_.size());
        for (size_t indexCluster = 0; indexCluster < clustersPoints_.size(); indexCluster++){
            //* Compute normals
            clustersNormals_[indexCluster].reset(new CloudNormT);
            {
                pcl::ScopeTime timeNormal("Computing normals");
                pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>);
                normal_estimator_.setSearchMethod(kdTree);
                normal_estimator_.setRadiusSearch(0.025); //radius of 2.5cm -> search in a sphere of diameter 5cm
                normal_estimator_.setInputCloud(clustersPoints_[indexCluster]);
                normal_estimator_.compute(*(clustersNormals_[indexCluster]));
            }
            //* Compute FPFH scene
            clustersFeatures_[indexCluster].reset(new CloudSignT);
            {
                pcl::ScopeTime timeFeatures("Computing FPFH");
                fpfh_estimator_.setInputCloud(clustersPoints_[indexCluster]);
                fpfh_estimator_.setRadiusSearch(0.05); // set to 0.05 for now, will be modified later
                fpfh_estimator_.setInputNormals(clustersNormals_[indexCluster]);
                fpfh_estimator_.setSearchSurface(clustersPoints_[indexCluster]);
                fpfh_estimator_.compute(*(clustersFeatures_[indexCluster]));
            }
            std::cout<<"\e[1mStatistics of the Cluster ["<<indexCluster<<"]\e[0m"<<std::endl;
            std::cout<<"Cloud:\n"<<*(clustersPoints_[indexCluster])<<"Normals:\n"<<*(clustersNormals_[indexCluster])<<"Features:\n"<<*(clustersFeatures_[indexCluster])<<std::endl;
        }
        return true;
    }

    float MFPFHLocalizer::compare2Scene(CloudT::Ptr sourcePoints, CloudSignT::Ptr sourceFeatures, Eigen::Isometry3d &isoOut){
        pcl::ScopeTime timer("SAC-IA Alignment");
        CloudT::Ptr resultPoints (new CloudT);
        
        aligner_scia_.setMaximumIterations(1000); //1000
        aligner_scia_.setNumberOfSamples(15); //3
        aligner_scia_.setCorrespondenceRandomness(5); // number of best correspondences to random
        aligner_scia_.setMaxCorrespondenceDistance(0.005); //0.15 //0.005 //0.03 GT workfine
        // aligner_scia_.setMinSampleDistance(0.0001); //0.01 //0.001 //0.008 GT workfine 

        aligner_scia_.setInputSource(sourcePoints);
        aligner_scia_.setSourceFeatures(sourceFeatures);
        aligner_scia_.setInputTarget(scenePoints_);
        aligner_scia_.setTargetFeatures(sceneFeatures_);

        aligner_scia_.align(*resultPoints);
        isoOut = aligner_scia_.getFinalTransformation().cast<double>();

        return aligner_scia_.getFitnessScore(); //* source->target
    }

    float MFPFHLocalizer::compare2Cluster(CloudT::Ptr sourcePoints, CloudSignT::Ptr sourceFeatures, size_t indexCluster, Eigen::Isometry3d &isoOut){
        pcl::ScopeTime timer("SAC-IA Alignment");
        CloudT::Ptr resultPoints (new CloudT);
        
        //set match parameters
        aligner_scia_.setMaximumIterations(1000); //1000
        aligner_scia_.setNumberOfSamples(15); //3
        aligner_scia_.setCorrespondenceRandomness(5);
        aligner_scia_.setMaxCorrespondenceDistance(0.005); //0.15 //0.005 //0.03 GT workfine
        // aligner_scia_.setMinSampleDistance(0.0001); //0.01 //0.001 //0.008 GT workfine 

        aligner_scia_.setInputSource(sourcePoints);
        aligner_scia_.setSourceFeatures(sourceFeatures);
        aligner_scia_.setInputTarget(clustersPoints_[indexCluster]);
        aligner_scia_.setTargetFeatures(clustersFeatures_[indexCluster]);

        aligner_scia_.align(*resultPoints);
        isoOut = aligner_scia_.getFinalTransformation().cast<double>();

        return aligner_scia_.getFitnessScore(); //* source->target
    }

    void MFPFHLocalizer::runClusters(Eigen::Isometry3d initialGuess){
        initialGuess_ = initialGuess;
        runClusters();
    }

    void MFPFHLocalizer::runClusters(){
        pcl::ScopeTime timer("Complete pipeline");

        std::string logFilename = savePath_+"/log_"+execDateStr_+"_"+execHourStr_+".log";
        std::cout<<"Log file: "<<logFilename<<std::endl;
        std::ofstream fileOut(logFilename);
        if(!fileOut.is_open()){
            std::cerr<<"\e[1;30mLog file does not open. (<"<<logFilename<<">)\e[0m"<<std::endl;
            return;
        }

        extractScales();

        extractObject();
        fileOut<<"\e[1mStatistics of model \""<<objName_<<"\"\e[0m"<<std::endl;
        fileOut<<modelObject_<<std::endl;

        extractScene();
        fileOut<<"\e[1mStatistics of the scene \""<<sceneFile_.substr(sceneFile_.find_last_of('/'))<<"\"\e[0m"<<std::endl;
        fileOut<<"Cloud:\n"<<*scenePoints_<<"Normals:\n"<<*sceneNormals_<<"Features:\n"<<*sceneFeatures_<<std::endl;

        std::cout<<"\n\e[1mStarting preprocessing\e[0m\n\n";
        fileOut<<"\n\e[1mStarting preprocessing\e[0m\n\n";
        if(!preProcessing()){
            std::cerr<<"Problems occurred while preprocessing the scene. Contact the devs.\n"<<std::endl;
            exit(-1); //! don't like it but let's go!
        }
        std::cout<<"\n\e[1mPreprocessing done.\e[0m\n\n";
        fileOut<<"\n\e[1mPreprocessing done.\e[0m\n\n";

        //* Save plane and clusters
        boost::filesystem::path pathSave = savePath_+"/"+execDateStr_+"_"+execHourStr_+"_scene.pcd";
        if(!saveCloud(pathSave, *scenePoints_)){
            std::cout<<"Scene cloud was not saved"<<std::endl;
        }
        pathSave = savePath_+"/"+execDateStr_+"_"+execHourStr_+"_plane.pcd";
        if(!saveCloud(pathSave, *planePoints_)){
            std::cout<<"Plane cloud was not saved"<<std::endl;
        }
        for (size_t indexCluster = 0; indexCluster < clustersPoints_.size(); indexCluster++){
            pathSave = savePath_+"/"+execDateStr_+"_"+execHourStr_+"_cluster_"+std::to_string(indexCluster)+".pcd";
            if(!saveCloud(pathSave, *(clustersPoints_[indexCluster]))){
                std::cout<<"Cluster "<<indexCluster<<" was not saved"<<std::endl;
            }
        }
        generateClustersFeatures();


    }
    
} // namespace multiscale_fpfh