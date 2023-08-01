#include "objectsegmentationplane.h"

namespace multiscale_fpfh
{
    // template<typename PointT>
    // ObjectSegmentationPlane<PointT>::ObjectSegmentationPlane(){
    // }

    //function to do planar segmentation basing on pcl::SACSegmentation and return the indices of plane and coeff.
    template<typename PointT>
    bool ObjectSegmentationPlane<PointT>::getPlaneIndicesAndCoeffSAC(typename CloudT::Ptr p_cloudInput, 
                                                                     pcl::PointIndices::Ptr p_indices, 
                                                                     pcl::ModelCoefficients::Ptr p_modelCoeff){
        sacSegmenter_.setOptimizeCoefficients(true);
        sacSegmenter_.setInputCloud(p_cloudInput);
        sacSegmenter_.setModelType(pcl::SACMODEL_PLANE);   //三维平面
        sacSegmenter_.setMethodType(pcl::SAC_RANSAC);
        sacSegmenter_.setDistanceThreshold(0.03);
        sacSegmenter_.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
        sacSegmenter_.setEpsAngle(60.0f * (3.14/180.0f));
        sacSegmenter_.segment(*p_indices, *p_modelCoeff);

        if(p_modelCoeff->values.size() == 0){
            std::cout<<"\e[33m[PlaneSegmentation] No Plane Detected\e[0m"<<std::endl;
            return false;
        }
        return true;
    }

    //function to extract plane and non-plane cloud
    template<typename PointT>
    void ObjectSegmentationPlane<PointT>::getPlaneAndNonPlaneCloud(typename CloudT::Ptr p_cloudInput,
                                                                   pcl::PointIndices::Ptr p_indices,
                                                                   typename CloudT::Ptr p_cloudPlane,
                                                                   typename CloudT::Ptr p_cloudNotPlane){
        indicesExtractor_.setInputCloud(p_cloudInput);
        indicesExtractor_.setIndices(p_indices);

        //extract planar cloud
        indicesExtractor_.setNegative(false);
        indicesExtractor_.filter(*p_cloudPlane);

        //extract rest of cloud
        indicesExtractor_.setNegative(true);
        indicesExtractor_.filter(*p_cloudNotPlane);
    }

    //function to perform euclidean clustering and return cluster indices
    template<typename PointT>
    std::vector<pcl::PointIndices> ObjectSegmentationPlane<PointT>::getClusters(typename CloudT::Ptr p_cloudInput){
        pcl::ScopeTime timer("Clustering");
        //create a kdTree for Nearest Neighbour search
        typename pcl::search::KdTree<PointT>::Ptr kdTree (new pcl::search::KdTree<PointT>);
        kdTree->setInputCloud(p_cloudInput);

        std::vector<pcl::PointIndices> clusterIndices;
        euclideanClusterExtractor_.setInputCloud(p_cloudInput);
        euclideanClusterExtractor_.setSearchMethod(kdTree);
        euclideanClusterExtractor_.setClusterTolerance(0.1); //0.02 for brick //0.5 //less tolerance more clusters
        euclideanClusterExtractor_.setMinClusterSize(300);
        euclideanClusterExtractor_.setMaxClusterSize(1e5);
        euclideanClusterExtractor_.extract(clusterIndices);

        return clusterIndices;
    }

    //function to project inliers into plane
    template<typename PointT>
    void ObjectSegmentationPlane<PointT>::getProjectedCloud(typename CloudT::Ptr p_cloudInput,
                                                            pcl::PointIndices::Ptr p_indices,
                                                            pcl::ModelCoefficients::Ptr p_modelCoeff,
                                                            typename CloudT::Ptr& p_cloudOutput){
        p_cloudOutput.reset(new CloudT);

        inliersProjector_.setInputCloud(p_cloudInput);
        inliersProjector_.setModelType(pcl::SACMODEL_PLANE);
        inliersProjector_.setIndices(p_indices);
        inliersProjector_.setModelCoefficients(p_modelCoeff);
        inliersProjector_.filter(*p_cloudOutput);
    }

    //function to create convex hull
    template<typename PointT>
    // typename ObjectSegmentationPlane<PointT>::CloudT::Ptr ObjectSegmentationPlane<PointT>::getConvexHull(typename CloudT::Ptr p_cloudInput){
    void ObjectSegmentationPlane<PointT>::getConvexHull(typename CloudT::Ptr p_cloudInput, typename CloudT::Ptr& p_cloudOutput){
        p_cloudOutput.reset(new CloudT);
        pcl::ConvexHull<PointT> convexHull;
        convexHull.setInputCloud(p_cloudInput);
        convexHull.reconstruct(*p_cloudOutput);    //compute a convex hull for all points given
    }

    //function to take and point cloud and return vector of pointcloud clusters using ConvexHull
    template<typename PointT>
    bool ObjectSegmentationPlane<PointT>::getSegmentedObjectsOnPlane(typename CloudT::Ptr p_cloudInput,
                                                                     std::vector<typename CloudT::Ptr> &cloudClusterVector,
                                                                     typename CloudT::Ptr &p_cloudPlane){
        pcl::ScopeTime timer("Segmentation");
        /* ************************************************
        * 1. Get the point cloud filtered
        * 2. Get plane indices and Coefficients
        * 3. Project inliers on plane and get projected cloud
        * 4. Get convex Hull of projected cloud
        * 5. Get polygonal prism data of filtered cloud using convex hull, to get objects attached to plane
        * 6. Now, we have plane and associated objects, so again apply plane detection and get indices.
        * 7. extract indices which doesnot belong to plane to get the segmented objects on plane
        * 8. apply euclidean clustering to segment each object individually. */
        /* *************************************************/

        typename CloudT::Ptr cloudProjected;
        typename CloudT::Ptr cloudHull;
        typename CloudT::Ptr cloudNotPlane (new CloudT);
        p_cloudPlane.reset(new CloudT);

        pcl::ModelCoefficients::Ptr modelCoefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr indicesPoint (new pcl::PointIndices);

        //get plane indices and coeff
        bool isPlane = getPlaneIndicesAndCoeffSAC(p_cloudInput, indicesPoint, modelCoefficients);

        if(!isPlane){
            cloudClusterVector.push_back((typename CloudT::Ptr) new CloudT);
            pcl::copyPointCloud(*p_cloudInput, *cloudClusterVector.at(0)); // this line hide an unauthorized access if the previous line is not added
            return false;
        }

        // ****** Processes cloud which is not plane to only get object clusters present on plane *****
        //project inliers on plane and get projected cloud
        getProjectedCloud(p_cloudInput, indicesPoint, modelCoefficients, cloudProjected);

        //get the convex hull of projected cloud
        getConvexHull(cloudProjected, cloudHull);

        //get extreme four corners of hull
        std::vector<float> vectorX, vectorY, vectorZ;

        PointT minPt, maxPt;
        pcl::getMinMax3D(*cloudHull, minPt, maxPt);// output is the point value min and max value in the point cloud input data

        vectorX.push_back(minPt.x - 0.1); vectorY.push_back(minPt.y - 0.1);
        vectorX.push_back(minPt.x - 0.1); vectorY.push_back(maxPt.y + 0.1);
        vectorX.push_back(maxPt.x + 0.1); vectorY.push_back(maxPt.y + 0.1);
        vectorX.push_back(maxPt.x + 0.1); vectorY.push_back(minPt.y - 0.1);

        //calculate Z from plane equation
        float a = modelCoefficients->values.at(0);
        float b = modelCoefficients->values.at(1);
        float c = modelCoefficients->values.at(2);
        float d = modelCoefficients->values.at(3);
        for(int i=0; i < 4; ++i){
            float x = vectorX.at(i);
            float y = vectorY.at(i);
            float z = -((a*x) + (b*y) + d)/c;
            vectorZ.push_back(z);
        }
        typename CloudT::Ptr convexHullNew (new CloudT);
        convexHullNew->width = 4;
        convexHullNew->height = 1;
        convexHullNew->is_dense = false;
        convexHullNew->points.resize(4);
        for(int i = 0; i < 4 ; ++i){
            convexHullNew->points[i].x = vectorX.at(i);
            convexHullNew->points[i].y = vectorY.at(i);
            convexHullNew->points[i].z = vectorZ.at(i);
        }

        //get polygonal prism data
        pcl::PointIndices::Ptr cloudIndices (new pcl::PointIndices);
        pcl::ExtractPolygonalPrismData<PointT> extractPolyData;
        extractPolyData.setInputCloud(p_cloudInput);
        extractPolyData.setInputPlanarHull(convexHullNew);
        extractPolyData.segment(*cloudIndices);

        //form the cloud
        typename CloudT::Ptr cloudObjWithPlane (new CloudT);
        for(std::vector<int>::const_iterator pit = cloudIndices->indices.begin(); pit != cloudIndices->indices.end(); ++pit){
            cloudObjWithPlane->points.push_back(p_cloudInput->points[*pit]);
        }
        cloudObjWithPlane->width = cloudObjWithPlane->points.size();
        cloudObjWithPlane->height = 1;
        cloudObjWithPlane->is_dense = true;

        //get plane indices and coeff
        isPlane = getPlaneIndicesAndCoeffSAC(cloudObjWithPlane, indicesPoint, modelCoefficients);

        if(!isPlane){
            pcl::copyPointCloud(*p_cloudInput, *cloudClusterVector.at(0));
            return false;
        }

        //get plane and not plane cloud
        getPlaneAndNonPlaneCloud(cloudObjWithPlane, indicesPoint, p_cloudPlane, cloudNotPlane);
        {
            // get clusters for the not plane cloud
            std::vector<pcl::PointIndices> clusterIndices;
            clusterIndices = getClusters(cloudNotPlane);

            // save clusters
            for(std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
            {
                typename CloudT::Ptr cloudCluster (new CloudT);
                pcl::copyPointCloud(*cloudNotPlane, it->indices, *cloudCluster);
                // save to vector
                cloudClusterVector.push_back(cloudCluster);
            }
        }
        return true;
    }

    template<typename PointT>
    bool ObjectSegmentationPlane<PointT>::getReducedCloud(typename CloudT::Ptr p_cloudIn, typename CloudT::Ptr& p_cloudOut, float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
        //create temp clouds to store intermediate results
        typename CloudT::Ptr tempCloud (new CloudT);
        p_cloudOut.reset(new CloudT);

        //* check limits:
        if(xMin > xMax) std::swap<float>(xMin, xMax); //exchange two value
        if(yMin > yMax) std::swap<float>(yMin, yMax);
        if(zMin > zMax) std::swap<float>(zMin, zMax);

        //create pass through filter object
        pcl::PassThrough<PointT> passThrough;
        //filter along Z direction with limits specified
        passThrough.setInputCloud(p_cloudIn);
        passThrough.setFilterFieldName("z");
        passThrough.setFilterLimits(zMin, zMax);
        passThrough.filter(*p_cloudOut);
        std::cout<<*p_cloudIn<<std::endl<<*p_cloudOut<<std::endl<<std::endl;
        
        //filter along Y direction with limits specified
        passThrough.setInputCloud(p_cloudOut);
        passThrough.setFilterFieldName("y");
        passThrough.setFilterLimits(yMin, yMax);
        passThrough.filter(*tempCloud);
        std::cout<<*p_cloudOut<<std::endl<<*tempCloud<<std::endl<<std::endl;

        p_cloudOut.reset(new CloudT);
        //filter along X direction with limits specified
        passThrough.setInputCloud(tempCloud);
        passThrough.setFilterFieldName("x");
        passThrough.setFilterLimits(xMin, xMax);
        passThrough.filter(*p_cloudOut);
        std::cout<<*tempCloud<<std::endl<<*p_cloudOut<<std::endl<<std::endl;

        return true;
    }

    template<typename PointT>
    bool ObjectSegmentationPlane<PointT>::getReducedCloud(typename CloudT::Ptr p_cloudIn, typename CloudT::Ptr& p_cloudOut, std::vector<float> limits){
        if(limits.size() == 6){
            return getReducedCloud(p_cloudIn, p_cloudOut, limits[0], limits[1], limits[2], limits[3], limits[4], limits[5]);
        } else {
            std::cerr<<"ObjectSegmentationPlane - wrong limits given, size not corresponding (got "<<limits.size()<<", should be 6)"<<std::endl;
            return false;
        }
    }


    //* To remove linker errors (will be investigated more later)
    template class ObjectSegmentationPlane<pcl::PointXYZ>;


} // namespace multiscale_fpfh