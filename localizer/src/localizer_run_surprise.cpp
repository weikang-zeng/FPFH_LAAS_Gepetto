#include "localizer.h"

namespace multiscale_fpfh
{
    void MFPFHLocalizer::runSurprise(float scale_max, float scale_limit, Eigen::Isometry3d initialGuess, int kmean_num){
        initialGuess_ = initialGuess;
        runSurprise(scale_max, scale_limit, kmean_num);
    }

    float MFPFHLocalizer::findNextScaleDes(float prev){
        // Considering scalesObj_ (std::vector) sorted in descending order
        // Considering the scale is exactly in scalesObj_ (std::vector)
        float next;
        //? search the current scale
        auto pos = std::find(scalesObj_.begin(), scalesObj_.end(), prev);
        if(pos == scalesObj_.end())
            return -2; // The previous scale isn't in the vector
        if(pos == scalesObj_.end()-1)
            return -1; // The previous scale was the last
        return scalesObj_[pos - scalesObj_.begin()];
    }

    float MFPFHLocalizer::findClosestScale(float scale){
        // Considering scalesObj_ (std::vector) sorted in descending order
        //? search the first lower value
        size_t pos = 0;
        while((scalesObj_[pos] > scale)&&(pos < scalesObj_.size()))
            pos ++;
        //? Closest is the last, because @a scale is lesser than all the values.
        if(pos >= scalesObj_.size())
            return scalesObj_.back();
        //? Closest is the first, because @a scale is greater than all the values.
        if(pos == 0)
            return scalesObj_.front();
        //? @a scale is between two values 
        if(scale-scalesObj_[pos] < scalesObj_[pos-1]-scale)
            return scalesObj_[pos];
        else
            return scalesObj_[pos-1];
    }

    int MFPFHLocalizer::findScaleIndex(const float scale){
        for(size_t indexScale = 0; indexScale < modelObject_[0].scales.size(); indexScale++)
            if(modelObject_[0].scales[indexScale].scale == scale)
                return indexScale;
        return -1;
    }


    void MFPFHLocalizer::selectCandidates(std::vector<ResultSurpriseT> &vec_out, CloudT::Ptr &pc_candidates, 
                                          const CloudT::Ptr pc_source, const CloudNormT::Ptr pc_normals, const float scale ,const int kmean_nb){
        vec_out.resize(0);    //resize container vec_out to 0
        int scale_index = findScaleIndex(scale);   
        if(scale_index == -1){
            std::cerr<<"Error while looking for scale "<<scale<<std::endl;
            return;
        }

        std::cout<<"\n\e[1mStarting preprocessing\e[0m\n\n";
        logFile_<<"\n\e[1mStarting preprocessing\e[0m\n\n";

        std::cout<<"\e[1mComputing some scene features at scale "<<scale<<"\e[0m\n"<<std::endl;


        std::cout<< "PointCloud before filtering has :" << pc_source->size() <<"data points."<<std::endl;     //before filtering, check

        //* create the filtering objet:downsample the dataset using a leaf size of scale/2
        pc_candidates.reset(new CloudT);
        {
            pcl::ScopeTime timeFeatures("Computing voxelisation");
            pcl::VoxelGrid<PointT> voxel_grid;                     
            voxel_grid.setInputCloud(pc_source);
            voxel_grid.setLeafSize(scale/2., scale/2., scale/2.);
            voxel_grid.filter(*pc_candidates);
            std::cout<< "PointCloud after filtering has :" << pc_candidates->size() <<"data points."<<std::endl;   //check result filtering
        }
        saveCloud(savePath_+"/scene/candidates_voxel_s"+std::to_string(scale)+".pcd", *pc_candidates);


        //* Compute FPFH scene
        CloudSignT::Ptr features_scene_scale(new CloudSignT);
        {
            pcl::ScopeTime timeFeatures("Computing FPFH");
            //pcl::io::loadPCDFile<pcl::FPFHSignature33> ("/home/wzeng/multiscale-fpfh/localizer/build/fpfhs/test.pcd",*features_scene_scale);
                        /*std::cout<<"no data so compute and save"<<std::endl;*/
                        fpfh_estimator_.setInputCloud(pc_candidates);
                        fpfh_estimator_.setRadiusSearch(scale);
                        fpfh_estimator_.setInputNormals(pc_normals);
                        fpfh_estimator_.setSearchSurface(pc_source);
                        fpfh_estimator_.compute(*features_scene_scale);

        }
        saveCloud(savePath_+"/scene/features_candidates_voxel_s"+std::to_string(scale)+".pcd", *features_scene_scale);
        std::cout<<"\nVoxelized cloud:\n"<<*pc_candidates<<"\nVoxelized features:\n"<<*features_scene_scale<<std::endl;
        logFile_<<"\nVoxelized cloud:\n"<<*pc_candidates<<"\nVoxelized features:\n"<<*features_scene_scale<<std::endl;


        
        //* Compute k-means
        int k = kmean_nb;              //number of kmean
        int kt =0;
        //std::cout<<k<<std::endl;
        CloudSignT::Ptr features_clustered(new CloudSignT); //
        //* This version gives at most k*nb_views descriptors to look at that can be close.
        //? Should it be reviewed to give less? (by reducing the closest groups)
        //std::cout<<"check the scale before"<<scale<<std::endl;

        //*for the part all in one scale 1.33
        if (scale>1.33){
            std::cout<<"we have features all in one "<<std::endl;
            int index=0;
            while((scale !=Allinone_[index].scale) && (index < Allinone_.size()))
                    index++;
                    std::cout<<"index is :"<<index <<std::endl;
                
            features_clustered=Allinone_[index].features;
            
        }

        //*for the part all in one scale 1.33
        /*else if (scale>0.66){
            std::cout<<"we have features all in one "<<std::endl;
            features_clustered=Allinone_[1].features;
        }*/

        else{
            std::cout<<"check the scale "<<scale<<std::endl;
            pcl::ScopeTime timer("Clusterization");
            //* Clusterize all models
            for(size_t indexModel = 0; indexModel < modelObject_.size(); indexModel++){
                //empty constructor
                pcl::Kmeans kmeans(static_cast<int>(modelObject_[indexModel].scales[scale_index].features->points.size()), 33);
                
                k =modelObject_[indexModel].scales[scale_index].k_compo;   //for value idetif k
                
                //std::cout<<modelObject_[indexModel].partName<<std::endl;   //for check
                //kt=kt+k;                                                   
                //std::cout<<kt<<std::endl;

                //Buiid a frame that fits the siez of the point cloud
                //cast data type to int
                //Kmeans(unsigned int num_points, unsigned int num_dimensions)
                kmeans.setClusterSize(k);     //sets the cluster size, h=number of cluster

                // add points to the clustering
                for(size_t indexPoint = 0; indexPoint < modelObject_[indexModel].scales[scale_index].features->points.size(); indexPoint++){
                    std::vector<float> data (33);
                    for(int idx = 0; idx < 33; idx++)
                        data[idx] = modelObject_[indexModel].scales[scale_index].features->points[indexPoint].histogram[idx];
                    kmeans.addDataPoint(data);
                }

                // k-means clustering
                kmeans.kMeans();

                // get the cluster centroids
                pcl::Kmeans::Centroids centroids = kmeans.get_centroids();

                // initialize output cloud
                features_clustered->width += static_cast<int>(centroids.size());
                features_clustered->height = 1;
                features_clustered->is_dense = false;
                // copy cluster centroids into feature cloud 
                for(size_t i = 0; i < centroids.size(); i++){
                    pcl::FPFHSignature33 feature;
                    for(int idx = 0; idx < 33; idx++)
                        feature.histogram[idx] = (std::isfinite(centroids[i][idx])?centroids[i][idx]:OUT_OF_COMPUTATION_FPFH);
                        //std::isfinite used to check if a given value is finite
                    //features_clustered->points.push_back(feature);
                    if(!std::any_of(std::begin(feature.histogram), std::end(feature.histogram), [](float x){return x == OUT_OF_COMPUTATION_FPFH;})){
                        features_clustered->points.push_back(feature);
                        //std::cout<<feature<<std::endl;
                    }
                    //std::cout<<features_clustered->points.back()<<std::endl;
                    else{
                            features_clustered->width --;
                    }
                }
                
            }
            saveCloud(savePath_+"/clusters/features_clustered_scale_"+std::to_string(scale)+".pcd",*features_clustered);
        }          //for else

        pcl::search::KdTree<SignatureT> features_tree;            //Creating the KdTree object for the search method (of the extraction)
        features_tree.setInputCloud(features_clustered);
        pcl::search::KdTree<PointT> point_tree;                   //Creating the KdTree object on the scene, to look for the real index
        point_tree.setInputCloud(scenePoints_);

        vec_out.resize(features_scene_scale->size());           //configure size accroding the FPFH scene 

        logFile_<<"\n\n## Results\n";
        {
            pcl::ScopeTime timer("Correspondences");
#pragma omp parallel for schedule(static)
            for (size_t index_scene = 0; index_scene < features_scene_scale->size(); index_scene++){
                std::vector<int> k_indices;
                std::vector<float> k_sqr_distances;
                //float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

                features_tree.nearestKSearch(features_scene_scale->points[index_scene], 10, k_indices, k_sqr_distances);  //10 default
                //features_tree.radiusSearch(features_scene_scale->points[index_scene], radius, k_indices, k_sqr_distances); 
                /*features_scene_scale->points[index_scene] is base point for search, 10: Take 10 points
                nearestKSearch (const PointCloud &cloud, int index, int k, std::vector<int> &k_indices, std::vector<float> &k_sqr_distances)
                features_scene_scale->points[index_scene] based on this incoming point, the reference point
                int k : take how many points closest to the base point
                k_indices: the subscripts of these K pointss
                k_sqr_distances : the distances of these K points to the reference point
                */
                vec_out[index_scene].indexReducedScene = index_scene;
                vec_out[index_scene].indexkClusters = k_indices[0];
                vec_out[index_scene].distance = k_sqr_distances[0];
                
                //**look for the index point in the real scene
                point_tree.nearestKSearch(pc_candidates->points[index_scene], 1, k_indices, k_sqr_distances);
                vec_out[index_scene].indexCompleteScene = k_indices[0];
                
                //std::cout<<"index"<<index_scene<<std::endl;
                /*the search results are stored in three arrays, one is the index of data set scene,one is the subscript and the other is the distance*/ 
            }
        }

        {
            pcl::ScopeTime timer("Writing");
            std::sort(vec_out.begin(), vec_out.end());
            for(size_t index = 0; index < vec_out.size(); index ++){
                logFile_<<"For point #"<<vec_out[index].indexCompleteScene
                        <<"(candidate #"<<vec_out[index].indexReducedScene
                        <<"): indice of features in k-means: "<<vec_out[index].indexkClusters
                        <<"; square distance: "<<vec_out[index].distance;
                logFile_<<" | Point feature is: [";
                for(size_t idx = 0; idx < 33; idx++)
                    logFile_<<features_scene_scale->points[vec_out[index].indexReducedScene].histogram[idx]<<((idx < 32)?", ":"]");
                logFile_<<std::endl;
            }
        }
    }

    void MFPFHLocalizer::extractCandidatesZone(const std::vector<ResultSurpriseT> vec_candidates, CloudT::Ptr pc_candidates,
                                               CloudT::Ptr &pc_points_out, CloudNormT::Ptr &pc_normals_out, const float scale,
                                               CloudT::Ptr &debug_not_selected){
        CloudT::Ptr scene_copy(new CloudT);
        pcl::copyPointCloud(*scenePoints_, *scene_copy);
        CloudNormT::Ptr scene_normals_copy(new CloudNormT);
        pcl::copyPointCloud(*sceneNormals_, *scene_normals_copy);

        pc_points_out.reset(new CloudT);
        pc_normals_out.reset(new CloudNormT);
        debug_not_selected.reset(new CloudT);
        CloudT::Ptr temp_out(new CloudT), temp_scene(new CloudT);
        CloudNormT::Ptr temp_normals_out(new CloudNormT), temp_scene_normals(new CloudNormT);

        pcl::search::KdTree<PointT> search_tree;
        pcl::ExtractIndices<PointT> extractor_points;
        pcl::ExtractIndices<NormalT> extractor_normals;
        search_tree.setInputCloud(scene_copy);
        extractor_points.setInputCloud(scene_copy);
        extractor_normals.setInputCloud(scene_normals_copy);
        pcl::IndicesPtr bp_k_indices(new pcl::Indices);
        // std::vector<int> k_indices;
        std::vector<float> k_sqr_distances;
        // bp_k_indices.reset(&k_indices);
        extractor_points.setIndices(bp_k_indices);
        extractor_normals.setIndices(bp_k_indices);
        for(size_t index_candidates = 0; index_candidates < vec_candidates.size(); index_candidates++){
            bp_k_indices->clear();
            k_sqr_distances.clear();
            //* Search from scene
            search_tree.setInputCloud(scene_copy);
            search_tree.radiusSearch(pc_candidates->points[vec_candidates[index_candidates].indexReducedScene], scale,
                                     *bp_k_indices, k_sqr_distances);

            //* extract from scene
            extractor_points.setNegative(false);
            extractor_normals.setNegative(false);
            extractor_points.filter(*temp_out);
            extractor_normals.filter(*temp_normals_out);
            *pc_points_out += *temp_out;
            *pc_normals_out += *temp_normals_out;

            //* To reduce computation and avoid double points: remove actuals neighborhoods from scene
            extractor_points.setNegative(true);
            extractor_normals.setNegative(true);
            extractor_points.filter(*temp_scene);
            extractor_normals.filter(*temp_scene_normals);
            pcl::copyPointCloud(*temp_scene, *scene_copy);
            pcl::copyPointCloud(*temp_scene_normals, *scene_normals_copy);
        }
        pcl::copyPointCloud(*scene_copy, *debug_not_selected);
    }

    void MFPFHLocalizer::formCandidatesVisualization(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out, const CloudT::Ptr pc_in, std::vector<ResultSurpriseT> vec_in){
        pc_out.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (size_t indexResult = 0; indexResult < vec_in.size(); indexResult++){
            float r, g, b;
            if(indexResult <= (int)floor(vec_in.size()*0.10)){
                r = 0.; g = 1.0; b = 0.0;
            } else if(indexResult <= (int)floor(vec_in.size()*0.20)){
                r = 0.0; g = 1.0; b = 0.5;
            } else if(indexResult <= (int)floor(vec_in.size()*0.30)){
                r = 0.; g = 1.0; b = 1.0;
            } else if(indexResult <= (int)floor(vec_in.size()*0.40)){
                r = 0.; g = 0.5; b = 1.0;
            } else if(indexResult <= (int)floor(vec_in.size()*0.50)){
                r = 0.; g = 0.; b = 1.0;
            } else if(indexResult <= (int)floor(vec_in.size()*0.60)){
                r = 0.5; g = 0.; b = 1.0;
            } else if(indexResult <= (int)floor(vec_in.size()*0.70)){
                r = 1.0; g = 0.; b = 1.0;
            } else if(indexResult <= (int)floor(vec_in.size()*0.80)){
                r = 1.0; g = 0.; b = 0.5;
            } else {
                r = 1.0; g = 0.0; b = 0.0;
            }
            pcl::PointXYZRGB newpoint;
            newpoint.x = pc_in->points[vec_in[indexResult].indexReducedScene].x;
            newpoint.y = pc_in->points[vec_in[indexResult].indexReducedScene].y;
            newpoint.z = pc_in->points[vec_in[indexResult].indexReducedScene].z;
            newpoint.r = r*255; newpoint.g = g*255; newpoint.b = b*255;
            pc_out->push_back(newpoint);
        }
    }

    int MFPFHLocalizer::visualize(const std::string window_name, 
                                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_candidates, const CloudT::Ptr pc_global, 
                                  const CloudT::Ptr pc_zone, const CloudT::Ptr pc_not_zone){
        int vpCandidates, vpZone;
        viewers_.push_back((pcl::visualization::PCLVisualizer::Ptr)new pcl::visualization::PCLVisualizer(window_name));
        viewers_.back()->setBackgroundColor(0.1, 0.1, 0.1);
        viewers_.back()->addCoordinateSystem(0.01);
        viewers_.back()->createViewPort(0.0, 0.0, 0.5, 1.0, vpCandidates);
        viewers_.back()->createViewPort(0.5, 0.0, 1.0, 1.0, vpZone);
        viewers_.back()->addPointCloud(pc_candidates, "candidates", vpCandidates);
        viewers_.back()->updatePointCloud(pc_candidates, "candidates");
        viewers_.back()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "candidates");
        viewers_.back()->addPointCloud(scenePoints_, "sceneC", vpCandidates);
        viewers_.back()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.6, 0.6, 0.6, "sceneC");
        viewers_.back()->addPointCloud(pc_zone, "Zone", vpZone);
        viewers_.back()->addPointCloud(pc_not_zone, "debugZone", vpZone);
        viewers_.back()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Zone");
        viewers_.back()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0., 0., 1., "Zone");
        viewers_.back()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "debugZone");
        viewers_.back()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0., 0., "debugZone");
        viewers_.back()->addPointCloud(scenePoints_, "sceneZ", vpZone);
        viewers_.back()->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.6, 0.6, 0.6, "sceneZ");

        return viewers_.size()-1;
    }

    void MFPFHLocalizer::runSurprise(float scale_max, float scale_limit, int kmean_num){
        pcl::ScopeTime timer("Complete pipeline");    //statistics runtime 
        std::cout<<"Check: "<<std::isnan(NAN)<<" (if this value is not 1, please call a dev.)"<<std::endl; //! If giving 0 -> not correct -> weird behavior can happen
        //std::isnan(NAN whether it is a numerical value
        extractScales();
        //extract_value_k();

        if(!extractObject())
            return;
        logFile_<<"\e[1mStatistics of model \""<<objName_<<"\"\e[0m"<<std::endl;
        logFile_<<modelObject_<<std::endl;       //To save the information and data in the process, by use the method of writing a file log
        
        

        extractScene();
        logFile_<<"\e[1mStatistics of the scene \""<<sceneFile_.substr(sceneFile_.find_last_of('/'))<<"\"\e[0m"<<std::endl;
        logFile_<<"\nCloud:"<<*scenePoints_<<"\nNormals:"<<*sceneNormals_<<"\nFeatures:"<<*sceneFeatures_<<std::endl;

        CloudT::Ptr points_scene_scale;
        std::vector<ResultSurpriseT> vec_features_diff;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_visu;
        CloudT::Ptr pc_zone, debug_not_selected;
        CloudNormT::Ptr pc_zone_normals;
        int viewerScale;
        int kmean_nb;
        float scale2use;
        // init computation
        kmean_nb = kmean_num; 
        scale2use = findClosestScale(scale_max);
        
        pc_zone.reset(new CloudT);
        pcl::copyPointCloud(*scenePoints_, *pc_zone);
        pc_zone_normals.reset(new CloudNormT);
        pcl::copyPointCloud(*sceneNormals_, *pc_zone_normals);

        //for save which one should be keep
        std::vector<int> indice;




        
            do {
                points_scene_scale.reset(new CloudT);
                vec_features_diff.clear();
                
                    std::cout<<"kmean_num is:\n"<<kmean_nb<<std::endl;
                    std::cout<<"scale input is:\n"<<scale2use<<std::endl;

                    selectCandidates(vec_features_diff, points_scene_scale, pc_zone, pc_zone_normals, scale2use, kmean_nb);


                    std::sort(vec_features_diff.begin(), vec_features_diff.end());
                    formCandidatesVisualization(pc_visu, points_scene_scale, vec_features_diff);
                    vec_features_diff.erase(vec_features_diff.begin()+(int)floor(vec_features_diff.size()*0.15), vec_features_diff.end());
                    
                    //*This section is designed to remove irrelevant perspective models after each scale process
                    /*if (scale2use>1.33){
                        std::cout<<"please dont do anything"<<std::endl;
                    }
                    else{ 
                    // for delete some view we don't need it
                    //start with 15% of results,which provides info for indexkClusters
                    for(size_t index = 0; index < vec_features_diff.size(); index ++){
                        //std::cout<<"result need to be comparer ="<<vec_features_diff[index].indexkClusters<<std::endl;
                        int k_2=0;
                        //Read the k values in the model and accumulate their values
                        for(size_t indexModel = 0; indexModel < modelObject_.size(); indexModel++){
                            int scale_index = findScaleIndex(scale2use);   
                            int k_1 =modelObject_[indexModel].scales[scale_index].k_compo;   //for value idetif k
                            //std::cout<<modelObject_[indexModel].partName<<std::endl;
                            k_2=k_2+k_1;
                            //compare with the value of indexkClusters to know the corresponding view index,and save this indexModel,out of the loop
                            //std::cout<<"k result is ="<<k_2<<std::endl;
                            if(vec_features_diff[index].indexkClusters<k_2){
                                indice.push_back(indexModel);
                                //std::cout<<"result index need to be keep  "<<indexModel<<std::endl;
                                indexModel=modelObject_.size();
                            }
                                //modelObject_.erase()
                            }
                    }
                    //sort the results and remove duplicates
                    std::sort(indice.begin(),indice.end());
                    auto last =std::unique(indice.begin(),indice.end());
                    indice.erase(last,indice.end());
                    //Extract the scene that needs to be retained into a newvector according to the index, and then release the memory of the old vector
                    for(size_t i= 0; i < indice.size(); i++){
                            swap_modelObject_.push_back(modelObject_[indice[i]]);
                    }
                    modelObject_.clear();
                    modelObject_.shrink_to_fit();    
                    //exchange each other
                    std::swap(swap_modelObject_,modelObject_);
                    std::cout<<modelObject_.size()<<std::endl;
                    //for check
                    for(size_t indexModel = 0; indexModel < modelObject_.size(); indexModel++){
                            std::cout<<modelObject_[indexModel].partName<<std::endl;
                        }
                    }

                    //for check the vector
                    //for(size_t i= 0; i < indice.size(); i++){
                        //std::cout<<indice[i]<<std::endl;
                    //}*/
                            
                    
                            

                    extractCandidatesZone(vec_features_diff, points_scene_scale, pc_zone, pc_zone_normals, scale2use, debug_not_selected);
                    
                    viewerScale = visualize("Candidates & Zone: scale "+std::to_string(scale2use), pc_visu, scenePoints_, pc_zone, debug_not_selected);
                    while (!viewers_[viewerScale]->wasStopped())
                        viewers_[viewerScale]->spinOnce();
                
                //* Let's go for a new scale !
                scale2use = findClosestScale(scale2use/2);
            } while (scale2use > scale_limit);
                                                        
        return;
        
    }
    
} // namespace multiscale_fpfh