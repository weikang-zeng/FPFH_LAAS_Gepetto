#include "localizer.h"

namespace multiscale_fpfh
{
    
    float MFPFHLocalizer::extractScaleFromFilename(std::string filename){
        // the filename should contain : features_sXXXXXX_YYYYYYY.pcd 
        // where: {X} is the scale
        //    and {Y} is the points filename

        std::size_t pos;
        float scale = 0.0;
        std::string scaleStr;
        if((pos = filename.find("features_s")) != std::string::npos){
            scaleStr = filename.substr(pos+10, filename.find("_")-pos-10);
            scale = std::atof(scaleStr.c_str());
        }
        return scale;
    }

    bool MFPFHLocalizer::extractScales(){
        boost::filesystem::directory_iterator end_files;     //Iterate over all files in a directory
        float actualScale = 0.0;     //Initialize the scale value

        //* For first object
        boost::filesystem::directory_iterator iter_files_1(inputPath_+"/"+objName_+"/features");
        //? Iterations over files
        while(iter_files_1 != end_files){                //stop until Iterations to last file
            boost::filesystem::path next_file(iter_files_1->path()); //get file path
            
            actualScale = extractScaleFromFilename(next_file.filename().string());    //extract the scale number in the name

            //! scale at 0: could not be extracted.
            if(actualScale == 0.0){
                std::cerr<<"\e[1;30mCould not extract any scale from <"<<next_file.string()<<">\e[0m"<<std::endl;
                return false;
            }
            //* Check if it's an already known scale, if not: add it
            bool found = false;
            for (size_t indexScale = 0; indexScale < scalesObj_.size(); indexScale++){
                if(actualScale == scalesObj_[indexScale])
                    found = true;
            }
            if(!found)
                scalesObj_.push_back(actualScale);
                


            iter_files_1++;
        }
        std::sort(scalesObj_.begin(), scalesObj_.end(), std::greater<float>());
        
        return true;
    }

    //* This part extracts reasonable clustering index values from the file names 
    //to provide k values for the next function
    //In order to use this part of the functionality 
    //you need to pre-process the model set using mcnd.py（Model clustering number detection） in the python folder 
    
    float MFPFHLocalizer::extractKvalueFromFilename(std::string filename){
        // the filename should contain : features_sXXXXXX_YYYYYYY.pcd 
        // where: {X} is the scale
        //    and {Y} is the points filename

        std::size_t pos;
        float vk = 0.0;
        std::string vkStr;
        if((pos = filename.find(".pcd")) != std::string::npos){
            vkStr = filename.substr(pos-2,2);
            vk = std::atof(vkStr.c_str());
        }
        return vk;
    }


    //*This part will extract the k values and detect the scale model corresponding to them. 
    //If there are no errors, the corresponding k-values are assigned to the model vector
    bool MFPFHLocalizer::extract_value_k(std::vector<ModelCompleteT>& vectorOut, std::string folderObject){
        pcl::ScopeTime timer("k_value extraction");
        boost::filesystem::directory_iterator end_files;     //Iterate over all files in a directory
        //* For first object
        boost::filesystem::directory_iterator iter_files_1(inputPath_+"/"+objName_+"/features_test");
        float actualK = 0.0;     //Initialize the scale value
        float scale = 0.0;     //Initialize the scale value
        //ModelFeaturesT new_k;
        std::size_t pos1, pos2, indexModel,indexScale;
        //? Iterations over files
        while(iter_files_1 != end_files){                //stop until Iterations to last file
            boost::filesystem::path next_file(iter_files_1->path()); //get file path
            actualK = extractKvalueFromFilename(next_file.filename().string());    //extract the k number in the name
            //! scale at 0: could not be extracted.
            if(actualK == 0.0){
                std::cerr<<"\e[1;30mCould not extract any K value from <"<<next_file.string()<<">\e[0m"<<std::endl;
                return false;
            }

            std::string filename = iter_files_1->path().filename().string();
            std::string cloudname;

            
            //* check if features set
            if((pos1 = filename.find("features_s")) == std::string::npos){
                std::cerr<<"\e[1;30mError(s) occurred while extracting <"<<filename<<">.\e[0m"<<std::endl;
                std::cerr<<"\e[1;30mDoes not looks like a features' cloud (name should be \"features_s\%f_\%s\", \%f being the scale, \%s being the name of the corresponding points cloud).\e[0m"<<std::endl;
                continue;
                // return false;
            }
            
            //* get model name
            cloudname = filename.substr(pos1+10);
            pos2 = filename.find("_");
            cloudname = cloudname.substr(pos2+1);
            cloudname.erase(cloudname.find_last_of('k')-1, cloudname.back());

            //* get scale value
            scale = extractScaleFromFilename(filename);
            if(scale == 0.0){ //* if no scale given (or null scale which should not be) -> error
                std::cerr<<"\e[1;30mCould not extract any scale from <"<<filename<<">\e[0m"<<std::endl;
                continue;
                // return false;
            }

            //* find corresponding point cloud in the model vector
            indexModel = 0;
            while((cloudname.compare(vectorOut[indexModel].partName) != 0) && (indexModel < vectorOut.size())) 
                indexModel++;
            if(indexModel == vectorOut.size()){
                std::cerr<<"\e[1;30mCould not find <"<<filename<<"> in the given points clouds\e[0m"<<std::endl;
                continue;
                // return false;
            }


            //* find scale in the model
            indexScale = 0;
            while((scale != vectorOut[indexModel].scales[indexScale].scale) && (indexScale < vectorOut[indexModel].scales.size()))
                indexScale++;

            if(indexScale == vectorOut[indexModel].scales.size()){
                std::cerr<<"\e[1;30mCould not find scale <"<<scale<<"> in the set. (certainly an error occurred in the extractions, please, call the devs)\e[0m"<<std::endl;
                continue;
                // return false;
            }

            //return value for vector
            vectorOut[indexModel].scales[indexScale].k_compo = actualK;
                    
            iter_files_1++;
        }
        return true;
    }

    bool MFPFHLocalizer::extractClouds(std::vector<ModelCompleteT>& vectorOut, std::string folderObject){
        pcl::ScopeTime timer("Clouds extraction");
        std::cout<<"Extracting clouds in <"<<folderObject<<"/points>"<<std::endl; // just for info because it can be long
        char countDebug=0;
        boost::filesystem::directory_iterator end_files;
        boost::filesystem::directory_iterator iter_files_points(folderObject+"/points");
        //? Iterations over files
        while(iter_files_points != end_files){
            std::cout<<(((countDebug%4)==0)?"\r      \r":"--")<<std::flush; // just for info because it can be long
            countDebug = (countDebug+1)%4;

            ModelCompleteT new_model;
            new_model.partName = iter_files_points->path().filename().string().substr(0UL, iter_files_points->path().filename().string().size()-4);
            new_model.points = (CloudT::Ptr) new CloudT;
            if(!loadCloud(iter_files_points->path(), *new_model.points)){
                std::cerr<<"\e[1;30mError(s) occurred while extracting <"<<iter_files_points->path().filename().string()<<">.\e[0m"<<std::endl;
                return false;
            }
            vectorOut.push_back(new_model);
            iter_files_points++;
        }
        std::cout<<std::endl; // just for info because it can be long
        return true;
    }

//*This section extracts the results after clustering all the FPFH descriptors of a certain scale model. 
    //to provide sample ALL IN ONE for the next function
    //In order to use this part of the functionality 
    //you need to pre-process the model set using faio.py（FOR ALL IN ONE） in the python folder 
bool MFPFHLocalizer::extractFeatures_allinone(std::vector<ModelFeaturesT>& vectorOut, std::string folderObject, std::vector<float> vectorScales){
        pcl::ScopeTime timer("One Features extraction");
        bool result = true;
        std::cout<<"Extracting features in <"<<folderObject<<"/features_AllInOne>"<<std::endl; // just for info because it can be long
        char countDebug=0;
        boost::filesystem::directory_iterator iter_files_features(folderObject+"/features_AllInOne");
        boost::filesystem::directory_iterator end_files;
        std::size_t indexScale;
        float scale = 0.0;
        //std::string scaleStr;
        //std::vector<std::string> file_list;


        while(iter_files_features != end_files){
            std::cout<<(((countDebug%4)==0)?"\r      \r":"--")<<std::flush; // just for info because it can be long
            countDebug = (countDebug+1)%4;
            
            std::string filename = iter_files_features->path().filename().string();
            //std::string cloudname;
            
            //* get scale value
            scale = extractScaleFromFilename(filename);
            if(scale == 0.0){ //* if no scale given (or null scale which should not be) -> error
                std::cerr<<"\e[1;30mCould not extract any scale from <"<<filename<<">\e[0m"<<std::endl;
                continue;
                // return false;
            }

            indexScale = 0;
            while((scale != vectorScales[indexScale]) && (indexScale < vectorScales.size()))
                indexScale++;
            if(indexScale == vectorScales.size()){
                std::cerr<<"\e[1;30mCould not find scale <"<<scale<<"> in the set. (certainly an error occurred in the extractions, please, call the devs)\e[0m"<<std::endl;
                continue;
                // return false;
            }

            ModelFeaturesT new_data;
            new_data.features.reset(new CloudSignT);
            new_data.scale = vectorScales[indexScale];
            std::cout<<new_data.scale<<std::endl;
            //* extract the features and save them    
            if(!loadCloud(iter_files_features->path(), *new_data.features)){
                std::cerr<<"\e[1;30mError(s) occurred while extracting <"<<iter_files_features->path().filename().string()<<">.\e[0m"<<std::endl;
                continue;
                // return false;
            }
            vectorOut.push_back(new_data);
            //vectorOut[indexScale].features
            iter_files_features++;
            
        }

        std::cout<<std::endl; // just for info because it can be long
        return true;
    }




    bool MFPFHLocalizer::extractFeatures(std::vector<ModelCompleteT>& vectorOut, std::string folderObject){
        pcl::ScopeTime timer("Features extraction");
        bool result = true;
        std::cout<<"Extracting features in <"<<folderObject<<"/features>"<<std::endl; // just for info because it can be long
        char countDebug=0;
        boost::filesystem::directory_iterator iter_files_features(folderObject+"/features");
        boost::filesystem::directory_iterator end_files;
        std::size_t pos1, pos2, indexModel, indexScale;
        float scale = 0.0;
        std::string scaleStr;
        std::vector<std::string> file_list;
        while(iter_files_features != end_files){
            std::cout<<(((countDebug%4)==0)?"\r      \r":"--")<<std::flush; // just for info because it can be long
            countDebug = (countDebug+1)%4;
            
            std::string filename = iter_files_features->path().filename().string();
            std::string cloudname;
            scale = 0.0;

            
            //* check if features set
            if((pos1 = filename.find("features_s")) == std::string::npos){
                std::cerr<<"\e[1;30mError(s) occurred while extracting <"<<filename<<">.\e[0m"<<std::endl;
                std::cerr<<"\e[1;30mDoes not looks like a features' cloud (name should be \"features_s\%f_\%s\", \%f being the scale, \%s being the name of the corresponding points cloud).\e[0m"<<std::endl;
                continue;
                // return false;
            }
            
            //* get model name
            cloudname = filename.substr(pos1+10);

            pos2 = filename.find("_");

            cloudname = cloudname.substr(pos2+1);

            cloudname.erase(cloudname.find_last_of('.'), cloudname.back());


            //* get scale value
            scale = extractScaleFromFilename(filename);
            if(scale == 0.0){ //* if no scale given (or null scale which should not be) -> error
                std::cerr<<"\e[1;30mCould not extract any scale from <"<<filename<<">\e[0m"<<std::endl;
                continue;
                // return false;
            }
            
            //* find corresponding point cloud in the model vector
            indexModel = 0;
            while((cloudname.compare(vectorOut[indexModel].partName) != 0) && (indexModel < vectorOut.size())) 
                indexModel++;
            if(indexModel == vectorOut.size()){
                std::cerr<<"\e[1;30mCould not find <"<<filename<<"> in the given points clouds\e[0m"<<std::endl;
                continue;
                // return false;
            }
            
           
            //* find scale in the model
            indexScale = 0;
            while((scale != vectorOut[indexModel].scales[indexScale].scale) && (indexScale < vectorOut[indexModel].scales.size()))
                indexScale++;

            if(indexScale == vectorOut[indexModel].scales.size()){
                std::cerr<<"\e[1;30mCould not find scale <"<<scale<<"> in the set. (certainly an error occurred in the extractions, please, call the devs)\e[0m"<<std::endl;
                continue;
                // return false;
            }
            
            //* extract the features and save them
            // vectorOut[indexModel].scales[indexScale].features.reset(new CloudSignT);
            if(!loadCloud(iter_files_features->path(), *vectorOut[indexModel].scales[indexScale].features)){
                std::cerr<<"\e[1;30mError(s) occurred while extracting <"<<iter_files_features->path().filename().string()<<">.\e[0m"<<std::endl;
                continue;
                // return false;
            }

            iter_files_features++;
        }
        std::cout<<std::endl; // just for info because it can be long
        return true;
    }



    bool MFPFHLocalizer::extractScene(){
        pcl::ScopeTime timer("Scene Extraction");
        boost::filesystem::path pathScene(sceneFile_);
        //* Load cloud
        scenePoints_.reset(new CloudT);
        if(!loadCloud(pathScene, *scenePoints_))
            return false; //! there're already enough error messages.
        //! make cloud unorganized:
        //? Remove unknow error:
        // [pcl::OrganizedNeighbor::radiusSearch] Input dataset is not from a projective device!
        scenePoints_->height = 1;
        scenePoints_->width = scenePoints_->points.size();
        //* Remove origins points: 0. 0. 0.


        pcl::ConditionOr<PointT>::Ptr non_null_cond (new pcl::ConditionOr<PointT> ());
        const float e_x(0.04350), e_y(0.04350), e_z(0.03675); // Values based on ouster_os1_64 approximated dimensions.
        non_null_cond->addComparison(pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::LT, -e_x)));
        non_null_cond->addComparison(pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::LT, -e_y)));
        non_null_cond->addComparison(pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::LT, -e_z)));
        non_null_cond->addComparison(pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("x", pcl::ComparisonOps::GT, e_x)));
        non_null_cond->addComparison(pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("y", pcl::ComparisonOps::GT, e_y)));
        non_null_cond->addComparison(pcl::FieldComparison<PointT>::Ptr (new pcl::FieldComparison<PointT>("z", pcl::ComparisonOps::GT, e_z)));
        pcl::ConditionalRemoval<PointT> filter_null_points;
        filter_null_points.setCondition(non_null_cond);
        filter_null_points.setInputCloud(scenePoints_);
        filter_null_points.filter(*scenePoints_);
        


        //* Compute normals
        sceneNormals_.reset(new CloudNormT);
        {
            pcl::ScopeTime timeNormal("Computing normals");
            pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>);
            normal_estimator_.setSearchMethod(kdTree);
            normal_estimator_.setRadiusSearch(0.05); //radius of 2.5cm -> search in a sphere of diameter 5cm
            normal_estimator_.setInputCloud(scenePoints_);
            normal_estimator_.compute(*sceneNormals_);
        }
        //* Compute FPFH scene
        sceneFeatures_.reset(new CloudSignT);
        {
            //if (pcl::io::loadPCDFile<pcl::FPFHSignature33> ("/home/wzeng/multiscale-fpfh/localizer/build/fpfhs/reso10fpfh/"+std::to_string(scale)+".pcd",*sceneFeatures_)==-1){
             //   std::cout<<"no data so compute and save"<<std::endl;
                pcl::ScopeTime timeFeatures("Computing FPFH");
                fpfh_estimator_.setInputCloud(scenePoints_);
                fpfh_estimator_.setRadiusSearch(0.05); // set to 0.05 for now, will be modified later
                fpfh_estimator_.setInputNormals(sceneNormals_);
                fpfh_estimator_.setSearchSurface(scenePoints_);
                fpfh_estimator_.compute(*sceneFeatures_);
                
        
                //pcl::io::savePCDFileASCII ("/home/wzeng/multiscale-fpfh/localizer/build/fpfhs/test.pcd",*sceneFeatures_);
            //}
            //else{
                        //std::cout<<"get data!!!!"<<std::endl;
                        
            //}
        }
        std::cout<<"\e[1mStatistics of the scene \""<<pathScene.filename().string()<<"\"\e[0m"<<std::endl;
        std::cout<<"Cloud:\n"<<*scenePoints_<<"Normals:\n"<<*sceneNormals_<<"Features:\n"<<*sceneFeatures_<<std::endl;
        return true;
    }

    bool MFPFHLocalizer::extractObject(){
        pcl::ScopeTime timer("Object Extraction");
        //* Extract clouds
        if(!extractClouds(modelObject_, objectPath_))
            return false; //! there're already enough error messages.
        //* init the scale structure
        initScales(modelObject_, scalesObj_);
        //initScales_forAIO(Allinone_, scalesObj_);
        //* Extract features

        if(!extractFeatures_allinone(Allinone_, objectPath_, scalesObj_))
            return false;
        //std::sort(Allinone_features.begin(), Allinone_features.end());
        if(!extractFeatures(modelObject_, objectPath_))
            return false; //! there're already enough error messages.
        if(!extract_value_k(modelObject_, objectPath_));       //for identif value k
            //return false; //! there're already enough error messages.
        //* check the model (because I'm not sure of myself)
        std::cout<<"\e[1mStatistics of model \""<<objName_<<"\"\e[0m"<<std::endl;
        std::cout<<modelObject_<<std::endl;
        // modelStatistics(modelObject_);
        return true;
    }

} // namespace multiscale_fpfh