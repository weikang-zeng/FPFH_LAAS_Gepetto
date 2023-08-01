#include "mfpfh.h"


namespace multiscale_fpfh
{
    void PressEnterToContinue(){
        std::cout<<"Press ENTER to continue... "<<std::flush;
        std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );
    }

    bool askUserValidation(std::string question){
        char answer;
        std::cout<<question<<" y/n"<<std::endl;
        std::cin>>answer;
        if(toupper(answer) == 'Y'){
            return true;
        }
        return false;
    }

    MFPFHEstimation::MFPFHEstimation():
            objName_(""), isFinished_(false), savePath_(BASE_DIR_OUTPUT), transformObjInWorld_(Eigen::Isometry3d::Identity()) {
        std::cerr<<"\e[1;30mError while creating app's instance. ros::NodeHandle must be given as argument.\e[0m"<<std::endl;
        exit(-1); //! don't like it but let's go!
    }

    MFPFHEstimation::MFPFHEstimation(std::string objName):
            objName_(objName), isFinished_(false), savePath_(BASE_DIR_OUTPUT), transformObjInWorld_(Eigen::Isometry3d::Identity()) {
        checkPathAndDir();
    }

    MFPFHEstimation::MFPFHEstimation(Eigen::Isometry3d objInWorld):
            objName_(""), isFinished_(false), savePath_(BASE_DIR_OUTPUT), transformObjInWorld_(objInWorld) {
        checkPathAndDir();
    }

    MFPFHEstimation::MFPFHEstimation(std::string objName, std::string savePath):
            objName_(objName), isFinished_(false), savePath_(savePath), transformObjInWorld_(Eigen::Isometry3d::Identity()) {
        checkPathAndDir();
    }

    MFPFHEstimation::MFPFHEstimation(std::string objName, Eigen::Isometry3d objInWorld):
            objName_(objName), isFinished_(false), savePath_(BASE_DIR_OUTPUT), transformObjInWorld_(objInWorld) {
        checkPathAndDir();
    }

    MFPFHEstimation::MFPFHEstimation(std::string objName, std::string savePath, Eigen::Isometry3d objInWorld):
            objName_(objName), isFinished_(false), savePath_(savePath), transformObjInWorld_(objInWorld) {
        checkPathAndDir();
    }

    MFPFHEstimation::~MFPFHEstimation(){
    }

    template <typename CloudInT>
    bool MFPFHEstimation::loadCloud(boost::filesystem::path filepath, CloudInT& cloud){
        if(filepath.extension() == ".pcd"){
            //load the cloud
            if(pcl::io::loadPCDFile(filepath.string(), cloud) == -1){
                PCL_ERROR(" could not load pcd file.\n");
                return false;
            }
        } else if(filepath.extension() == ".ply"){
            //load the cloud
            if(pcl::io::loadPLYFile(filepath.string(), cloud) == -1){
                PCL_ERROR(" could not load pcd file.\n");
                return false;
            }
        } else {
            std::cout<<" not to load."<<std::endl;
            return false;
        }
        return true;
    }

    void MFPFHEstimation::run(void){
        pcl::ScopeTime timer_run("Complete Run");
        float radius = highScale_;

        boost::filesystem::directory_iterator iter_files(savePath_+"/"+objName_);
        boost::filesystem::directory_iterator end_files;

        //? Iterations over files
        while(iter_files != end_files){
            boost::filesystem::path next_file(iter_files->path()); //get file path
            
            if(boost::filesystem::is_regular_file(next_file)){
                CloudT::Ptr cloudIn (new CloudT);
                CloudNormT::Ptr cloudNormal(new CloudNormT);
                std::cout<<"Loading <"<<next_file.string()<<">...  ";
                if(loadCloud<CloudT>(next_file, *cloudIn)){
                    std::cout<<" loaded ! ("<<cloudIn->size()<<" points)"<<std::endl;
                } else {
                    std::cout<<"<"<<next_file.string()<<"> not loaded..."<<std::endl;
                    iter_files++;
                    continue; //! Not clean but easy!
                }
                pcl::ScopeTime timer_scale("Complete file");

                //* compute normals
                {
                    pcl::ScopeTime timeNormal("Computing normals");
                    pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>);
                    normalEstimation_.setSearchMethod(kdTree);
                    normalEstimation_.setRadiusSearch(0.025); //radius of 2.5cm -> search in a sphere of diameter 5cm
                    normalEstimation_.setInputCloud(cloudIn);
                    normalEstimation_.compute(*cloudNormal);
                }

                //? Iterations over radii
                radius = highScale_;
                while(radius > lowScale_){

                    pcl::ScopeTime timeTotal("Complete descriptors generation");
                    //* Deal with the descriptor
                    CloudSignT::Ptr cloudSignature(new CloudSignT);

                    //* Compute FPFH
                    {
                        pcl::ScopeTime timeNormal("Computing FPFH");
                        fpfhEstimation_.setInputCloud(cloudIn);
                        fpfhEstimation_.setRadiusSearch(radius);
                        fpfhEstimation_.setInputNormals(cloudNormal);
                        fpfhEstimation_.setSearchSurface(cloudIn);
                        fpfhEstimation_.compute(*cloudSignature);
                    }
                    
                    //* Save FPFH 
                        //? HOW?
                    std::string cloudSaveName = next_file.parent_path().string()
                                              + "/features/features_s" + std::to_string(radius)
                                              +"_"+next_file.filename().string();
                    if (pcl::io::savePCDFile(cloudSaveName, *cloudSignature, false) == 0)
                        std::cout<<"\e[1;32mSaved features as <"<<cloudSaveName<<">!\e[0m"<<std::endl;
                    else{
                        std::cout<<"Problems occurred while saving features.\n"<<std::endl;
                    }
                    std::cout<<"number of FPFH: "<<cloudSignature->size()<<std::endl;

                    //* Update radius
                    radius /= 2.;
                }
            }
            
            //? Update file
            iter_files++;
        }
    }

    void MFPFHEstimation::runWithNormals(void){
        pcl::ScopeTime timer_run("Complete Run");
        float radius = highScale_;
        boost::filesystem::directory_iterator iter_files(savePath_+"/"+objName_);
        boost::filesystem::directory_iterator end_files;

        //? Iterations over files
        while(iter_files != end_files){
            boost::filesystem::path next_file(iter_files->path()); //get file path

            if(boost::filesystem::is_regular_file(next_file)){
                
                CloudWNormT::Ptr cloudIn (new CloudWNormT);
                std::cout<<"Loading <"<<next_file.string()<<">...  ";
                if(loadCloud<CloudWNormT>(next_file, *cloudIn)){
                    std::cout<<" loaded ! ("<<cloudIn->size()<<" points)"<<std::endl;
                } else {
                    std::cout<<"<"<<next_file.string()<<"> not loaded..."<<std::endl;
                    iter_files++;
                    continue; //! Not clean but easy!
                }

                pcl::ScopeTime timer_scale("Complete file");

                //? Iterations over radii
                radius = highScale_;
                while(radius > lowScale_){
                    pcl::ScopeTime timeTotal("Complete descriptors generation");
                    //* Deal with the descriptor
                    CloudSignT::Ptr cloudSignature(new CloudSignT);

                    //* Compute FPFH
                    {
                        pcl::ScopeTime timeNormal("Computing FPFH");
                        fpfhEstimationWNorm_.setInputCloud(cloudIn);
                        fpfhEstimationWNorm_.setRadiusSearch(radius);
                        fpfhEstimationWNorm_.setInputNormals(cloudIn);
                        fpfhEstimationWNorm_.setSearchSurface(cloudIn);
                        fpfhEstimationWNorm_.compute(*cloudSignature);
                    }
                    
                    //* Save FPFH 
                        //? HOW?
                    std::string cloudSaveName = next_file.parent_path().string()
                                              + "/features/features_s" + std::to_string(radius)
                                              +"_"+next_file.filename().string();
                    if (pcl::io::savePCDFile(cloudSaveName, *cloudSignature, false) == 0)
                        std::cout<<"\e[1;32mSaved features as <"<<cloudSaveName<<">!\e[0m"<<std::endl;
                    else{
                        std::cout<<"Problems occurred while saving features.\n"<<std::endl;
                    }
                    std::cout<<"number of FPFH: "<<cloudSignature->size()<<std::endl;

                    //* Update radius
                    radius /= 2.;
                }
            }
            
            //? Update file
            iter_files++;
        }
    }

    void MFPFHEstimation::checkPathAndDir(){
        //? Check global path
        boost::filesystem::path pathSaveDir(savePath_);
        if(!(boost::filesystem::exists(pathSaveDir) && boost::filesystem::is_directory(pathSaveDir))){
            std::cerr<<"\e[1;30mError while checking the save path given. Not able to ensure it existence.\e[0m"<<std::endl;
            std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and is where your clouds are.\e[0m"<<std::endl;
            exit(-1); //! don't like it but let's go!
        }
        //? Check object path
        pathSaveDir+="/"+objName_;
        if(!(boost::filesystem::exists(pathSaveDir) && boost::filesystem::is_directory(pathSaveDir))){
            std::cerr<<"\e[1;30mError while checking the save path given. Not able to ensure it existence.\e[0m"<<std::endl;
            std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and is where your clouds are.\e[0m"<<std::endl;
            exit(-1); //! don't like it but let's go!
        }
        std::cout<<"The files are:"<<std::endl;
        for(auto & p : boost::filesystem::directory_iterator(pathSaveDir)){
            if(boost::filesystem::is_regular_file(p))
                std::cout<<"\t"<<p<<std::endl;
        }
        //? Check and create features directory
        pathSaveDir+="/features";
        if(boost::filesystem::exists(pathSaveDir)){
            if(!askUserValidation("Folder "+pathSaveDir.string()+" already exists. Continue?")){
                std::cout<<"Leaving"<<std::endl;
                exit(-1); //! don't like it but let's go!
            }
        } else {
            if(!boost::filesystem::create_directory(pathSaveDir)){
                std::cerr<<"\e[1;30mError while checking the save path given. Not able to ensure it existence.\e[0m"<<std::endl;
                std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and/or is where you want to save.\e[0m"<<std::endl;
                exit(-1); //! don't like it but let's go!
            }
        }
    }

} // namespace multiscale_fpfh
