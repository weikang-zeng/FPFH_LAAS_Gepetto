#include "localizer.h"

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
    void MFPFHLocalizer::generateExecDate(){
        std::time_t myTime(std::time(NULL));
        std::stringstream ss;
        ss<<std::put_time(std::localtime(&myTime),"%y-%d-%m");
        execDateStr_ = ss.str();
        std::stringstream ss2;
        ss2<<std::put_time(std::localtime(&myTime),"%H-%M-%S");
        execHourStr_ = ss2.str();
    }
    
    MFPFHLocalizer::MFPFHLocalizer():
            objName_(""), inputPath_(BASE_DIR_OUTPUT), outputPath_(BASE_DIR_OUTPUT), sceneFile_(""),
            objectPath_(""), savePath_(""),
            initialGuess_(Eigen::Isometry3d::Identity()) {
        std::cerr<<"\e[1;30mError while creating app's instance. the object name must be given as argument.\e[0m"<<std::endl;
        exit(-1); //! don't like it but let's go!
    }
    MFPFHLocalizer::MFPFHLocalizer(std::string object):
            objName_(object), inputPath_(BASE_DIR_OUTPUT), outputPath_(BASE_DIR_OUTPUT), sceneFile_(""),
            objectPath_(""), savePath_(""),
            initialGuess_(Eigen::Isometry3d::Identity()) {
        generateExecDate();
        checkPathAndDir();
    }
    MFPFHLocalizer::MFPFHLocalizer(std::string object, std::string path):
            objName_(object), inputPath_(path), outputPath_(path), sceneFile_(""),
            objectPath_(""), savePath_(""),
            initialGuess_(Eigen::Isometry3d::Identity()) {
        generateExecDate();
        checkPathAndDir();
    }
    MFPFHLocalizer::MFPFHLocalizer(std::string object, std::string pathIn, std::string pathOut):
            objName_(object), inputPath_(pathIn), outputPath_(pathOut), sceneFile_(""),
            objectPath_(""), savePath_(""),
            initialGuess_(Eigen::Isometry3d::Identity()) {
        generateExecDate();
        checkPathAndDir();
    }
    MFPFHLocalizer::MFPFHLocalizer(std::string object, std::string pathIn, std::string pathOut, std::string fileScene):
            objName_(object), inputPath_(pathIn), outputPath_(pathOut), sceneFile_(fileScene),
            objectPath_(""), savePath_(""),
            initialGuess_(Eigen::Isometry3d::Identity()) {
        generateExecDate();
        checkPathAndDir();
    }
    MFPFHLocalizer::~MFPFHLocalizer(){
    }

    void MFPFHLocalizer::checkPathAndDir(){
        //** Check input path
        boost::filesystem::path pathSaveDir(inputPath_);
        if(!(boost::filesystem::exists(pathSaveDir) && boost::filesystem::is_directory(pathSaveDir))){
            std::cerr<<"\e[1;30mError while checking the input directory given. Not able to ensure it existence.\e[0m"<<std::endl;
            std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and is where your clouds are.\e[0m"<<std::endl;
            exit(-1); //! don't like it but let's go!
        }
        
        //** Check object path
        pathSaveDir = inputPath_ + "/" + objName_;
        if(!(boost::filesystem::exists(pathSaveDir) && boost::filesystem::is_directory(pathSaveDir))){
            std::cerr<<"\e[1;30mError while checking the first object's ("<<objName_<<") directory. Not able to ensure it existence.\e[0m"<<std::endl;
            std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and is where your clouds are.\e[0m"<<std::endl;
            exit(-1); //! don't like it but let's go!
        }
        objectPath_ = inputPath_ + "/" + objName_;
        //? Check subfolders
        pathSaveDir = objectPath_ + "/points";
        if(!(boost::filesystem::exists(pathSaveDir) && boost::filesystem::is_directory(pathSaveDir))){
            std::cerr<<"\e[1;30mError while checking the first object's ("<<objName_<<") points' directory. Not able to ensure it existence.\e[0m"<<std::endl;
            std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and is where your clouds are.\e[0m"<<std::endl;
            exit(-1); //! don't like it but let's go!
        }
        pathSaveDir = objectPath_ + "/features";
        if(!(boost::filesystem::exists(pathSaveDir) && boost::filesystem::is_directory(pathSaveDir))){
            std::cerr<<"\e[1;30mError while checking the first object's ("<<objName_<<") features' directory. Not able to ensure it existence.\e[0m"<<std::endl;
            std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and is where your clouds are.\e[0m"<<std::endl;
            exit(-1); //! don't like it but let's go!
        }

        //** Check scene path
        pathSaveDir = sceneFile_;
        if(!(boost::filesystem::exists(pathSaveDir) && boost::filesystem::is_regular_file(pathSaveDir))){
            std::cerr<<"\e[1;30mError while checking the scene ("<<sceneFile_<<") file. Not able to ensure it existence.\e[0m"<<std::endl;
            std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and is your cloud's scene.\e[0m"<<std::endl;
            exit(-1); //! don't like it but let's go!
        }


        //** Check output path
        pathSaveDir = outputPath_ + "/" + objName_ + "_localisation";
        if(!(boost::filesystem::exists(pathSaveDir) && boost::filesystem::is_directory(pathSaveDir))){
            if(!boost::filesystem::create_directory(pathSaveDir)){
                std::cerr<<"\e[1;30mError while checking the save path given. Not able to ensure it existence.\e[0m"<<std::endl;
                std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and/or is where you want to save.\e[0m"<<std::endl;
                exit(-1); //! don't like it but let's go!
            }
        }
        pathSaveDir = outputPath_ + "/" + objName_ + "_localisation/"+execDateStr_;
        if(!(boost::filesystem::exists(pathSaveDir) && boost::filesystem::is_directory(pathSaveDir))){
            if(!boost::filesystem::create_directory(pathSaveDir)){
                std::cerr<<"\e[1;30mError while checking the save path given. Not able to ensure it existence.\e[0m"<<std::endl;
                std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and/or is where you want to save.\e[0m"<<std::endl;
                exit(-1); //! don't like it but let's go!
            }
        }
        pathSaveDir = outputPath_ + "/" + objName_ + "_localisation/" + execDateStr_ + "/" + execHourStr_;
        if(!boost::filesystem::create_directory(pathSaveDir)){
            std::cerr<<"\e[1;30mError while creating the output directory.\e[0m"<<std::endl;
            std::cerr<<"\e[1;30m\tPlease check that the system have the right to create <"<<pathSaveDir.c_str()<<">.\e[0m"<<std::endl;
            exit(-1); //! don't like it but let's go!
        }
        savePath_ = outputPath_ + "/" + objName_ + "_localisation/" + execDateStr_ + "/" + execHourStr_;

        std::string logFilename = savePath_+"/log_"+execDateStr_+"_"+execHourStr_+".log";
        logFile_.open(logFilename, ios::out);
        std::cout<<"Log file: "<<logFilename<<std::endl;
        if(!logFile_.is_open()){
            std::cerr<<"\e[1;30mLog file does not open. (<"<<logFilename<<">)\e[0m"<<std::endl;
            return;
        }
    }
    /*add it for Allinone
    void MFPFHLocalizer::initScales_forAIO(std::vector<ModelFeaturesT>& vectorOut, std::vector<float> vectorScales){
        //* initial structure of model Features allinone
            for (size_t indexScales = 0; indexScales < vectorScales.size(); indexScales++){
                ModelFeaturesT new_scale;
                new_scale.scale = vectorScales[indexScales];
                new_scale.features.reset(new CloudSignT);
                vectorOut[indexScales].push_back(new_scale);
            }
        }
    }*/


    void MFPFHLocalizer::initScales(std::vector<ModelCompleteT>& vectorOut, std::vector<float> vectorScales){
        for(size_t indexClouds = 0; indexClouds < vectorOut.size(); indexClouds++){
            vectorOut[indexClouds].scales.clear();
            for (size_t indexScales = 0; indexScales < vectorScales.size(); indexScales++){
                ModelFeaturesT new_scale;
                new_scale.scale = vectorScales[indexScales];
                new_scale.features.reset(new CloudSignT);
                vectorOut[indexClouds].scales.push_back(new_scale);
            }
        }
    }
    
    std::ostream& operator<<(std::ostream& os, std::vector<MFPFHLocalizer::ModelCompleteT> model){
        os<<"Model Size:\n\t"<<model.size()<<std::endl;
        if(model.size() == 0){
            os<<"(empty model)"<<std::endl;
            return os;
        }
        os<<"Number of scales in the model:\n\t"<<model[0].scales.size()<<std::endl;
        os<<"Scales are:\n\t";
        for (auto i = 0; i < model[0].scales.size(); i++)
            os<<model[0].scales[i].scale<<" "<<std::flush;
        os<<std::endl;
        for(auto indMod = 0; indMod < model.size(); indMod++){
            for(auto indScl = 0; indScl < model[indMod].scales.size(); indScl++){
                if(model[indMod].scales[indScl].features->empty())
                    os<<"Part <"<<model[indMod].partName<<"> of the model has no features at scale <"<<model[indMod].scales[indScl].scale<<">"<<std::endl;
            }
        }
        return os;
    }

    void MFPFHLocalizer::modelStatistics(std::vector<ModelCompleteT> model){
        std::cout<<"Model Size:\n\t"<<model.size()<<std::endl;
        if(model.size() == 0){
            std::cout<<"(empty model)"<<std::endl;
            return;
        }std::cout<<"Number of scales in the model:\n\t"<<model[0].scales.size()<<std::endl;
        std::cout<<"Scales are:\n\t";
        for (auto i = 0; i < model[0].scales.size(); i++)
            std::cout<<model[0].scales[i].scale<<" "<<std::flush;
        std::cout<<std::endl;
        for(auto indMod = 0; indMod < model.size(); indMod++){
            for(auto indScl = 0; indScl < model[indMod].scales.size(); indScl++){
                if(model[indMod].scales[indScl].features->empty())
                    std::cout<<"Part <"<<model[indMod].partName<<"> of the model has no features at scale <"<<model[indMod].scales[indScl].scale<<">"<<std::endl;
            }
        }
    }

    std::ostream& operator<<(std::ostream& os, MFPFHLocalizer::ResultCompareT result){
        Eigen::Quaterniond q (result.pose.rotation());
        os<<"Cluster index: "<<result.indexCluster<<std::endl;
        os<<"  Model index: "<<result.indexInModel<<std::endl;
        os<<"      Fitness: "<<result.fitness<<std::endl;
        os<<"    Transform: "<<std::endl
          <<"    - translation:"<<std::endl
          <<"           x: "<<result.pose.translation().x()<<std::endl
          <<"           y: "<<result.pose.translation().y()<<std::endl
          <<"           z: "<<result.pose.translation().z()<<std::endl
          <<"    -    rotation:"<<std::endl
          <<"           x: "<<q.x()<<std::endl
          <<"           y: "<<q.y()<<std::endl
          <<"           z: "<<q.z()<<std::endl
          <<"           w: "<<q.w()<<std::endl;
        return os;
    }

    bool MFPFHLocalizer::compareComparisonResults(const ResultCompareT &a, const ResultCompareT &b){
        return a.fitness < b.fitness;
    }

} // namespace multiscale_fpfh