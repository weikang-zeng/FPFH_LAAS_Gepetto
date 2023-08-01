#include "localizer.h"

namespace multiscale_fpfh
{
    void MFPFHLocalizer::runkmeans(Eigen::Isometry3d initialGuess){
        initialGuess_ = initialGuess;
        runkmeans();
    }

    void MFPFHLocalizer::runkmeans(){
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

        return;
    }
    
} // namespace multiscale_fpfh