#ifndef LOCALIZER_TMPL_H
#define LOCALIZER_TMPL_H

#include "localizer.h"

namespace multiscale_fpfh{

    template <typename CloudInT>
    bool MFPFHLocalizer::saveCloud(boost::filesystem::path filepath, CloudInT cloud){
        // Check for the folder existence
        boost::filesystem::path path_base(filepath.parent_path());
        if(!(boost::filesystem::exists(path_base) && boost::filesystem::is_directory(path_base))){
            if(!boost::filesystem::create_directories(path_base)){
                std::cerr<<"\e[1;30mError while checking the parent directory <"<<path_base.c_str()<<">.\e[0m"<<std::endl;
                std::cerr<<"\e[1;30m\tPlease check that the system have the right to write <"<<filepath.c_str()<<"> and its parents.\e[0m"<<std::endl;
                return false;
            }
        }

        if(filepath.extension() == ".pcd"){
            //load the cloud
            if(pcl::io::savePCDFile(filepath.string(), cloud) == -1){
                PCL_ERROR("%s could not save pcd file.\n", filepath.string());
                return false;
            }
        } else if(filepath.extension() == ".ply"){
            //load the cloud
            if(pcl::io::savePLYFile(filepath.string(), cloud) == -1){
                PCL_ERROR("%s could not save ply file.\n", filepath.string());
                return false;
            }
        } else {
            std::cout<<filepath.string()<<" does not have a coherent extension."<<std::endl;
            return false;
        }
        return true;
    }

    template <typename CloudInT>
    bool MFPFHLocalizer::loadCloud(boost::filesystem::path filepath, CloudInT& cloud){
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

} // namespace multiscale_fpfh

#endif