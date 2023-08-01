#include <boost/program_options.hpp>
#include "localizer.h"






int main(int argc, char** argv){
    namespace po = boost::program_options;




    // Declare options:
    std::string object_name,
                method,
                input_path("/home/zeng/multiscale-fpfh/output"),
                output_path("/home/zeng/multiscale-fpfh/output"),
                scene_file("/home/zeng/multiscale-fpfh/data/cloud/bauzil_plateforms/bauzil_plateforms_45.pcd");
    std::vector<double> scales;
    int kmean_number =0;
    //double eps=0 ;
    //double max_steps=10;

    po::options_description mandatory_args("Mandatory arguments");
    mandatory_args.add_options()
        ("name,n", po::value<std::string>(&object_name), "Name of the object in the folder.")
    ;
    po::options_description optional_args("Optional arguments");
    optional_args.add_options()
        ("help,h", "Produce this helper.")
        ("method,m", po::value<std::string>(&method)->default_value("scale-drop"), "Define the method to be used. Actually implemented: object-plan, scale-drop.")
        ("kmean-number,k", po::value<int>(&kmean_number)->default_value(100), "Define the number of the k-mean for the Clussterization.")
        ("scales,S", po::value<std::vector<double> >(&scales)->multitoken(), "Define the starting (>=max) and stopping (<=min) scales for the scale-drop.")
        ("scene-file,s", po::value<std::string>(&scene_file)->default_value("/home/zeng/multiscale-fpfh/data/cloud/scene_side/scene_side.pcd"), "Path to the scene point cloud.")
        ("input-path,i", po::value<std::string>(&input_path)->default_value("/home/zeng/multiscale-fpfh/output"), "Input directory.")
        ("output-path,o", po::value<std::string>(&output_path)->default_value("/home/zeng/multiscale-fpfh/output"), "Output directory.")
        ("init-guess,g", po::value<std::vector<double> >()->multitoken(), "Initial guess of the object position. (either translation (xyw), rotation as quaternion (xyzw) or both(xyzxyzw))")
        ("test,T",po::value<std::string>(&method)->default_value("test-gmm"),"test GMM")
    ;
    po::options_description all_args("Arguments");
    all_args.add(mandatory_args).add(optional_args);
    po::variables_map var_map;
    po::store(po::command_line_parser(argc, argv).options(all_args).run(), var_map);
    po::notify(var_map);

    if(var_map.count("help")) {
        std::cout<<all_args<<std::endl;
        return 0;
    }

    if(!var_map.count("name")){
        std::cerr<<"\"name,n\" is a mandatory argument. See helper bellow."<<std::endl;
        std::cout<<all_args<<std::endl;
        return -1;
    }
    std::cout<<"Performing localisation for "<<object_name<<std::endl;

    Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
    if(var_map.count("init-guess")){
        std::vector<double> pose_vec = var_map["init-guess"].as<std::vector<double> >();
        switch (pose_vec.size()){
            case 3:
                initial_guess.translation() << pose_vec[0], pose_vec[1], pose_vec[2];
            break;
            case 4:
                initial_guess.rotate(Eigen::Quaterniond(pose_vec[3], pose_vec[0], pose_vec[1], pose_vec[2])); // xyzw -> wxyz
            break;
            case 7:
                initial_guess.translation() << pose_vec[0], pose_vec[1], pose_vec[2];
                initial_guess.rotate(Eigen::Quaterniond(pose_vec[6], pose_vec[3], pose_vec[4], pose_vec[5])); // xyzw -> wxyz
            break;
            default:
                std::cerr<<"Too much or missing data as \"init-guess,i\" argument. See helper bellow."<<std::endl;
                std::cout<<all_args<<std::endl;
                return -1;
        }
    }

    multiscale_fpfh::MFPFHLocalizer localizer(object_name, //object name
                                              input_path, //input dir
                                              output_path, //output dir
                                              scene_file); //scene direct path

    /*if(var_map.count("method")){
        if(var_map["method"].as<std::string>() == "test-gmm"){
            //const vector<double> &inputData
            test(const int clustNum = 5, double eps = 0.01, double max_steps = 20);

        }*/
        if(var_map["method"].as<std::string>() == "scale-drop"){
            double scale_start(1.34), scale_limit(0.1);
            if(var_map.count("scales")){
                std::vector<double> scales_vec = var_map["scales"].as<std::vector<double> >();
                if(scales_vec.size() == 2){
                    scale_start = *std::max_element(scales_vec.begin(), scales_vec.end());
                    scale_limit = *std::min_element(scales_vec.begin(), scales_vec.end());
                } else if(scales_vec.size() == 1){
                    scale_start = scales_vec.front();
                    std::cout<<"Scale limit not defined, will use the default value."<<std::endl;
                } else {
                    std::cout<<"Error on the scale limits definition. Please check that you have provided 1 or 2 elements to the \"scales,S\" option."<<std::endl;
                    return -1;
                }
            }
            std::cout<<"Scale drop for scales from "<<scale_start<<" to "<<scale_limit<<std::endl;
            if(var_map.count("kmean-number")){
                //std::<int> kmeans_vec = var_map["kmean-number"].as<std::vector<int> >();
                //int kmeans_num=var_map["kmean-number"].as<int>;
                //kmeans_num=kmeans_vec.front();
                if(kmean_number >0){
                    std::cout<<"happy is +num for kmean"<<std::endl;
                }else if(kmean_number <0){
                    std::cout<<"i dont like - , so try it with + again"<<std::endl;
                    std::cout<<"Error on the kmeans number definition. "<<std::endl;
                    return -1;
                }else{
                    std::cout<<"Error on the kmeans number definition. Please check that you have provided 1 element to the \"kmeans-number,k\" option."<<std::endl;
                    return -1;
                }

            }
            std::cout<<"The kmean number which you want to test is "<<kmean_number<<std::endl;
            localizer.runSurprise(scale_start, scale_limit, initial_guess, kmean_number);
        } else if(var_map["method"].as<std::string>() == "object-plan"){
            localizer.runClusters(initial_guess);
        }
    //}   for method test gmm

    return 0;
}