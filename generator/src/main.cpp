#include <boost/program_options.hpp>
#include "mfpfh.h"

int main(int argc, char** argv){
    namespace po = boost::program_options;

    // Declare options:
    std::string object_name, output_path;
    bool has_normals;
    po::options_description mandatory_args("Mandatory arguments");
    mandatory_args.add_options()
        ("name,n", po::value<std::string>(&object_name), "Name of the object in the folder.")
    ;
    po::options_description optional_args("Optional arguments");
    optional_args.add_options()
        ("help,h", "Produce this helper.")
        ("odir,o", po::value<std::string>(&output_path)->default_value("/local/users/tlasguigne/work/dev/multiscale-fpfh/output"), "Output directory.")
        ("normals,N", po::bool_switch(&has_normals)->default_value(false), "Define if the normals are defined in the clouds.")
        ("pose,p", po::value<std::vector<double> >()->multitoken(), "Pose of the object in the world. (either translation (xyw), rotation as quaternion (xyzw) or both(xyzxyzw))")
        ("scales,s", po::value<std::vector<double> >()->multitoken(), "Scale limits. The scales will be set as the vector S s.t. S={s(i+1) = s(i) / 2 | s(0) = max(scales), s >= min(scales)}")
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
    std::cout<<"Performing generation for "<<object_name<<std::endl;

    Eigen::Isometry3d objInWorld = Eigen::Isometry3d::Identity();
    if(var_map.count("pose")){
        std::vector<double> pose_vec = var_map["pose"].as<std::vector<double> >();
        switch (pose_vec.size()){
            case 3:
                objInWorld.translation() << pose_vec[0], pose_vec[1], pose_vec[2];
            break;
            case 4:
                objInWorld.rotate(Eigen::Quaterniond(pose_vec[3], pose_vec[0], pose_vec[1], pose_vec[2])); // xyzw -> wxyz
            break;
            case 7:
                objInWorld.translation() << pose_vec[0], pose_vec[1], pose_vec[2];
                objInWorld.rotate(Eigen::Quaterniond(pose_vec[6], pose_vec[3], pose_vec[4], pose_vec[5])); // xyzw -> wxyz
            break;
            default:
                std::cerr<<"Too much or missing data as \"pose\" argument. See helper bellow."<<std::endl;
                std::cout<<all_args<<std::endl;
                return -1;
        }
    } else {
        objInWorld.translation() << 0.625, 0.25, 0.16;
    }

    double scale_max(1.34), scale_min(0.01);
    if(var_map.count("scales")){
        std::vector<double> scales_vec = var_map["scales"].as<std::vector<double> >();
        scale_max = *std::max_element(scales_vec.begin(), scales_vec.end());
        scale_min = *std::min_element(scales_vec.begin(), scales_vec.end());
    }
    std::cout<<"Scales limits are: "<<scale_max<<", "<<scale_min<<std::endl;

    multiscale_fpfh::MFPFHEstimation generator(object_name, output_path, objInWorld);
    generator.setScalesLimits(scale_max, scale_min);
    
    if(has_normals){
        std::cout<<"Computing with normals"<<std::endl;
        generator.runWithNormals();
    } else
        std::cout<<"Computing without normals"<<std::endl;
        generator.run();

    return 0;
}