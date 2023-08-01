#include <ros/ros.h>

#include "extractor.h"

int main(int argc, char** argv){

    ros::init(argc, argv, "mfpfh_views_extractor");
    ros::NodeHandle nh("~");
    ROS_INFO("Node handle ready.");

    multiscale_fpfh::ExtractorConfig extractor_config;

    extractor_config.objInWorld.translation() << 0.625, 0.25, 0.16;

    if(nh.param<std::string>("object", extractor_config.objectName, "plateforms"))
        ROS_INFO("Got parameter : object:=%s", extractor_config.objectName.c_str());

    if(nh.param<std::string>("output", extractor_config.outputFolder, "/local/users/tlasguigne/work/dev/multiscale-fpfh/output"))
        ROS_INFO("Got parameter : output:=%s", extractor_config.outputFolder.c_str());

    if(nh.param<int>("reso", extractor_config.resolution, 10))
        ROS_INFO("Got parameter : reso:=%d", extractor_config.resolution);

    if(nh.param<bool>("for_model", extractor_config.for_model, true))
        ROS_INFO("Got parameter : is_model:=%s", extractor_config.for_model?"true":"false");

    if(nh.param<bool>("save_normals", extractor_config.save_normals, false))
        ROS_INFO("Got parameter : is_model:=%s", extractor_config.save_normals?"true":"false");

    multiscale_fpfh::Extractor views_extractor(&nh, extractor_config);

    views_extractor.run();

    return 0;
}