#include "extractor.h"


namespace multiscale_fpfh
{
    void PressEnterToContinueROS(){
        ROS_INFO("Press ENTER to continue... ");
        std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );
    }
    
    bool askUserValidationROS(std::string question){
        char answer;
        ROS_INFO("%s y/n", question.c_str());
        std::cin>>answer;
        if(toupper(answer) == 'Y'){
            return true;
        }
        return false;
    }
    Extractor::Extractor() {
        std::cerr<<"\e[1;30mError while creating app's instance. ros::NodeHandle must be given as argument.\e[0m"<<std::endl;
        exit(-1); //! don't like it but let's go!
    }

    Extractor::Extractor(ros::NodeHandle* nh, ExtractorConfig config):
            nh_(nh), config_(config) {
        assert(nh_ && "Error when transmitting ros::NodeHandle.");
        assert(((config_.resolution > 0) || ((config_.th_resolution > 0) && (config_.ph_resolution > 0))) 
               && "Resolution should be defined, either on global resolution or on specific angles resolution (th and ph).");
        if (config_.resolution > 0){
            config_.th_resolution = config_.resolution;
            config_.ph_resolution = config_.resolution;
            ROS_INFO("Set resolutions to: th:=%d, ph:=%d", config_.th_resolution, config_.ph_resolution);
        }
        setModelClient_ = nh_->serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        checkPathAndDir();
        debug_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("debug_cloud", 10);
    }

    Extractor::~Extractor(){
    }

    void Extractor::run(void){
        CloudT::Ptr cloudIn (new CloudT), cloudSave (new CloudT);
        pcl::PCLPointCloud2 cloudPCLpc2;
        sensor_msgs::PointCloud2 pcIn;
        Eigen::Isometry3d newPose = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d transformation = Eigen::Isometry3d::Identity();
        // Build the conditional filter for points in the LiDAR filter
        // Null point condition: (-e_x < x < e_x).(-e_y < y < e_y).(-e_z < z < e_z)
        // => non null point condition: ((x < -e_x) + (x > e_x)) + ((y < -e_y) + (y > e_y)) + ((z < -e_z) + (z > e_z))
        // (priority can be removed)
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

        float radius = 1.34;
        float theta;
        float phi;
        ROS_INFO("Get resolutions to: th:=%d, ph:=%d", config_.th_resolution, config_.ph_resolution);
        for(theta = -M_PI; theta < M_PI; theta += ((M_PI/2.)/config_.th_resolution)){
            for(phi = M_PI/2.; phi > std::numeric_limits<float>::epsilon(); phi -= ((M_PI/2.)/config_.ph_resolution)){
                //******** Set new pose ********//
                newPose = spherical2cartesian(radius, theta, phi);
                setNewPose(newPose);

                //******* Gen new cloud ********//
                pcIn = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/os_cloud_node/points"));
                pcl_conversions::toPCL(pcIn, cloudPCLpc2);
                cloudIn.reset(new CloudT);
                pcl::fromPCLPointCloud2(cloudPCLpc2, *cloudIn);

                //***** Filter null points ******//
                filter_null_points.setInputCloud(cloudIn);
                filter_null_points.filter(*cloudIn);

                if(config_.save_normals){
                    //****** Compute Normals *******//
                    CloudPointNormT::Ptr cloudNormals(new CloudPointNormT);
                    CloudNormalT::Ptr normals(new CloudNormalT);
                    pcl::search::KdTree<PointT>::Ptr kdTree(new pcl::search::KdTree<PointT>);
                    normalEstimation_.setSearchMethod(kdTree);
                    normalEstimation_.setRadiusSearch(0.05); //radius of 5cm -> search in a sphere of diameter 10cm
                    normalEstimation_.setInputCloud(cloudIn);
                    normalEstimation_.compute(*normals);
                    pcl::concatenateFields(*cloudIn, *normals, *cloudNormals);
                    ROS_INFO("Normals computed.");
                    //********* Transform **********//
                    ROS_INFO("Give newPose: x:=%f y:=%f z:=%f", newPose.translation().x(), newPose.translation().y(), newPose.translation().z());
                    post_process(*cloudNormals, radius, newPose);
                } else {
                    //********* Transform **********//
                    post_process(*cloudIn, radius, newPose);
                }
            }
        }
    }

    bool Extractor::setNewPose(Eigen::Isometry3d sensorPose){
        geometry_msgs::Transform newPoseGM;
        tf::transformEigenToMsg(sensorPose, newPoseGM);

        gazebo_msgs::ModelState newState;
        newState.model_name = (std::string) "sensor";
        newState.reference_frame = (std::string) "world";
        newState.pose.position.x = newPoseGM.translation.x;
        newState.pose.position.y = newPoseGM.translation.y;
        newState.pose.position.z = newPoseGM.translation.z;
        newState.pose.orientation.x = newPoseGM.rotation.x;
        newState.pose.orientation.y = newPoseGM.rotation.y;
        newState.pose.orientation.z = newPoseGM.rotation.z;
        newState.pose.orientation.w = newPoseGM.rotation.w;
        newState.twist.linear.x = 0.0;
        newState.twist.linear.y = 0.0;
        newState.twist.linear.z = 0.0;
        newState.twist.angular.x = 0.0;
        newState.twist.angular.y = 0.0;
        newState.twist.angular.z = 0.0;

        gazebo_msgs::SetModelState setModelStateMsg;
        setModelStateMsg.request.model_state = newState;

        ROS_INFO("Calling service.");
        setModelClient_.call(setModelStateMsg);
        ROS_INFO("Service called.");

        return true;
    }

    Eigen::Isometry3d Extractor::spherical2cartesian(float radius, float theta, float phi){
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();

        float z = radius * cosf(phi);
        float x = radius * sinf(phi) * cosf(theta);
        float y = radius * sinf(phi) * sinf(theta);

        result.rotate(Eigen::AngleAxisd(theta-M_PI, Eigen::Vector3d::UnitZ()));
        result.rotate(Eigen::AngleAxisd((M_PI/2.)-phi, Eigen::Vector3d::UnitY()));

        result.translation() << x, y, z;

        return result;
    }

    std::string Extractor::defineFilename(float radius, Eigen::Isometry3f pose, Eigen::Quaternionf q, bool wo){
        std::string filename = config_.outputFolder+"/"+config_.objectName+(wo?"/wo/":"/")+config_.objectName
                               +"_r"+std::to_string(radius)
                               +"_px"+std::to_string(pose.translation()(0))
                               +"_py"+std::to_string(pose.translation()(1))
                               +"_pz"+std::to_string(pose.translation()(2))
                               +"_ox"+std::to_string(q.x())
                               +"_oy"+std::to_string(q.y())
                               +"_oz"+std::to_string(q.z())
                               +"_ow"+std::to_string(q.w())
                               +".pcd";
        return filename;
    }

    void Extractor::checkPathAndDir(){
        boost::filesystem::path pathSaveDir(config_.outputFolder);
        if(!(boost::filesystem::exists(pathSaveDir) && boost::filesystem::is_directory(pathSaveDir))){
            if(!boost::filesystem::create_directories(pathSaveDir)){
                std::cerr<<"\e[1;30mError while checking the save path given. Not able to ensure it existence.\e[0m"<<std::endl;
                std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and/or is where you want to save.\e[0m"<<std::endl;
                exit(-1); //! don't like it but let's go!
            }
        }
        pathSaveDir+="/"+config_.objectName;
        if(boost::filesystem::exists(pathSaveDir)){
            if(!askUserValidationROS("Folder "+pathSaveDir.string()+" already exists. Continue?")){
                exit(-1); //! don't like it but let's go!
            }
        } else {
            if(!boost::filesystem::create_directory(pathSaveDir)){
                std::cerr<<"\e[1;30mError while checking the save path given. Not able to ensure it existence.\e[0m"<<std::endl;
                std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and/or is where you want to save.\e[0m"<<std::endl;
                exit(-1); //! don't like it but let's go!
            }
            pathSaveDir+="/wo";
            if(!boost::filesystem::create_directory(pathSaveDir)){
                std::cerr<<"\e[1;30mError while checking the save path given. Not able to ensure it existence.\e[0m"<<std::endl;
                std::cerr<<"\e[1;30m\tPlease check that <"<<pathSaveDir.c_str()<<"> exists and/or is where you want to save.\e[0m"<<std::endl;
                exit(-1); //! don't like it but let's go!
            }
        }
        ROS_INFO("Ready to save in <%s{,/wo}>", pathSaveDir.c_str());
    }

} // namespace multiscale_fpfh
