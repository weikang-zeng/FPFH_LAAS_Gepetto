#include <iostream>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/video/video.hpp>
#include <opencv/cv.h>

//PCL
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

typedef struct{
    bool active;
    int maxIter;
    int nbSamples;
    int corrRand;
    float inlierF;
    float simiThresh;
    float maxCorrDist;
} FPFHparameters;

typedef pcl::PointXYZRGB PointTReg;
typedef pcl::PointXYZRGBNormal PointTNorm;

bool finish = false;
bool tryAgain = false;

//function to get ICP aligned pointcloud using Color Supporting Generalized-ICP
Eigen::Matrix4f getFPFH(pcl::PointCloud<PointTReg>::Ptr clSrcXYZRGB,
                        pcl::PointCloud<PointTReg>::Ptr clTgtXYZRGB,
                        FPFHparameters params,
                        bool& convergence)
{
    //clouds to store intermediate results
    pcl::PointCloud<PointTReg>::Ptr cloudfirst ( new pcl::PointCloud<PointTReg> );
    pcl::SampleConsensusPrerejective<PointTReg,PointTReg,pcl::FPFHSignature33> aligner;

    {
        pcl::ScopeTime t("Feature Alignment");
        pcl::FPFHEstimation<PointTReg, PointTNorm, pcl::FPFHSignature33> fpfhEstimator;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr clFpfhSrc(new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr clFpfhTgt(new pcl::PointCloud<pcl::FPFHSignature33>);

        //FPFHEstimation uses Normals so : Normals calculation
        pcl::PointCloud<PointTNorm>::Ptr cloudSrcNormal(new pcl::PointCloud<PointTNorm>);
        pcl::PointCloud<PointTNorm>::Ptr cloudTgtNormal(new pcl::PointCloud<PointTNorm>);
        pcl::NormalEstimation<PointTReg, PointTNorm> normEstimation;
        //set parameters
        normEstimation.setRadiusSearch(0.01);
        //compute the normals for Source cloud
        normEstimation.setInputCloud(clSrcXYZRGB);
        normEstimation.compute(*cloudSrcNormal);
        pcl::copyPointCloud(*clSrcXYZRGB, *cloudSrcNormal); // add the point cloud datas to the normals
        //compute the normals for Target cloud
        normEstimation.setInputCloud(clTgtXYZRGB);
        normEstimation.compute(*cloudTgtNormal);
        pcl::copyPointCloud(*clTgtXYZRGB, *cloudTgtNormal); // add the point cloud datas to the normals

        // Compute features Estimations
        pcl::search::KdTree<PointTReg>::Ptr kdtreeFpfh (new pcl::search::KdTree<PointTReg>);
        fpfhEstimator.setRadiusSearch(0.01);
        //For the source :
        fpfhEstimator.setInputCloud(clSrcXYZRGB);
        fpfhEstimator.setInputNormals(cloudSrcNormal);
        fpfhEstimator.compute(*clFpfhSrc);
        //For the target :
        fpfhEstimator.setInputCloud(clTgtXYZRGB);
        fpfhEstimator.setInputNormals(cloudTgtNormal);
        fpfhEstimator.compute(*clFpfhTgt);
        
        // Perform the first alignment
        aligner.setMaximumIterations(params.maxIter);
        aligner.setNumberOfSamples(params.nbSamples);
        aligner.setCorrespondenceRandomness(params.corrRand);
        aligner.setInlierFraction(params.inlierF);
        aligner.setSimilarityThreshold(params.simiThresh);
        aligner.setMaxCorrespondenceDistance(params.maxCorrDist);

        std::cout<<"\e[36mPerforming sample consensus with : "<<endl;
        std::cout<<"\t Inlier fraction: "<<aligner.getInlierFraction()<<endl;
        std::cout<<"\t number of samples: "<<aligner.getNumberOfSamples()<<endl;
        std::cout<<"\t Maximum iterations: "<<aligner.getMaximumIterations()<<endl;
        std::cout<<"\t Similarity threshold: "<<aligner.getSimilarityThreshold()<<endl;
        std::cout<<"\t Correspondence randomness: "<<aligner.getCorrespondenceRandomness()<<endl;
        std::cout<<"\t Max Correspondence distance: "<<aligner.getMaxCorrespondenceDistance();
        std::cout<<"\e[0m"<<endl;
        aligner.setInputSource(clSrcXYZRGB);
        aligner.setSourceFeatures(clFpfhSrc);
        aligner.setInputTarget(clTgtXYZRGB);
        aligner.setTargetFeatures(clFpfhTgt);
        aligner.align(*cloudfirst);
    }
    
    Eigen::Matrix4f transform = aligner.getFinalTransformation();
    return transform;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event){
    if(event.getKeySym() == "t" && event.keyDown())
        tryAgain = true;
    else if(event.getKeySym() == "f" && event.keyDown())
        finish = true;
}

int main (int argc, char** argv)
{
    pcl::PointCloud<PointTReg>::Ptr cloudSource (new pcl::PointCloud<PointTReg>);
    pcl::PointCloud<PointTReg>::Ptr cloudTarget (new pcl::PointCloud<PointTReg>);
    pcl::PointCloud<PointTReg>::Ptr cloudVoxe (new pcl::PointCloud<PointTReg>);
    // ******************************** Command line parser for arguments *************************************
    cv::CommandLineParser parser(argc, argv,
                             #if CV_MAJOR_VERSION == 3
                                 "{ h help       | false                    | [bool]   print this message.}"
                                 "{ s Source     | ../example/my_source.pcd | [string] Provide the PCD file of the point cloud to be load as Source.}"
                                 "{ t Target     | ../example/my_target.pcd | [string] Provide the PCD file of the point cloud to be load as Target.}"
                                 "{ l leaf       | 0.01,0.01,0.01           | [floats] Provide the leaf size of the Voxel algorithms (x,y,z).}"
                                 "{ cd corrdist  | 0.005                    | [float]  Provide the Max Correspondence distance.}"
                                 "{ f  fpfhactiv | true                     | [bool]   FPFH fitting: determine if the fitting will use a pre-alignment working by FPFH alignment.}"
                                 "{ fi fpfhinli  | 0.2                      | [float]  FPFH fitting: Inlier fraction [To explicit].}"
                                 "{ fm fpfhiter  | 25000                    | [int]    FPFH fitting: number of iterations [To explicit].}"
                                 "{ fn fpfhnbsp  | 3                        | [int]    FPFH fitting: number of samples [To explicit].}"
                                 "{ fs fpfhsimi  | 0.6                      | [float]  FPFH fitting: Similarity threshold [To explicit].}"
                                 "{ fc fpfhcord  | 2                        | [int]    FPFH fitting: Correspondence randomness [To explicit].}"
                                 "{ fd fpfhdist  | 0.1                      | [float]  FPFH fitting: Maximum Correspondence Distance [To explicit].}"
                             #else
                                 "{ h  | help      | false                  | [bool]   print this message.}"
                                 "{ s  | Source    | ../example/my_source.pcd [string] Provide the PCD file of the point cloud to be load as Source.}"
                                 "{ t  | Target    | ../example/my_target.pcd [string] Provide the PCD file of the point cloud to be load as Target.}"
                                 "{ l  | leaf      | 0.01,0.01,0.01         | [floats] Provide the leaf size of the Voxel algorithms (x,y,z).}"
                                 "{ cd | corrdist  | 0.005                  | [float]  Provide the Max Correspondence distance.}"
                                 "{ f  | fpfhactiv | true                   | [bool]   FPFH fitting: determine if the fitting will use a pre-alignment working by FPFH alignment.}"
                                 "{ fi | fpfhinli  | 0.2                    | [float]  FPFH fitting: Inlier fraction [To explicit].}"
                                 "{ fm | fpfhiter  | 25000                  | [int]    FPFH fitting: number of iterations [To explicit].}"
                                 "{ fn | fpfhnbsp  | 3                      | [int]    FPFH fitting: number of samples [To explicit].}"
                                 "{ fs | fpfhsimi  | 0.6                    | [float]  FPFH fitting: Similarity threshold [To explicit].}"
                                 "{ fc | fpfhcord  | 2                      | [int]    FPFH fitting: Correspondence randomness [To explicit].}"
                                 "{ fd | fpfhdist  | 0.1                    | [float]  FPFH fitting: Maximum Correspondence Distance [To explicit].}"
                             #endif
                                 );
    
    // ******************************** Get line parser arguments *************************************
    //print help
    bool helpReq = parser.get<bool>("h");
    if ( helpReq )
    {
#if CV_MAJOR_VERSION == 3
        parser.printMessage();
#else
        parser.printParams();
#endif
        return 0;
    }

    std::string fileName = parser.get<std::string>("s");
    std::cout<<"Loading Source : <"<<fileName<<">"<<endl;
    pcl::io::loadPCDFile (fileName, *cloudSource);

    fileName = parser.get<std::string>("t");
    std::cout<<"Loading Target : <"<<fileName<<">"<<endl;
    pcl::io::loadPCDFile (fileName, *cloudTarget);

    Eigen::Vector3f leafSize;
    std::string leafSizeStr = parser.get<std::string>("l");
    std::vector<std::string> fieldleafsize;
    boost::split(fieldleafsize, leafSizeStr, boost::is_any_of(","));
    for(int i = 0; i < fieldleafsize.size(); ++i)
        leafSize(i) = boost::lexical_cast<float>(fieldleafsize.at(i));

    float maxcd = parser.get<float>("cd");

    FPFHparameters fpfhPar;
    fpfhPar.active = parser.get<bool>("f");
    fpfhPar.maxIter = parser.get<int>("fm");
    fpfhPar.nbSamples = parser.get<int>("fn");
    fpfhPar.corrRand = parser.get<int>("fc");
    fpfhPar.inlierF = parser.get<float>("fi");
    fpfhPar.simiThresh = parser.get<float>("fs");
    fpfhPar.maxCorrDist = parser.get<float>("fd");

    //FPFH
    bool resu;
    int vpG1, vpG2;
    pcl::visualization::PCLVisualizer viewer("FPFH alignment");
    viewer.createViewPort(0., 0., 0.5, 1., vpG1);
    viewer.createViewPort(0.5, 0., 1., 1., vpG2);
    viewer.setBackgroundColor(.7, .7, .7);
    pcl::PointCloud<PointTReg>::Ptr cloudAligned(new pcl::PointCloud<PointTReg>);
    pcl::PointCloud<PointTReg>::Ptr cloudTemp(new pcl::PointCloud<PointTReg>);
    viewer.addText("FPFH Alignment results", 25, 25, "name");
    viewer.addText("Press t to try the alignment with current position, Press f to close the program", 10, 10, "command");
    viewer.registerKeyboardCallback(&keyboardEventOccurred);
    {
        pcl::ScopeTime t("First FPFH");
        pcl::transformPointCloud(*cloudSource, *cloudAligned, getFPFH(cloudSource, cloudTarget, fpfhPar, resu));
    }
    viewer.addPointCloud(cloudTarget, "Target1", vpG1);
    viewer.addPointCloud(cloudTarget, "Target2", vpG2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "Target1");
    viewer.addPointCloud(cloudAligned, "Aligned1", vpG1);
    viewer.addPointCloud(cloudAligned, "Aligned2", vpG2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, resu?0:1, resu?1:0, 0, "Aligned1");


    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        if(tryAgain){
            tryAgain = false;
            {
                pcl::ScopeTime t("New FPFH");
                pcl::transformPointCloud(*cloudSource, *cloudAligned, getFPFH(cloudSource, cloudTarget, fpfhPar, resu));
            }
            //update cloud viewer simple.
            pcl::copyPointCloud(*cloudAligned, *cloudSource);

            //update cloud viewer gicp6d
            viewer.updatePointCloud(cloudAligned, "Aligned1");
            viewer.updatePointCloud(cloudAligned, "Aligned2");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, resu?0:1, resu?1:0, 0, "Aligned1");

        }
        if(finish){
            finish = false;
            viewer.close();
            break;
        }
    }
    return 0;
}