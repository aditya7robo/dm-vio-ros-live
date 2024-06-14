#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include <locale.h>
#include <stdlib.h>
#include <stdio.h>

#include "IOWrapper/Output3DWrapper.h"

#include "util/Undistort.h"

#include <boost/thread.hpp>
#include "dso/util/settings.h"
#include "dso/util/globalCalib.h"
#include "util/TimeMeasurement.h"

#include "FullSystem/FullSystem.h"

#include <util/SettingsUtil.h>

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"

#include "util/MainSettings.h"
#include "live/FrameSkippingStrategy.h"
#include "live/IMUInterpolator.h"
#include "ROSOutputWrapper.h"

#include <live/FrameContainer.h>

#ifdef STACKTRACE
#include <boost/stacktrace.hpp>
#include <signal.h>
#endif

using namespace dso;
using namespace cv; 

#ifdef STACKTRACE
void segfault_handler(int sig)
{
    std::cerr << "Caught signal " << sig << ", printing stacktrace:" << std::endl;
    std::cerr << boost::stacktrace::stacktrace() << std::endl;
    exit(1);
}
#endif

dmvio::FrameContainer frameContainer;
dmvio::IMUInterpolator imuInt(frameContainer, nullptr);
dmvio::MainSettings mainSettings;
dmvio::IMUCalibration imuCalibration;
dmvio::IMUSettings imuSettings;
dmvio::FrameSkippingSettings frameSkippingSettings;
std::unique_ptr<Undistort> undistorter;
bool stopSystem = false;
int start = 2;
double timeshift = 0.0;
std::string imageTopic, imuTopic;

static const std::string OPENCV_WINDOW = "Image window";

void vidCb(const sensor_msgs::ImageConstPtr img);
void imuCb(const sensor_msgs::ImuConstPtr imu);

void run(IOWrap::PangolinDSOViewer* viewer)
{
    bool linearizeOperation = false;

    auto fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);

    if(setting_photometricCalibration > 0 && undistorter->photometricUndist == nullptr)
    {
        printf("ERROR: dont't have photometric calibration. Need to use commandline options mode=1 or mode=2 ");
        exit(1);
    }

    if(undistorter->photometricUndist != nullptr)
    {
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
    }

    if(viewer)
    {
        fullSystem->outputWrapper.push_back(viewer);
    }

    dmvio::FrameSkippingStrategy frameSkipping(frameSkippingSettings);
    fullSystem->outputWrapper.push_back(&frameSkipping);

    dmvio::ROSOutputWrapper rosOutput;
    fullSystem->outputWrapper.push_back(&rosOutput);

    int ii = 0;
    int lastResetIndex = 0;

    while(!stopSystem)
    {
        if(start > 0 && ii < start)
        {
            auto pair = frameContainer.getImageAndIMUData(0);
            ++ii;
            continue;
        }

        int numSkipFrames = frameSkipping.getMaxSkipFrames(frameContainer.getQueueSize());
        auto pair = frameContainer.getImageAndIMUData(numSkipFrames);

        if(!pair.first) continue;

        fullSystem->addActiveFrame(pair.first.get(), ii, &(pair.second), nullptr);

        if(fullSystem->initFailed || setting_fullResetRequested)
        {
            if(ii - lastResetIndex < 250 || setting_fullResetRequested)
            {
                printf("RESETTING!\n");
                std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
                fullSystem.reset();
                for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();

                fullSystem = std::make_unique<FullSystem>(linearizeOperation, imuCalibration, imuSettings);
                if(undistorter->photometricUndist != nullptr)
                {
                    fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
                }
                fullSystem->outputWrapper = wraps;

                setting_fullResetRequested = false;
                lastResetIndex = ii;
            }
        }

        if(viewer != nullptr && viewer->shouldQuit())
        {
            std::cout << "User closed window -> Quit!" << std::endl;
            break;
        }

        if(fullSystem->isLost)
        {
            printf("LOST!!\n");
            break;
        }

        ++ii;
    }

    fullSystem->blockUntilMappingIsFinished();
    fullSystem->printResult(imuSettings.resultsPrefix + "result.txt", false, false, true);
    fullSystem->printResult(imuSettings.resultsPrefix + "resultScaled.txt", false, true, true);

    dmvio::TimeMeasurement::saveResults(imuSettings.resultsPrefix + "timings.txt");

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
    }

    printf("DELETE FULLSYSTEM!\n");
    fullSystem.reset();

    ros::shutdown();

    printf("EXIT NOW!\n");
}

double convertStamp(const ros::Time& time)
{
    return time.sec * 1.0 + time.nsec / 1000000000.0;
}

void vidCb(const sensor_msgs::ImageConstPtr img)
{
    ROS_INFO("[dmvio_live] Video Callback (1).");
    double stamp = convertStamp(img->header.stamp) + timeshift;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    assert(cv_ptr->image.type() == CV_8U);
    assert(cv_ptr->image.channels() == 1);

    ROS_INFO("[dmvio_live] Video Callback (2).");

    // cv::namedWindow(OPENCV_WINDOW, WINDOW_AUTOSIZE);
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);

    MinimalImageB minImg((int) cv_ptr->image.cols, (int) cv_ptr->image.rows, (unsigned char*) cv_ptr->image.data);

    std::unique_ptr<ImageAndExposure> undistImg(undistorter->undistort<unsigned char>(&minImg, 1.0, stamp, 1.0f));

    imuInt.addImage(std::move(undistImg), stamp);
    ROS_INFO("[dmvio_live] Video Callback (3).");
}

void imuCb(const sensor_msgs::ImuConstPtr imu)
{
    ROS_INFO("[dmvio_live] IMU Callback.");
    std::vector<float> accData;
    accData.push_back(imu->linear_acceleration.x);
    accData.push_back(imu->linear_acceleration.y);
    accData.push_back(imu->linear_acceleration.z);

    std::vector<float> gyrData;
    gyrData.push_back(imu->angular_velocity.x);
    gyrData.push_back(imu->angular_velocity.y);
    gyrData.push_back(imu->angular_velocity.z);

    ros::Time time = imu->header.stamp;
    double timestamp = convertStamp(time);
    imuInt.addAccData(accData, timestamp);
    imuInt.addGyrData(gyrData, timestamp);
}

int main(int argc, char** argv)
{
#ifdef STACKTRACE
    struct sigaction sa;
    sa.sa_handler = segfault_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART | SA_SIGINFO;
    sigaction(SIGSEGV, &sa, NULL);
#endif

    ros::init(argc, argv, "DMVIO_ros");
    ros::NodeHandle nh;

    setlocale(LC_ALL, "C");

#ifdef DEBUG
    std::cout << "DEBUG MODE!" << std::endl;
#endif

    auto settingsUtil = std::make_shared<dmvio::SettingsUtil>();

    imuSettings.registerArgs(*settingsUtil);
    imuCalibration.registerArgs(*settingsUtil);
    mainSettings.registerArgs(*settingsUtil);
    frameSkippingSettings.registerArgs(*settingsUtil);

    settingsUtil->registerArg("start", start);

    auto normalizeCamSize = std::make_shared<double>(0.0);
    settingsUtil->registerArg("normalizeCamSize", *normalizeCamSize, 0.0, 5.0);
    settingsUtil->registerArg("timeshift", timeshift);

    mainSettings.parseArguments(argc, argv, *settingsUtil);

    std::cout << "Settings:\n";
    settingsUtil->printAllSettings(std::cout);
    {
        std::ofstream settingsStream;
        settingsStream.open(imuSettings.resultsPrefix + "usedSettingsdso.txt");
        settingsUtil->printAllSettings(settingsStream);
    }

    undistorter.reset(
            Undistort::getUndistorterForFile(mainSettings.calib, mainSettings.gammaCalib, mainSettings.vignette));

    setGlobalCalib(
            (int) undistorter->getSize()[0],
            (int) undistorter->getSize()[1],
            undistorter->getK().cast<float>());

    imuCalibration.loadFromFile(mainSettings.imuCalibFile);

    std::unique_ptr<IOWrap::PangolinDSOViewer> viewer;

    if(!disableAllDisplay)
    {
        viewer = std::make_unique<IOWrap::PangolinDSOViewer>(wG[0], hG[0], true, settingsUtil, normalizeCamSize);
    }

    imageTopic = nh.resolveName("/mono_image");
    imuTopic = nh.resolveName("imu0");
    std::cout << "Image topic: " << imageTopic << std::endl;
    std::cout << "IMU topic: " << imuTopic << std::endl;

    std::thread runThread = std::thread(boost::bind(run, viewer.get()));

    ros::Subscriber imageSub = nh.subscribe("/mono_image", 3, &vidCb);
    ros::Subscriber imuSub = nh.subscribe("/imu0", 50, &imuCb);

    ros::spin();
    stopSystem = true;
    frameContainer.stop();

    runThread.join();

    return 0;
}


/*     _____         _      ____          _      
      |_   _|__  ___| |_   / ___|___   __| | ___ 
        | |/ _ \/ __| __| | |   / _ \ / _` |/ _ \
        | |  __/\__ \ |_  | |__| (_) | (_| |  __/
        |_|\___||___/\__|  \____\___/ \__,_|\___|                                           
*/

// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

// static const std::string OPENCV_WINDOW = "Image window";

// class ImageConverter
// {
//   ros::NodeHandle nh_;
//   image_transport::ImageTransport it_;
//   image_transport::Subscriber image_sub_;
//   image_transport::Publisher image_pub_;

// public:
//   ImageConverter()
//     : it_(nh_)
//   {
//     ROS_INFO("[opencv_ex] ImageConverter - 1.");
//     // Subscrive to input video feed and publish output video feed
//     image_sub_ = it_.subscribe("/mono_image", 1,
//       &ImageConverter::imageCb, this);
//     image_pub_ = it_.advertise("/image_converter/output_video", 1);

//     cv::namedWindow(OPENCV_WINDOW);
//     ROS_INFO("[opencv_ex] ImageConverter - 2.");
//   }

//   ~ImageConverter()
//   {
//     ROS_INFO("[opencv_ex] Destructor - 1.");
//     cv::destroyWindow(OPENCV_WINDOW);
//   }

//   void imageCb(const sensor_msgs::ImageConstPtr& msg)
//   {
//     ROS_INFO("[opencv_ex] imageCb - 1.");
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
//     }
//     catch (cv_bridge::Exception& e)
//     {
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//     }

//     ROS_INFO("[opencv_ex] imageCb - 2.");

//     // Draw an example circle on the video stream
//     if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//       cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

//     // Update GUI Window
//     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//     cv::waitKey(3);

//     // Output modified video stream
//     image_pub_.publish(cv_ptr->toImageMsg());
//     ROS_INFO("[opencv_ex] imageCb - 3.");
//   }
// };

// int main(int argc, char** argv)
// {
//   ROS_INFO("[opencv_ex] Main - 1.");
//   ros::init(argc, argv, "image_converter");
//   ImageConverter ic;
//   ros::spin();
//   ROS_INFO("[opencv_ex] Main - 2.");
//   return 0;
// }
