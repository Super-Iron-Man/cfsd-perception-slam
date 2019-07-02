#ifndef VISUAL_INERTIAL_SLAM_HPP
#define VISUAL_INERTIAL_SLAM_HPP

#include "cfsd/config.hpp"
#include "cfsd/camera-model.hpp"
#include "cfsd/loop-closure.hpp"
#include "cfsd/feature-tracker.hpp"
#include "cfsd/imu-preintegrator.hpp"
#include "cfsd/optimizer.hpp"
#include "cfsd/map.hpp"

#ifdef CLUON
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#endif

#ifdef USE_VIEWER
#include "cfsd/viewer.hpp"
#endif

#include <fstream>

namespace cfsd {

enum SensorType {
  ACCELEROMETER,
  GYROSCOPE
};

class VisualInertialSLAM {
  public:
    // visual inertial odometry state
    enum VIOstate {
        SYNCHRONIZING, // matching imu and camera timestamp
        SFM, // struct from motion
        INITIALIZING, // initialize imu
        OK, // process
        LOST // caused by lose track or corrupted bias, need re-initialization
    };

  public:
    VisualInertialSLAM(const bool verbose);

    #ifdef CLUON
    VisualInertialSLAM(cluon::OD4Session* od4, const bool verbose);
    #endif

    #ifdef USE_VIEWER
    void setViewer(const cfsd::Ptr<Viewer>& pViewer) { _pMap->_pViewer = pViewer; }
    #endif

    bool process(const cv::Mat& grayL, const cv::Mat& grayR, const long& imgTimestamp);

    void collectImuData(const cfsd::SensorType& st, const long& timestamp, const float& x, const float& y, const float& z);

    void saveResults();

    #ifdef CLUON
    void od4sendGroundSpeed();
    #endif

    #ifdef SHOW_IMG
    void showImage(cv::Mat& imgL, const double& dt);
    #endif

  private:
    VIOstate _state;
    
    const bool _verbose;

    #ifdef CLUON
    cluon::OD4Session* _od4{};
    #endif

    cfsd::Ptr<CameraModel> _pCameraModel;

    cfsd::Ptr<Map> _pMap;

    cfsd::Ptr<LoopClosure> _pLoopClosure;

    cfsd::Ptr<FeatureTracker> _pFeatureTracker;

    cfsd::Ptr<Optimizer> _pOptimizer;

    cfsd::Ptr<ImuPreintegrator> _pImuPreintegrator;

    Eigen::Vector3d _gyr{};
    bool _gyrGot{false};
    
    Eigen::Vector3d _acc{};
    bool _accGot{false};

    int _numNoMatch{0};

    int _sfmCount{0};

    std::thread _loopThread{};

    #ifdef USE_VIEWER
    cfsd::Ptr<Viewer> _pViewer{};
    std::thread _viewerThread{};
    #endif
};

} // namespace cfsd

#endif // VISUAL_INERTIAL_SLAM_HPP