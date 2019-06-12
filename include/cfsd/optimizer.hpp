#ifndef OPTIMIZER_HPP
#define OPTIMIZER_HPP

#include "cfsd/cost-functions.hpp"
#include "cfsd/config.hpp"

namespace cfsd {

class Optimizer {
  public:
    Optimizer(const cfsd::Ptr<Map>& _pMap, const cfsd::Ptr<FeatureTracker>& pFeatureTracker, const cfsd::Ptr<ImuPreintegrator>& pImuPreintegrator, const cfsd::Ptr<CameraModel>& pCameraModel, const bool verbose);

    // ~Optimizer();

    /* Only optimize motion (i.e. vehicle states), keep landmarks fixed.
       map points: (x x   x  x  x x)  <- fixed
                   /| |\ /|\ | /| |\
           frames: #  #  #  #  #  #  #  $    (# is keyframe, $ is latest frame)
                   | <=fixed=> |  | <=> | <- local-window to be optimizer
    */
    void motionOnlyBA();

    // Estimate initial gyroscope bias, using rotation residuals.
    void initialGyrBias();

    // Eisimate initial gravity direction in body frame and body velocity, using velocity and position residuals.
    void initialGravityVelocity();
    
    // Align the gravity in body coordinate to world coordinate, and refine the magnitude.
    void initialAlignment();

    // Estimate initial accelerometer bias, using velocity and position residuals.
    void initialAccBias();

    void loopCorrection(const int& curFrameID);

    void fullBA();

    // bool linearizeReprojection(const size_t& mapPointID, const int& startFrameID, std::vector<double*>& delta_pose_img, int& errorTerms, Eigen::VectorXd& error, Eigen::MatrixXd F);

  private:
    cfsd::Ptr<Map> _pMap;

    cfsd::Ptr<FeatureTracker> _pFeatureTracker;

    cfsd::Ptr<ImuPreintegrator> _pImuPreintegrator;

    const cfsd::Ptr<CameraModel>& _pCameraModel;

    const bool _verbose;

    double _pose[WINDOWSIZE][6];  // pose (rotation vector, translation vector / position)
    double _v_bga[WINDOWSIZE][9]; // velocity, bias of gyroscope, bias of accelerometer

    // Camera intrinsics.
    double _fx{0}, _fy{0}, _cx{0}, _cy{0};
    
    // Inverse of pixel standard deviation.
    Eigen::Matrix2d _invStdT{};
    
    // TODO: marginalization
    // double _priorWeight{0.0};

    // Ceres solver settings.
    bool _minimizerProgressToStdout{true};
    int _maxNumIterations{0};
    double _maxSolverTimeInSeconds{0.0};
    int _numThreads{0};
    bool _checkGradients{false};

    // std::mutex _loopMutex{};
};

} // namespace cfsd

#endif // OPTIMIZER_HPP