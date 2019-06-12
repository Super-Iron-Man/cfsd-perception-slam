#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "cfsd/common.hpp"

namespace cfsd {

struct ImuConstraint {
    ImuConstraint() {}
    
    ImuConstraint(const Eigen::Matrix<double,15,15>& invCovPreintegration_ij_, const Eigen::Vector3d& bg_i_, const Eigen::Vector3d& ba_i_,
                  const Sophus::SO3d& delta_R_ij_, const Eigen::Vector3d& delta_v_ij_, const Eigen::Vector3d& delta_p_ij_,
                  const Eigen::Matrix3d& d_R_bg_ij_, const Eigen::Matrix3d& d_v_bg_ij_, const Eigen::Matrix3d& d_v_ba_ij_, 
                  const Eigen::Matrix3d& d_p_bg_ij_, const Eigen::Matrix3d& d_p_ba_ij_, const double& dt_) : 
        invCovPreintegration_ij(invCovPreintegration_ij_), bg_i(bg_i_), ba_i(ba_i_), delta_R_ij(delta_R_ij_), delta_v_ij(delta_v_ij_), delta_p_ij(delta_p_ij_),
        d_R_bg_ij(d_R_bg_ij_), d_v_bg_ij(d_v_bg_ij_), d_v_ba_ij(d_v_ba_ij_), d_p_bg_ij(d_p_bg_ij_), d_p_ba_ij(d_p_ba_ij_), dt(dt_) { dt2 = dt * dt; }

    // Inverse of covariance matrix of preintegrated noise [delta_rvec, delta_v, delta_p, delta_bg, delta_ba]
    Eigen::Matrix<double,15,15> invCovPreintegration_ij{};

    // Bias of gyroscope and accelerometer at time i.
    Eigen::Vector3d bg_i{};
    Eigen::Vector3d ba_i{};
    
    // Preintegrated delta_R, delta_v, delta_p.
    Sophus::SO3d delta_R_ij{};
    Eigen::Vector3d delta_v_ij{};
    Eigen::Vector3d delta_p_ij{};

    // Partial derivative of R, v, p with respect to bias of gyr and acc (denoated as bg and ba).
    Eigen::Matrix3d d_R_bg_ij{};
    Eigen::Matrix3d d_v_bg_ij{};
    Eigen::Matrix3d d_v_ba_ij{};
    Eigen::Matrix3d d_p_bg_ij{};
    Eigen::Matrix3d d_p_ba_ij{};

    // The time between two camera frames, dt2 = dt^2.
    double dt{0}, dt2{0};
};

struct Feature {
    Feature() {}

    Feature(const int& frameID_, const cv::Point2d& pixelL_, const cv::KeyPoint& keypointL_, const cv::KeyPoint& keypointR_, const cv::Mat& descriptorL_, const cv::Mat& descriptorR_, const int& age_)
      : frameID(frameID_), pixelL(pixelL_), keypointL(keypointL_), keypointR(keypointR_), descriptorL(descriptorL_), descriptorR(descriptorR_), age(age_) {}

    Feature(const int& frameID_, const cv::Point2d& pixelL_, const cv::KeyPoint& keypointL_, const cv::Mat& descriptorL_, const int& age_)
      : frameID(frameID_), pixelL(pixelL_), keypointL(keypointL_), descriptorL(descriptorL_), age(age_) {}
    
    int frameID{0};

    cv::Point2d pixelL{};

    cv::KeyPoint keypointL{};
    cv::KeyPoint keypointR{};
    cv::Mat descriptorL{};
    cv::Mat descriptorR{};

    int age{0};
};

struct MapPoint {
    MapPoint() {}

    MapPoint(const Eigen::Vector3d& position_, const int& frameID_, const cv::Point2d& pixel_) : position(position_) {
        addPixel(frameID_, pixel_);
    }

    void addPixel(const int& frameID, const cv::Point2d& pixel) {
        pixels[frameID] = pixel;
    }

    // 3D position w.r.t world coordinate.
    Eigen::Vector3d position{};

    // frameID and pixel coordinate in that frame.
    std::map<int, cv::Point2d> pixels{};
};

struct Keyframe {
    Keyframe() : R(Eigen::Matrix3d::Identity()), p(Eigen::Vector3d::Zero()), v(Eigen::Vector3d::Zero()), dbg(Eigen::Vector3d::Zero()), dba(Eigen::Vector3d::Zero()) {}

    Sophus::SO3d R;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d dbg;
    Eigen::Vector3d dba;
    cfsd::Ptr<ImuConstraint> pImuConstraint{};
    std::vector<size_t> mapPointIDs{};
    cv::Mat descriptors{};
    long timestamp{0};
};

struct LoopInfo {
    LoopInfo() {}
    
    LoopInfo(const int& loopFrameID_, const Sophus::SO3d& R_, const Eigen::Vector3d& p_) : loopFrameID(loopFrameID_), R(R_), p(p_) {}

    int loopFrameID{-1};
    // R and p are transform from loop frame to current frame.
    Sophus::SO3d R{};
    Eigen::Vector3d p{};
};

} // namespace cfsd

#endif // STRUCTS_HPP
