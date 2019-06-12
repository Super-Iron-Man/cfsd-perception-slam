#ifndef FEATURE_TRACKER_HPP
#define FEATURE_TRACKER_HPP

#include "cfsd/common.hpp"
#include "cfsd/camera-model.hpp"
#include "cfsd/structs.hpp"
#include "cfsd/map.hpp"
#include "thirdparty/ORBextractor.h"
#include "thirdparty/gms_matcher.h"

#ifdef SHOW_IMG
#include <opencv2/highgui/highgui.hpp>
#endif

namespace cfsd {

class FeatureTracker {
  public:
    FeatureTracker(const cfsd::Ptr<Map>& pMap, const cfsd::Ptr<CameraModel>& pCameraModel, const bool verbose);

    // Diasble copy constructor.
    FeatureTracker(const FeatureTracker&) = delete;

    // Disable copy assignment constructor.
    FeatureTracker& operator=(const FeatureTracker&) = delete;

    // Grid-based detection using OpenCV's ORB.
    void orbDetectWithGrid(int flag, const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

    // Use ORB_SLAM2's ORB.
    void extractORB(int flag, const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

    // Feature matching and tracking, including:
    // - internal match
    // - external track
    // - filter matches (improve the quality of matching)
    bool processImage(const cv::Mat& imgLeft, const cv::Mat& imgRight, cv::Mat& descriptorsMat);

    // Match left image features and right image features in current frame.
    void internalMatch(const cv::Mat& imgLeft, const cv::Mat& imgRight, cv::Mat& descriptorsMat);

    // Match current features and past features.
    void externalTrack();

    void processMatching(const cv::DMatch& m, std::unordered_map<size_t, bool>& uniqueFeature, std::vector<size_t>& mapPointIDs, cv::Mat& descriptors);

    // Maintain the feature pool or local map.
    void featurePoolUpdate(const long& imgTimestamp);

    // SfM: use RANSAC scheme for outlier rejection, and solve 3D-2D PnP problem (in particular, P3P problem).
    // This is used in initialization.
    bool structFromMotion(const cv::Mat& grayLeft, const cv::Mat& grayRight, Eigen::Vector3d& r, Eigen::Vector3d& p, const bool atBeginning = false);

  private:
    cfsd::Ptr<Map> _pMap;

    // Pinhole camera Model.
    const cfsd::Ptr<CameraModel>& _pCameraModel;

    const bool _verbose;

    // For CFSD, only part of the image is considered to be useful.
    // (e.g. the upper half of the image containing sky contributes little to useful features)
    cv::Rect _roi{};

    // Used in GMS matcher.
    cv::Size _imgSize{};

    int _frameID{0};
    size_t _featureID{0};

    // Detector to be used.
    bool _cvORB{false};
    cv::Ptr<cv::ORB> _orbLeft{};
    cv::Ptr<cv::ORB> _orbRight{};

    ORB_SLAM2::ORBextractor* _ORBextractorLeft{};
    ORB_SLAM2::ORBextractor* _ORBextractorRight{};

    // Current features matched with history features.
    // std::vector<cv::Point2d> _matchedHistPixelsL, _matchedHistPixelsR;
    // std::vector<cv::Point2d> _matchedCurPixelsL, _matchedCurPixelsR;
    
    // Match distance should be less than max(_matchRatio*minDist, _minMatchDist)
    // Ratio for selecting good matches.
    float _matchRatio{0};
    // Min match distance, based on experience, e.g. 30.0f
    float _minMatchDist{0};
    // For matched pixel (ul, vl) and (ur, vr), |vl-vr| should be small enough if the image has been rectified.
    float _maxVerticalPixelDist{0};

    // Features that stay in the pool for too long time should be be removed.
    int _maxFeatureAge{0};

    // Triangulated depth, if the 3D point is too far it is less accurate, so should not be added to pool.
    double _maxDepth{0};

    // Current frame's keypoints' pixel position and descriptors.
    std::vector<cv::Point2d> _curPixelsL{};
    std::vector<cv::KeyPoint> _curKeypointsL{};
    cv::Mat _curDescriptorsL{};
    // The information of current right frame is used for triangulation.
    std::vector<cv::Point2d> _curPixelsR{};
    std::vector<cv::KeyPoint> _curKeypointsR{};
    cv::Mat _curDescriptorsR{};

    // Record which features in current frame will possibly be viewed as new features, if circular matching is satisfied, it will be false; otherwise, true.
    std::vector<bool> _curFeatureMask{};

    // It's possible circular matching being too strict that few matches are left for further computing.
    bool _useCircularMatch{true};

    // History features' id and descriptors.
    std::vector<size_t> _histFeatureIDs{};
    std::vector<cv::KeyPoint> _histKeypointsL{};
    cv::Mat _histDescriptorsL{};
    // The history information of right frames is used for circular matching.
    std::vector<cv::KeyPoint> _histKeypointsR{};
    cv::Mat _histDescriptorsR{};

    // Used in initial SfM, previous frame's keypoints and descriptors in left image.
    std::vector<cv::KeyPoint> _refKeypointsL{};
    cv::Mat _refDescriptorsL{};
    
    // Pick one of the methods provided by OpenCV, see config file for detail.
    int _solvePnP{0};

    // Used in initial SfM, minimum rotation and translation for picking solvePnP results.
    double _minRotation{0};
    double _minTranslation{0};

  public:
    // Features that pass circular matching, i.e. curLeft <=> histLeft <=> histRight <=> curRight <=> curLeft
    // store the id of the circularly matched features, s.t. the _age grows normally, i.e., increase 1.
    // also store the id of the matches (either left or right side) but not circularly matched features, s.t. the _age will grow more than 1 as penalty.
    // for those not matched features, the _age will grow much more as penalty.
    std::vector<size_t> _matchedFeatureIDs{};

    // Available features from history frames.
    // - new features would be added
    // - matched features' _age will be updated
    // - old features that are not useful anymore would be removed
    // so std::map container is choosed due to the efficient access, insert and erase operation.
    std::unordered_map<size_t, cfsd::Ptr<Feature>> _pFeatures{};

    // If the image is cropped, the pixel coordinate would be different with the uncropped ones.
    int _cropOffset{0};
};

} // namespace cfsd

#endif // FEATURE_TRACKER_HPP