//
// Created by dutrix on 10/28/23.
//

#ifndef STEREO_RECORDER_DEPTHMATCHER_H
#define STEREO_RECORDER_DEPTHMATCHER_H

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include "StereoCameraCalibration.h"

struct DepthMatcherParams
{
public:
    DepthMatcherParams()
    {
        blockSize = 3;
        minDisparity = 0;
        numDisparities = 96;
        mode = cv::StereoSGBM::MODE_SGBM; // MODE_SGBM = 0, MODE_HH   = 1, MODE_SGBM_3WAY = 2, MODE_HH4  = 3
        P1 = 24 * blockSize * blockSize;
        P2 = 4 * P1;
        disp12MaxDiff = 96;
        preFilterCap = 63;
        uniquenessRatio = 5;
        speckleWindowSize = 255;
        speckleRange = 1;

        minDepth_mm = 300.f;
        maxDepth_mm = 10000.f;
    }

public:
    int blockSize;         //!< [default: 3] Matched block size. It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
    int minDisparity;      //!< [default: 0] Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
    int numDisparities;    //!< [default: 96] Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16.
    int mode;              //!< Set it to StereoSGBM::MODE_HH to run the full-scale two-pass dynamic programming algorithm. It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures. By default, it is set to `cv::StereoSGBM::MODE_SGBM_3WAY`.
    int P1;                //!< [default: 24*blockSize*blockSize] The first parameter controlling the disparity smoothness. See below.
    int P2;                //!< [default: 4*PI]The second parameter controlling the disparity smoothness. The larger the values are, the smoother the disparity is. P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels. The algorithm requires P2 > P1 . See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown (like 8*number_of_image_channels*blockSize*blockSize and 32*number_of_image_channels*blockSize*blockSize , respectively).
    int disp12MaxDiff;     //!< [default: 96] Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check.
    int preFilterCap;      //!< [default: 63] Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function.
    int uniquenessRatio;   //!< [default: 5] Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
    int speckleWindowSize; //!< [default: 255] Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
    int speckleRange;      //!< [default: 1] Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.

    float minDepth_mm; //!< [default: 300] Minimum value of depth for the extracted depth map
    float maxDepth_mm; //!< [default: 10000] Maximum value of depth for the extracted depth map
};

class DepthMatcher
{
public:
    DepthMatcher() : DepthMatcher(DepthMatcherParams()) {}

    DepthMatcher(DepthMatcherParams depthMatcherSettings) : depthMatcherSettings(depthMatcherSettings),
                                                            stereoMatcher(cv::StereoSGBM::create())
    {

        auto stereoPar = depthMatcherSettings;
        this->stereoMatcher->setMinDisparity(stereoPar.minDisparity);
        this->stereoMatcher->setNumDisparities(stereoPar.numDisparities);
        this->stereoMatcher->setBlockSize(stereoPar.blockSize);
        this->stereoMatcher->setP1(stereoPar.P1);
        this->stereoMatcher->setP2(stereoPar.P2);
        this->stereoMatcher->setDisp12MaxDiff(stereoPar.disp12MaxDiff);
        this->stereoMatcher->setMode(stereoPar.mode);
        this->stereoMatcher->setPreFilterCap(stereoPar.preFilterCap);
        this->stereoMatcher->setUniquenessRatio(stereoPar.uniquenessRatio);
        this->stereoMatcher->setSpeckleWindowSize(stereoPar.speckleWindowSize);
        this->stereoMatcher->setSpeckleRange(stereoPar.speckleRange);
    }

    //
    //    cv::UMat calculateDisparityHalfSize(const cv::UMat &left, const cv::UMat &right) {
    //        const double resize_mul = 0.5;
    //        cv::UMat left_resized, right_resized, disparity;
    //
    //        cv::resize(left, left_resized, cv::Size(), resize_mul, resize_mul, cv::INTER_AREA);
    //        cv::resize(right, right_resized, cv::Size(), resize_mul, resize_mul, cv::INTER_AREA);
    //
    //
    //        disparity = calculateDisparity(left_resized, right_resized);
    //
    //        cv::multiply(disparity, 2., disparity); // Last 4 bits of SGBM disparity are decimal
    //        cv::resize(disparity, disparity, cv::Size(), 1. / resize_mul, 1. / resize_mul, cv::INTER_AREA);
    //
    //        return disparity;
    //    }

    // left + right should be rectified
    void calculateDisparity(cv::InputArray left, cv::InputArray right, cv::OutputArray disparity)
    {
        cv::UMat disparityFixedpoint, disparity_float, left_dist;

        // Apply stereo matching
        stereoMatcher->compute(left, right, disparityFixedpoint);

        disparityFixedpoint.convertTo(disparity_float, CV_32FC1);
        cv::multiply(disparity_float, 1.f / 16.f, disparity); // convert fixed point to float
    }

    void
    createPointCloud(cv::InputArray disparity, StereoCameraCalibrationData &calibration, cv::OutputArray pointCloud)
    {

        cv::UMat outputImage;
        cv::reprojectImageTo3D(disparity, pointCloud, calibration.stereoRectification.Q, true, -1);
    }

    int getMinDisparity() const { return depthMatcherSettings.minDisparity; }

    int getNumDisparities() const { return depthMatcherSettings.numDisparities; }

private:
    DepthMatcherParams depthMatcherSettings;
    cv::Ptr<cv::StereoSGBM> stereoMatcher;
};

#endif // STEREO_RECORDER_DEPTHMATCHER_H
