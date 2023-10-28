//
// Created by dutrix on 10/28/23.
//

#ifndef STEREO_RECORDER_STEREOCAMERACALIBRATION_H
#define STEREO_RECORDER_STEREOCAMERACALIBRATION_H

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>

std::string MatType2str(int type);

struct SingleCameraCalibration {
    double fx;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double p1;
    double p2;

    double k3;


    //https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
    cv::Mat asIntrinsicMatrix() const {
        /*
         *     fx  0 cx
         * A = 0  fy cy
         *     0  0  1
         */

        double intrinsicMatrix[9] /*3x3*/ = {fx, 0, cx,
                                             0, fy, cy,
                                             0, 0, 1};
        return cv::Mat_<double>(3, 3, intrinsicMatrix).clone();
    }

    cv::Mat asDistortionCoefficients() const {
        double distortionCoefficients[5] = {k1, k2, p1, p2, k3};

        return cv::Mat_<double>(5, 1, distortionCoefficients).clone();

    }

    SingleCameraCalibration(double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2,
                            double k3)
            : fx(fx), fy(fy), cx(cx), cy(cy), k1(k1), k2(k2), p1(p1), p2(p2), k3(k3) {}

    SingleCameraCalibration() = default;

};

struct StereoRectifyResults {


    cv::Mat R1;
    cv::Mat R2;
    cv::Mat P1;
    cv::Mat P2;
    cv::Mat Q;
    cv::Size2i new_image_size;

    StereoRectifyResults() = default;

    StereoRectifyResults(const cv::Mat &r1, const cv::Mat &r2, const cv::Mat &p1, const cv::Mat &p2, const cv::Mat &q,
                         const cv::Size2i &newImageSize) : R1(r1), R2(r2), P1(p1), P2(p2), Q(q),
                                                           new_image_size(newImageSize) {}
};

//https://support.stereolabs.com/hc/en-us/articles/360007497173-What-is-the-calibration-file-
struct StereoCameraCalibrationData {
    cv::Size2i image_size;

    SingleCameraCalibration leftCam;
    cv::Mat undistortLeftX;
    cv::Mat undistortLeftY;

    SingleCameraCalibration rightCam;
    cv::Mat undistortRightX;
    cv::Mat undistortRightY;

    double baseline; //distance between cameras  (mm)
    double ty;
    double tz;

    //I HAVE NO IDEA WHAT THIS MEANS THIS CODE IS COPIED
    double cv; // cross product of vector
    double rx; //rodriguez notation
    double rz; //rodriguez notation




    StereoRectifyResults stereoRectification;

    StereoCameraCalibrationData(const cv::Size2i singleCameraImageSize,
                                const SingleCameraCalibration leftCam,
                                const SingleCameraCalibration rightCam,
                                double baseline,
                                double ty,
                                double tz,
                                double cv,
                                double rx,
                                double rz) : image_size(singleCameraImageSize), leftCam(leftCam), rightCam(rightCam),
                                             baseline(baseline), ty(ty), tz(tz), cv(cv), rx(rx), rz(rz),
                                             stereoRectification(stereoRectify()) {


        initUndistortRectifyMap(leftCam.asIntrinsicMatrix(),
                                leftCam.asDistortionCoefficients(),
                                stereoRectification.R1,
                                stereoRectification.P1,
                                stereoRectification.new_image_size, CV_32FC1,
                //out
                                undistortLeftX, undistortLeftY);


        initUndistortRectifyMap(rightCam.asIntrinsicMatrix(),
                                rightCam.asDistortionCoefficients(),
                                stereoRectification.R2,
                                stereoRectification.P2,
                                stereoRectification.new_image_size,
                                CV_32FC1,
                //out
                                undistortRightX,
                                undistortRightY);
    }

    StereoCameraCalibrationData() = default;

    ///IDK/IDC RODRIGUEZ NOTATION >????????<
    cv::Mat rotationAsVector() const {
        double r_zed[] = {rx, cv, rz};
        return cv::Mat_<double>(1, 3, r_zed).clone();
    }

//    I probably need this one
    cv::Mat rotationAsMatrix() const {
        cv::Mat R_zed, R;
        R_zed = rotationAsVector();
        cv::Rodrigues(R_zed /*in*/, R /*out*/);
        return R;
    }

    cv::Mat translationVector() const {
        double translationVector[] = {baseline, ty, tz};
        return cv::Mat_<double>(3, 1, translationVector).clone();
    }

private:
    StereoRectifyResults stereoRectify() {
        cv::Mat R1, R2, P1, P2, Q;
        cv::Size2i image_size_new;


        const cv::Mat lcMat = leftCam.asIntrinsicMatrix();
        const cv::Mat lcDist = leftCam.asDistortionCoefficients();
        const cv::Mat rcMat = rightCam.asIntrinsicMatrix();
        const cv::Mat rcDistCoeff = rightCam.asDistortionCoefficients();
        const cv::Mat rotationAsMatrix1 = rotationAsMatrix();
        const cv::Mat translationVector1 = translationVector();


        std::string types[] =
                {
                        MatType2str(lcMat.type()),
                        MatType2str(lcDist.type()),
                        MatType2str(rcMat.type()),
                        MatType2str(rcDistCoeff.type()),
                        MatType2str(rotationAsMatrix1.type()),
                        MatType2str(translationVector1.type())
                };

        cv::stereoRectify(lcMat,
                          lcDist,
                          rcMat,
                          rcDistCoeff,
                          image_size,
                          rotationAsMatrix1,
                          translationVector1,
                //out
                          R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY,
                          0, image_size);

        return StereoRectifyResults(R1, R2, P1, P2, Q, image_size);

    }

};


StereoCameraCalibrationData getCalibration(cv::Size2i image_size);

#endif //STEREO_RECORDER_STEREOCAMERACALIBRATION_H
