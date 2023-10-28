//
// Created by dutrix on 10/28/23.
//
#include "StereoCameraCalibration.h"


StereoCameraCalibrationData calibrationData2K(cv::Size2i image_size) {
    auto left_fx = 1400.23;
    auto left_fy = 1400.23;
    auto left_cx = 1131.24;
    auto left_cy = 623.73;
    auto left_k1 = -0.167491;
    auto left_k2 = 0.0207842;
    auto left_p1 = 0;
    auto left_p2 = 0;
    auto left_k3 = 0;

    auto right_fx = 1399.91;
    auto right_fy = 1399.91;
    auto right_cx = 1121.78;
    auto right_cy = 651.336;
    auto right_k1 = -0.170864;
    auto right_k2 = 0.0245901;
    auto right_p1 = 0;
    auto right_p2 = 0;
    auto right_k3 = 0;

    const SingleCameraCalibration leftCam(
            left_fx,
            left_fy,
            left_cx,
            left_cy,
            left_k1,
            left_k2,
            left_p1,
            left_p2,
            left_k3
    );

    const SingleCameraCalibration rightCam(
            right_fx,
            right_fy,
            right_cx,
            right_cy,
            right_k1,
            right_k2,
            right_p1,
            right_p2,
            right_k3
    );

    return StereoCameraCalibrationData(image_size, leftCam, rightCam,
                                       120,
                                       0,
                                       0,

                                       0.0126552,
                                       0.00543845,
                                       -4.02686e-05
    );
}


StereoCameraCalibrationData calibrationDataVGA(cv::Size2i image_size) {

    auto left_fx = 350.0575;
    auto left_fy = 350.0575;
    auto left_cx = 341.56;
    auto left_cy = 187.4325;
    auto left_k1 = -0.167491;
    auto left_k2 = 0.0207842;
    auto left_p1 = 0;
    auto left_p2 = 0;
    auto left_k3 = 0;

    auto right_fx = 349.9775;
    auto right_fy = 349.9775;
    auto right_cx = 339.195;
    auto right_cy = 194.334;
    auto right_k1 = -0.170864;
    auto right_k2 = 0.0245901;
    auto right_p1 = 0;
    auto right_p2 = 0;
    auto right_k3 = 0;


    const SingleCameraCalibration leftCam(
            left_fx,
            left_fy,
            left_cx,
            left_cy,
            left_k1,
            left_k2,
            left_p1,
            left_p2,
            left_k3
    );

    const SingleCameraCalibration rightCam(
            right_fx,
            right_fy,
            right_cx,
            right_cy,
            right_k1,
            right_k2,
            right_p1,
            right_p2,
            right_k3
    );

    return StereoCameraCalibrationData(image_size, leftCam, rightCam,
                                       120,
                                       0,
                                       0,

                                       0.0126552,
                                       0.00543845,
                                       -4.02686e-05

    );
}


StereoCameraCalibrationData calibrationDataFHD(cv::Size2i image_size) {


    auto left_fx = 1400.23;
    auto left_fy = 1400.23;
    auto left_cx = 987.24;
    auto left_cy = 542.73;
    auto left_k1 = -0.167491;
    auto left_k2 = 0.0207842;
    auto left_p1 = 0;
    auto left_p2 = 0;
    auto left_k3 = 0;

    auto right_fx = 1399.91;
    auto right_fy = 1399.91;
    auto right_cx = 977.78;
    auto right_cy = 570.336;
    auto right_k1 = -0.170864;
    auto right_k2 = 0.0245901;
    auto right_p1 = 0;
    auto right_p2 = 0;
    auto right_k3 = 0;


    const SingleCameraCalibration leftCam(
            left_fx,
            left_fy,
            left_cx,
            left_cy,
            left_k1,
            left_k2,
            left_p1,
            left_p2,
            left_k3
    );

    const SingleCameraCalibration rightCam(
            right_fx,
            right_fy,
            right_cx,
            right_cy,
            right_k1,
            right_k2,
            right_p1,
            right_p2,
            right_k3
    );

    return StereoCameraCalibrationData(image_size, leftCam, rightCam,
                                       120,
                                       0,
                                       0,

                                       0.0126552,
                                       0.00543845,
                                       -4.02686e-05

    );
}


StereoCameraCalibrationData calibrationDataHD(cv::Size2i image_size) {


    auto left_fx = 700.115;
    auto left_fy = 700.115;
    auto left_cx = 652.12;
    auto left_cy = 359.865;
    auto left_k1 = -0.167491;
    auto left_k2 = 0.0207842;
    auto left_p1 = 0;
    auto left_p2 = 0;
    auto left_k3 = 0;

    auto right_fx = 699.955;
    auto right_fy = 699.955;
    auto right_cx = 647.39;
    auto right_cy = 373.668;
    auto right_k1 = -0.170864;
    auto right_k2 = 0.0245901;
    auto right_p1 = 0;
    auto right_p2 = 0;
    auto right_k3 = 0;


    const SingleCameraCalibration leftCam(
            left_fx,
            left_fy,
            left_cx,
            left_cy,
            left_k1,
            left_k2,
            left_p1,
            left_p2,
            left_k3
    );

    const SingleCameraCalibration rightCam(
            right_fx,
            right_fy,
            right_cx,
            right_cy,
            right_k1,
            right_k2,
            right_p1,
            right_p2,
            right_k3
    );

    return StereoCameraCalibrationData(image_size, leftCam, rightCam,
                                       120,
                                       0,
                                       0,

                                       0.0126552,
                                       0.00543845,
                                       -4.02686e-05

    );
}


StereoCameraCalibrationData getCalibration(cv::Size2i image_size) {
    //break not needed as they're just returned
    switch ((int) image_size.width) {
        case 2208:
            return calibrationData2K(image_size);
        case 1920:
            return calibrationDataFHD(image_size);
        case 1280:
            return calibrationDataHD(image_size);
        case 672:
            return calibrationDataVGA(image_size);
        default:
            return calibrationDataHD(image_size);
    }
}

std::string MatType2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
        case CV_8U:
            r = "8U";
            break;
        case CV_8S:
            r = "8S";
            break;
        case CV_16U:
            r = "16U";
            break;
        case CV_16S:
            r = "16S";
            break;
        case CV_32S:
            r = "32S";
            break;
        case CV_32F:
            r = "32F";
            break;
        case CV_64F:
            r = "64F";
            break;
        default:
            r = "User";
            break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

#if 0
/*
 * bool initCalibration(std::string calibration_file, cv::Size2i image_size, cv::Mat &map_left_x, cv::Mat &map_left_y,
        cv::Mat &map_right_x, cv::Mat &map_right_y, cv::Mat &cameraMatrix_left, cv::Mat &cameraMatrix_right, double *baseline=nullptr) {

    if (!checkFile(calibration_file)) {
        std::cout << "Calibration file missing." << std::endl;
        return 0;
    }

    // Open camera configuration file
    ConfManager camerareader(calibration_file.c_str());
    if (!camerareader.isOpened())
        return 0;

    std::string resolution_str;
    switch ((int) image_size.width) {
        case 2208:
            resolution_str = "2k";
            break;
        case 1920:
            resolution_str = "fhd";
            break;
        case 1280:
            resolution_str = "hd";
            break;
        case 672:
            resolution_str = "vga";
            break;
        default:
            resolution_str = "hd";
            break;
    }

    // Get translations
    float T_[3];
    T_[0] = camerareader.getValue("stereo:baseline", 0.0f);
    T_[1] = camerareader.getValue("stereo:ty_" + resolution_str, 0.f);
    T_[2] = camerareader.getValue("stereo:tz_" + resolution_str, 0.f);

    if(baseline) *baseline=T_[0];

    // Get left parameters
    float left_cam_cx = camerareader.getValue("left_cam_" + resolution_str + ":cx", 0.0f);
    float left_cam_cy = camerareader.getValue("left_cam_" + resolution_str + ":cy", 0.0f);
    float left_cam_fx = camerareader.getValue("left_cam_" + resolution_str + ":fx", 0.0f);
    float left_cam_fy = camerareader.getValue("left_cam_" + resolution_str + ":fy", 0.0f);
    float left_cam_k1 = camerareader.getValue("left_cam_" + resolution_str + ":k1", 0.0f);
    float left_cam_k2 = camerareader.getValue("left_cam_" + resolution_str + ":k2", 0.0f);
    float left_cam_p1 = camerareader.getValue("left_cam_" + resolution_str + ":p1", 0.0f);
    float left_cam_p2 = camerareader.getValue("left_cam_" + resolution_str + ":p2", 0.0f);
    float left_cam_k3 = camerareader.getValue("left_cam_" + resolution_str + ":k3", 0.0f);

    // Get right parameters
    float right_cam_cx = camerareader.getValue("right_cam_" + resolution_str + ":cx", 0.0f);
    float right_cam_cy = camerareader.getValue("right_cam_" + resolution_str + ":cy", 0.0f);
    float right_cam_fx = camerareader.getValue("right_cam_" + resolution_str + ":fx", 0.0f);
    float right_cam_fy = camerareader.getValue("right_cam_" + resolution_str + ":fy", 0.0f);
    float right_cam_k1 = camerareader.getValue("right_cam_" + resolution_str + ":k1", 0.0f);
    float right_cam_k2 = camerareader.getValue("right_cam_" + resolution_str + ":k2", 0.0f);
    float right_cam_p1 = camerareader.getValue("right_cam_" + resolution_str + ":p1", 0.0f);
    float right_cam_p2 = camerareader.getValue("right_cam_" + resolution_str + ":p2", 0.0f);
    float right_cam_k3 = camerareader.getValue("right_cam_" + resolution_str + ":k3", 0.0f);

    // (Linux only) Safety check A: Wrong "." or "," reading in file conf.
#ifndef _WIN32
    if (right_cam_k1 == 0 && left_cam_k1 == 0 && left_cam_k2 == 0 && right_cam_k2 == 0) {
        std::cout << "ZED File invalid" << std::endl;

        std::string cmd = "rm " + calibration_file;
        int res = system(cmd.c_str());
        if( res == EXIT_FAILURE )
        {
            exit(1);
        }

        exit(1);
    }
#endif

    // Get rotations
    cv::Mat R_zed = (cv::Mat_<double>(1, 3) << camerareader.getValue("stereo:rx_" + resolution_str, 0.f), camerareader.getValue("stereo:cv_" + resolution_str, 0.f), camerareader.getValue("stereo:rz_" + resolution_str, 0.f));
    cv::Mat R;

    cv::Rodrigues(R_zed /*in*/, R /*out*/);

cv::Mat distCoeffs_left, distCoeffs_right;

// Left
cameraMatrix_left = (cv::Mat_<double>(3, 3) << left_cam_fx, 0, left_cam_cx, 0, left_cam_fy, left_cam_cy, 0, 0, 1);
distCoeffs_left = (cv::Mat_<double>(5, 1) << left_cam_k1, left_cam_k2, left_cam_p1, left_cam_p2, left_cam_k3);

// Right
cameraMatrix_right = (cv::Mat_<double>(3, 3) << right_cam_fx, 0, right_cam_cx, 0, right_cam_fy, right_cam_cy, 0, 0, 1);
distCoeffs_right = (cv::Mat_<double>(5, 1) << right_cam_k1, right_cam_k2, right_cam_p1, right_cam_p2, right_cam_k3);

// Stereo
cv::Mat T = (cv::Mat_<double>(3, 1) << T_[0], T_[1], T_[2]);
std::cout << " Camera Matrix L: \n" << cameraMatrix_left << std::endl << std::endl;
std::cout << " Camera Matrix R: \n" << cameraMatrix_right << std::endl << std::endl;

cv::Mat R1, R2, P1, P2, Q;
cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, image_size, R, T,
        R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, image_size);

//Precompute maps for cv::remap()
initUndistortRectifyMap(cameraMatrix_left, distCoeffs_left, R1, P1, image_size, CV_32FC1, map_left_x, map_left_y);
initUndistortRectifyMap(cameraMatrix_right, distCoeffs_right, R2, P2, image_size, CV_32FC1, map_right_x, map_right_y);

cameraMatrix_left = P1;
cameraMatrix_right = P2;

return 1;
}
 */
#endif