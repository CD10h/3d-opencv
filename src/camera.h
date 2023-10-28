#pragma once
#include <utility>
#include <opencv2/opencv.hpp>
#include <videocapture.hpp>
#include "calibration.hpp"
#include "stereo.hpp"

enum class CamControlParams
{
    Brightness,
    Contrast,
    Hue,
    Saturation,
    Gain,
    Exposure,
    WhiteBalance,
    Sharpness,
    Gamma
};

class CameraWrapped
{
public:
    CameraWrapped() : CameraWrapped(defaultParams())
    {
    }
    CameraWrapped(sl_oc::video::VideoParams params) : video_params(params), video_device(params), have_frame(false)
    {
    }

    bool init(int device_id);
    cv::Mat getCombinedStereoFrame();
    bool captureFrame();
    inline bool haveFrame() const { return this->have_frame; }
    sl_oc::video::RESOLUTION getResolution() const { return this->video_params.res; }

    void resetControls();
    void toggleAutoWB();
    void toggleAutoExpGain();
    void cvShowImage(std::string name, const std::string &win_name, cv::Mat &img, bool drawROI, int img_w, int img_h);
    void applyROI(bool applyAECAGCrectLeft, bool applyAECAGCrectRight);

    void updateAllCtrlValues();
    void initCalibration();

    void frameSize(int &outW, int &outH);
    std::pair<cv::Mat, cv::Mat> GetRawLR();
    std::pair<cv::Mat, cv::Mat> GetRectifiedLR();

    void init_depthmatch();
    void depth_single_frame(bool half);

private:
    void setControlValue(CamControlParams param, int value);
    void changeControlValue(CamControlParams param, bool increase);

    sl_oc::video::VideoParams defaultParams() const
    {
        sl_oc::video::VideoParams parameters;
        parameters.verbose = sl_oc::VERBOSITY::INFO;
        parameters.res = sl_oc::video::RESOLUTION::HD2K;
        parameters.fps = sl_oc::video::FPS::FPS_60;
        return parameters;
    }

    sl_oc::video::VideoParams video_params;
    sl_oc::video::VideoCapture video_device;
    bool have_frame;
    sl_oc::video::Frame last_frame;

    uint8_t brightness_val;
    uint8_t contrast_val;
    uint8_t hue_val;
    uint8_t saturation_val;
    uint8_t gain_val_left;
    uint8_t gain_val_right;
    uint8_t exposure_val_left;
    uint8_t exposure_val_right;
    int whiteBalance_val;
    uint8_t sharpness_val;
    uint8_t gamma_val;
    bool autoAECAGC = false;
    bool autoWB = false;
    cv::Rect aecagc_roi_left = {0, 0, 0, 0};  // The current agcaec ROI rectangle
    cv::Rect aecagc_roi_right = {0, 0, 0, 0}; // The current agcaec ROI rectangle
    cv::Point origin_roi = {0, 0};            // TODO: Figure this shit out

    // calibration shit
    cv::Mat map_left_x, map_left_y;
    cv::Mat map_right_x, map_right_y;
    cv::Mat cameraMatrix_left, cameraMatrix_right;

    // depth shit
    sl_oc::tools::StereoSgbmPar stereoPar;
    cv::Ptr<cv::StereoSGBM> left_matcher;
    double fx;
    double fy;
    double cx;
    double cy;
};