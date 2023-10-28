#include "camera.h"

#include <cassert>

bool CameraWrapped::init(int device_id = -1)
{
    return this->video_device.initializeVideo(device_id);
}

bool CameraWrapped::captureFrame()
{

    auto frame = this->video_device.getLastFrame();
    if (frame.data == nullptr)
    {
        this->have_frame = false;
        return false;
    }
    this->have_frame = true;
    this->last_frame = frame;

    return true;
}

cv::Mat CameraWrapped::getCombinedStereoFrame()
{
    assert(this->have_frame);

    cv::Mat frameYUV = cv::Mat(this->last_frame.height, this->last_frame.width, CV_8UC2, this->last_frame.data);
    cv::Mat frameBGR;
    cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

    return frameBGR;
}

void CameraWrapped::resetControls()
{
    this->video_device.resetBrightness();
    this->video_device.resetSharpness();
    this->video_device.resetContrast();
    this->video_device.resetHue();
    this->video_device.resetSaturation();
    this->video_device.resetGamma();
    this->video_device.resetAECAGC();
    this->video_device.resetAutoWhiteBalance();
    this->video_device.resetROIforAECAGC(sl_oc::video::CAM_SENS_POS::LEFT);
    this->video_device.resetROIforAECAGC(sl_oc::video::CAM_SENS_POS::RIGHT);
}

void CameraWrapped::toggleAutoExpGain()
{
    bool curValue = this->video_device.getAECAGC();
    this->video_device.setAECAGC(!curValue);
}

void CameraWrapped::toggleAutoWB()
{
    bool curValue = this->video_device.getAutoWhiteBalance();
    this->video_device.setAutoWhiteBalance(!curValue);
}

void CameraWrapped::updateAllCtrlValues()
{
    sl_oc::video::VideoCapture &cap = this->video_device;

    this->brightness_val = cap.getBrightness();
    this->sharpness_val = cap.getSharpness();
    this->contrast_val = cap.getContrast();
    this->hue_val = cap.getHue();
    this->saturation_val = cap.getSaturation();
    this->gamma_val = cap.getGamma();
    this->autoAECAGC = cap.getAECAGC();
    this->autoWB = cap.getAutoWhiteBalance();
    this->gain_val_left = cap.getGain(sl_oc::video::CAM_SENS_POS::LEFT);
    this->gain_val_right = cap.getGain(sl_oc::video::CAM_SENS_POS::RIGHT);
    this->whiteBalance_val = cap.getWhiteBalance();

    uint16_t x, y, w, h;
    cap.getROIforAECAGC(sl_oc::video::CAM_SENS_POS::LEFT, x, y, w, h);
    aecagc_roi_left.x = x;
    aecagc_roi_left.y = y;
    aecagc_roi_left.width = w;
    aecagc_roi_left.height = h;
    cap.getROIforAECAGC(sl_oc::video::CAM_SENS_POS::RIGHT, x, y, w, h);
    aecagc_roi_right.x = x;
    aecagc_roi_right.y = y;
    aecagc_roi_right.width = w;
    aecagc_roi_right.height = h;
}

void CameraWrapped::initCalibration()
{
    const std::string calibration_file = "~/zed/settings/SN4461.conf";

    int w, h;
    this->video_device.getFrameSize(w, h);

    this->calibration = getCalibration(cv::Size(w / 2, h));
}

// Set new value for the active control
void CameraWrapped::setControlValue(CamControlParams param, int value)
{
    sl_oc::video::VideoCapture &cap = this->video_device;
    int newValue;
    switch (param)
    {
    case CamControlParams::Brightness:
        cap.setBrightness(value);
        newValue = cap.getBrightness();
        brightness_val = newValue;

        std::cout << "New Brightness value: ";
        break;

    case CamControlParams::Contrast:
        cap.setContrast(value);
        newValue = cap.getContrast();
        contrast_val = newValue;

        std::cout << "New Contrast value: ";
        break;

    case CamControlParams::Hue:
        cap.setHue(value);
        newValue = cap.getHue();
        hue_val = newValue;

        std::cout << "New Hue value: ";
        break;

    case CamControlParams::Saturation:
        cap.setSaturation(value);
        newValue = cap.getSaturation();
        saturation_val = newValue;

        std::cout << "New Saturation value: ";
        break;

    case CamControlParams::WhiteBalance:
        cap.setWhiteBalance(2800 + value * 411);
        newValue = cap.getWhiteBalance();
        whiteBalance_val = newValue;

        std::cout << "New White Balance value: ";
        break;

    case CamControlParams::Sharpness:
        cap.setSharpness(value);
        newValue = cap.getSharpness();
        sharpness_val = newValue;

        std::cout << "New Sharpness value: ";
        break;

    case CamControlParams::Gamma:
        cap.setGamma(value);
        newValue = cap.getGamma();
        gamma_val = newValue;

        std::cout << "New Gamma value: ";
        break;

    case CamControlParams::Gain:
    case CamControlParams::Exposure:
    default:
        // Nothing to do here
        return;
    }

    std::cout << newValue << std::endl;
}

// '+' or '-' pressed
void CameraWrapped::changeControlValue(CamControlParams param, bool increase)
{
    sl_oc::video::VideoCapture &cap = this->video_device;

    int curValue = 0;
    switch (param)
    {
    case CamControlParams::Brightness:
        curValue = cap.getBrightness();
        brightness_val = curValue;
        break;

    case CamControlParams::Contrast:
        curValue = cap.getContrast();
        contrast_val = curValue;
        break;

    case CamControlParams::Hue:
        curValue = cap.getHue();
        hue_val = curValue;
        break;

    case CamControlParams::Saturation:
        curValue = cap.getSaturation();
        saturation_val = curValue;
        break;

    case CamControlParams::Gain:
    {
        int curValueLeft = cap.getGain(sl_oc::video::CAM_SENS_POS::LEFT);
        int curValueRight = cap.getGain(sl_oc::video::CAM_SENS_POS::RIGHT);

        if (increase)
        {
            cap.setGain(sl_oc::video::CAM_SENS_POS::LEFT, ++curValueLeft);
            cap.setGain(sl_oc::video::CAM_SENS_POS::RIGHT, ++curValueRight);
        }
        else
        {
            cap.setGain(sl_oc::video::CAM_SENS_POS::LEFT, --curValueLeft);
            cap.setGain(sl_oc::video::CAM_SENS_POS::RIGHT, --curValueRight);
            ;
        }

        gain_val_left = cap.getGain(sl_oc::video::CAM_SENS_POS::LEFT);
        gain_val_right = cap.getGain(sl_oc::video::CAM_SENS_POS::RIGHT);

        std::cout << "New Left Gain value: " << (int)gain_val_left << std::endl;
        std::cout << "New Right Gain value: " << (int)gain_val_right << std::endl;

        autoAECAGC = cap.getAECAGC();
    }
    break;

    case CamControlParams::Exposure:
    {
        int curValueLeft = cap.getExposure(sl_oc::video::CAM_SENS_POS::LEFT);
        int curValueRight = cap.getExposure(sl_oc::video::CAM_SENS_POS::RIGHT);

        if (increase)
        {
            cap.setExposure(sl_oc::video::CAM_SENS_POS::LEFT, ++curValueLeft);
            cap.setExposure(sl_oc::video::CAM_SENS_POS::RIGHT, ++curValueRight);
        }
        else
        {
            cap.setExposure(sl_oc::video::CAM_SENS_POS::LEFT, --curValueLeft);
            cap.setExposure(sl_oc::video::CAM_SENS_POS::RIGHT, --curValueRight);
            ;
        }

        exposure_val_left = cap.getExposure(sl_oc::video::CAM_SENS_POS::LEFT);
        exposure_val_right = cap.getExposure(sl_oc::video::CAM_SENS_POS::RIGHT);

        std::cout << "New Left Exposure value: " << (int)exposure_val_left << std::endl;
        std::cout << "New Right Exposure value: " << (int)exposure_val_right << std::endl;

        autoAECAGC = cap.getAECAGC();
    }
    break;

    case CamControlParams::WhiteBalance:
        cap.setAutoWhiteBalance(false);
        curValue = cap.getWhiteBalance();
        whiteBalance_val = curValue;
        autoWB = cap.getAutoWhiteBalance();
        break;

    case CamControlParams::Sharpness:
        curValue = cap.getSharpness();
        sharpness_val = curValue;
        break;

    case CamControlParams::Gamma:
        curValue = cap.getGamma();
        gamma_val = curValue;
        break;
    }

    if (param == CamControlParams::WhiteBalance)
    {
        if (increase)
            curValue += 100;
        else
            curValue -= 100;

        cap.setWhiteBalance(curValue);

        whiteBalance_val = cap.getWhiteBalance();
        std::cout << "New White Balance value: " << whiteBalance_val << std::endl;
    }
    else if (param != CamControlParams::Gain && param != CamControlParams::Exposure)
    {
        if (increase)
            setControlValue(param, ++curValue);
        else
            setControlValue(param, --curValue);
    }
}

// Rescale the images according to the selected resolution to better display them on screen
void CameraWrapped::cvShowImage(std::string name, const std::string &win_name, cv::Mat &img, bool drawROI, int img_w,
                                int img_h)
{
    double img_resize_factor = 1.0; // Image resize factor for drawing

    sl_oc::video::RESOLUTION res = this->video_params.res;
    cv::Mat resized;
    switch (res)
    {
    default:
    case sl_oc::video::RESOLUTION::VGA:
        img_resize_factor = 1.0;
        resized = img;
        break;
    case sl_oc::video::RESOLUTION::HD720:
        name += " [Resize factor 0.6]";
        img_resize_factor = 0.6;
        cv::resize(img, resized, cv::Size(), img_resize_factor, img_resize_factor);
        break;
    case sl_oc::video::RESOLUTION::HD1080:
    case sl_oc::video::RESOLUTION::HD2K:
        name += " [Resize factor 0.4]";
        img_resize_factor = 0.4;
        cv::resize(img, resized, cv::Size(), img_resize_factor, img_resize_factor);
        break;
    }

    if (autoAECAGC && drawROI)
    {
        // Check that left selection rectangle is valid and draw it on the image
        if ((aecagc_roi_left.area() > 0) &&
            (aecagc_roi_left.width - aecagc_roi_left.x) <= img_w / 2 &&
            (aecagc_roi_left.height - aecagc_roi_left.y) <= img_h)
        {
            cv::Rect rescaled_roi;
            rescaled_roi.x = aecagc_roi_left.x * img_resize_factor;
            rescaled_roi.y = aecagc_roi_left.y * img_resize_factor;
            rescaled_roi.width = aecagc_roi_left.width * img_resize_factor;
            rescaled_roi.height = aecagc_roi_left.height * img_resize_factor;
            cv::rectangle(resized, rescaled_roi, cv::Scalar(220, 180, 20), 2);
        }

        // Check that right selection rectangle is valid and draw it on the image
        if ((aecagc_roi_right.area() > 0) &&
            (aecagc_roi_right.width - aecagc_roi_right.x) <= img_w / 2 &&
            (aecagc_roi_right.height - aecagc_roi_right.y) <= img_h)
        {
            cv::Rect rescaled_roi;
            rescaled_roi.x = (img_w / 2 + aecagc_roi_right.x) * img_resize_factor;
            rescaled_roi.y = aecagc_roi_right.y * img_resize_factor;
            rescaled_roi.width = aecagc_roi_right.width * img_resize_factor;
            rescaled_roi.height = aecagc_roi_right.height * img_resize_factor;
            cv::rectangle(resized, rescaled_roi, cv::Scalar(20, 180, 220), 2);
        }
    }

    cv::imshow(win_name, resized);
}

void CameraWrapped::applyROI(bool applyAECAGCrectLeft, bool applyAECAGCrectRight)
{
    sl_oc::video::VideoCapture &cap = this->video_device;

    if (applyAECAGCrectLeft)
    {
        applyAECAGCrectLeft = false;
        cap.setROIforAECAGC(sl_oc::video::CAM_SENS_POS::LEFT,
                            aecagc_roi_left.x, aecagc_roi_left.y,
                            aecagc_roi_left.width, aecagc_roi_left.height);
        // selectLeft = false;
        // selectRight = false;
    }
    if (applyAECAGCrectRight)
    {
        applyAECAGCrectRight = false;
        cap.setROIforAECAGC(sl_oc::video::CAM_SENS_POS::RIGHT,
                            aecagc_roi_right.x, aecagc_roi_right.y,
                            aecagc_roi_right.width, aecagc_roi_right.height);
        // selectLeft = false;
        // selectRight = false;
    }
}

void CameraWrapped::frameSize(int &outW, int &outH)
{
    this->video_device.getFrameSize(outW, outH);
}

std::pair<cv::Mat, cv::Mat> CameraWrapped::GetRawLR()
{
    cv::Mat combinedStereo = this->getCombinedStereoFrame();

    cv::Mat left, right;

    left = combinedStereo(cv::Rect(0, 0, combinedStereo.cols / 2, combinedStereo.rows));
    right = combinedStereo(cv::Rect(combinedStereo.cols / 2, 0, combinedStereo.cols / 2, combinedStereo.rows));

    return std::pair<cv::Mat, cv::Mat>(std::move(left), std::move(right));
}

void CameraWrapped::GetRectifiedLR(cv::OutputArray &outLeft, cv::OutputArray &outRight)
{
    auto lr_raw = this->GetRawLR();

    cv::Mat left_raw = lr_raw.first;
    cv::Mat right_raw = lr_raw.second;

    cv::remap(left_raw, outLeft, calibration.undistortLeftX, calibration.undistortLeftY, cv::INTER_AREA);
    cv::remap(right_raw, outRight, calibration.undistortRightX, calibration.undistortRightY, cv::INTER_AREA);
}
