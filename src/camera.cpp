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

    if (!sl_oc::tools::initCalibration(calibration_file, cv::Size(w / 2, h), map_left_x, map_left_y, map_right_x, map_right_y,
                                       cameraMatrix_left, cameraMatrix_right))
    {
        std::cerr << "BAD!!!" << std::endl;
        abort();
    }
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
void CameraWrapped::cvShowImage(std::string name, const std::string &win_name, cv::Mat &img, bool drawROI, int img_w, int img_h)
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

    return std::pair<cv::Mat, cv::Mat>(left, right);
}
std::pair<cv::Mat, cv::Mat> CameraWrapped::GetRectifiedLR()
{
    std::pair<cv::Mat, cv::Mat> rawLR = this->GetRawLR();

    cv::Mat raw_left, raw_right, rectified_left, rectified_right;
    raw_left = rawLR.first;
    raw_right = rawLR.second;

    cv::remap(raw_left, rectified_left, map_left_x, map_left_y, cv::INTER_LINEAR);
    cv::remap(raw_right, rectified_right, map_right_x, map_right_y, cv::INTER_LINEAR);

    return std::pair<cv::Mat, cv::Mat>(rectified_left, rectified_right);
}

#undef HAVE_OPENCV_VIZ // Uncomment if cannot use Viz3D for point cloud rendering

#if HAVE_OPENCV_VIZ == 1
#include <opencv2/viz.hpp>
#include <opencv2/viz/viz3d.hpp>
#endif

#define USE_OCV_TAPI       // Comment to use "normal" cv::Mat instead of CV::UMat
#define USE_HALF_SIZE_DISP // Comment to compute depth matching on full image frames

void CameraWrapped::init_depthmatch()
{

    fx = cameraMatrix_left.at<double>(0, 0);
    fy = cameraMatrix_left.at<double>(1, 1);
    cx = cameraMatrix_left.at<double>(0, 2);
    cy = cameraMatrix_left.at<double>(1, 2);

    std::cout << " Camera Matrix L: \n"
              << cameraMatrix_left << std::endl
              << std::endl;
    std::cout << " Camera Matrix R: \n"
              << cameraMatrix_right << std::endl
              << std::endl;

    cv::UMat map_left_x_gpu = map_left_x.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat map_left_y_gpu = map_left_y.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat map_right_x_gpu = map_right_x.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat map_right_y_gpu = map_right_y.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

       this->left_matcher = cv::StereoSGBM::create(stereoPar.minDisparity, stereoPar.numDisparities, stereoPar.blockSize);
    left_matcher->setMinDisparity(stereoPar.minDisparity);
    left_matcher->setNumDisparities(stereoPar.numDisparities);
    left_matcher->setBlockSize(stereoPar.blockSize);
    left_matcher->setP1(stereoPar.P1);
    left_matcher->setP2(stereoPar.P2);
    left_matcher->setDisp12MaxDiff(stereoPar.disp12MaxDiff);
    left_matcher->setMode(stereoPar.mode);
    left_matcher->setPreFilterCap(stereoPar.preFilterCap);
    left_matcher->setUniquenessRatio(stereoPar.uniquenessRatio);
    left_matcher->setSpeckleWindowSize(stereoPar.speckleWindowSize);
    left_matcher->setSpeckleRange(stereoPar.speckleRange);
}

void CameraWrapped::depth_single_frame(bool half = true)
{
    const double baseline = 0;
    auto frame = this->last_frame;
    // ----> If the frame is valid we can convert, rectify and display it
    {

        auto rectLR = this->GetRectifiedLR();
        cv::UMat left_rect = rectLR.first.getUMat(cv::ACCESS_READ);
        cv::UMat right_rect = rectLR.second.getUMat(cv::ACCESS_READ);
        cv::UMat left_for_matcher;  // Left image for the stereo matcher
        cv::UMat right_for_matcher; // Right image for the stereo matcher
        cv::UMat left_disp_half;    // Half sized disparity map
        cv::UMat left_disp_float;   // Final disparity map in float32
        cv::UMat left_disp;         // Full output disparity

        cv::UMat left_disp_image; // Normalized and color remapped disparity map to be displayed
        cv::UMat left_depth_map;  // Depth map in float32

        // ----> Stereo matching
        double resize_fact = 1.0;
        if (half)
        {
            resize_fact = 0.5;
            // Resize the original images to improve performances
            cv::resize(left_rect, left_for_matcher, cv::Size(), resize_fact, resize_fact, cv::INTER_AREA);
            cv::resize(right_rect, right_for_matcher, cv::Size(), resize_fact, resize_fact, cv::INTER_AREA);
        }
        else
        {
            left_for_matcher = left_rect;   // No data copy
            right_for_matcher = right_rect; // No data copy
        }
        // Apply stereo matching
        left_matcher->compute(left_for_matcher, right_for_matcher, left_disp_half);

        left_disp_half.convertTo(left_disp_float, CV_32FC1);
        cv::multiply(left_disp_float, 1. / 16., left_disp_float); // Last 4 bits of SGBM disparity are decimal

        if (half)
        {
            cv::multiply(left_disp_float, 2., left_disp_float); // Last 4 bits of SGBM disparity are decimal
            cv::UMat tmp = left_disp_float;                     // Required for OpenCV 3.2
            cv::resize(tmp, left_disp_float, cv::Size(), 1. / resize_fact, 1. / resize_fact, cv::INTER_AREA);
        }
        else
        {
            left_disp = left_disp_float;
        }

        // ----> Show disparity image
        cv::add(left_disp_float, -static_cast<double>(stereoPar.minDisparity - 1), left_disp_float);  // Minimum disparity offset correction
        cv::multiply(left_disp_float, 1. / stereoPar.numDisparities, left_disp_image, 255., CV_8UC1); // Normalization and rescaling

        cv::applyColorMap(left_disp_image, left_disp_image, cv::COLORMAP_JET); // COLORMAP_INFERNO is better, but it's only available starting from OpenCV v4.1.0

        // <---- Show disparity image

        // ----> Extract Depth map
        // The DISPARITY MAP can be now transformed in DEPTH MAP using the formula
        // depth = (f * B) / disparity
        // where 'f' is the camera focal, 'B' is the camera baseline, 'disparity' is the pixel disparity

        double num = static_cast<double>(fx * baseline);
        cv::divide(num, left_disp_float, left_depth_map);

        float central_depth = left_depth_map.getMat(cv::ACCESS_READ).at<float>(left_depth_map.rows / 2, left_depth_map.cols / 2);
        std::cout << "Depth of the central pixel: " << central_depth << " mm" << std::endl;
        // <---- Extract Depth map

        // ----> Create Point Cloud
        size_t buf_size = static_cast<size_t>(left_depth_map.cols * left_depth_map.rows);
        std::vector<cv::Vec3d> buffer(buf_size, cv::Vec3f::all(std::numeric_limits<float>::quiet_NaN()));
        cv::Mat depth_map_cpu = left_depth_map.getMat(cv::ACCESS_READ);
        float *depth_vec = (float *)(&(depth_map_cpu.data[0]));

        // omp parallel
        for (size_t idx = 0; idx < buf_size; idx++)
        {
            size_t r = idx / left_depth_map.cols;
            size_t c = idx % left_depth_map.cols;
            double depth = static_cast<double>(depth_vec[idx]);
            if (!isinf(depth) && depth >= 0 && depth > stereoPar.minDepth_mm && depth < stereoPar.maxDepth_mm)
            {
                buffer[idx].val[2] = depth;                 // Z
                buffer[idx].val[0] = (c - cx) * depth / fx; // X
                buffer[idx].val[1] = (r - cy) * depth / fy; // Y
            }
        }
        cv::Mat cloudMat = cv::Mat(left_depth_map.rows, left_depth_map.cols, CV_64FC3, &buffer[0]).clone();
    }
}
