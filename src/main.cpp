////////////////////////////////////////////////////////////////////////////
////
//// Copyright (c) 2021, STEREOLABS.
////
//// All rights reserved.
////
//// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
////
/////////////////////////////////////////////////////////////////////////////

//// ----> Includes
#define VIDEO_MOD_AVAILABLE 1

#include "videocapture.hpp"
#include "ocv_display.hpp"

#include <iostream>
#include <iomanip>

#include <opencv2/opencv.hpp>
// <---- Includes
#define TEST_FPS 0

// TODO: Reinitialize if signal lost
// TODO: Write video
// TODO: Depth mapping
// TODO: 3d reconstruction
// TODO: Location tracking? ( no IMU available btw)

bool handle_frame(sl_oc::video::VideoCapture &camera, const sl_oc::video::Frame &frame, const sl_oc::video::VideoParams &params)
{

    cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
    cv::Mat frameBGR;
    cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

    sl_oc::tools::showImage("Stream RGB", frameBGR, params.res);
    return true;
}

// The main function
int main(int argc, char *argv[])
{
    // ----> Silence unused warning
    (void)argc;
    (void)argv;
    // <---- Silence unused warning

    sl_oc::video::VideoParams params;
    params.verbose = sl_oc::VERBOSITY::INFO;
    params.res = sl_oc::video::RESOLUTION::HD2K;
    params.fps = sl_oc::video::FPS::FPS_60;

    // ----> Create Video Capture
    sl_oc::video::VideoCapture cap_0(params);
    if (!cap_0.initializeVideo())
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;

        return EXIT_FAILURE;
    }

    std::cout << "Connected to camera sn: " << cap_0.getSerialNumber() << std::endl;
    // <---- Create Video Capture

#ifdef TEST_FPS
    // Timestamp to check FPS
    double lastTime = static_cast<double>(getSteadyTimestamp()) / 1e9;
    // Frame timestamp to check FPS
    uint64_t lastFrameTs = 0;
#endif
    bool quit = false;
    // Infinite video grabbing loop
    while (!quit)
    {
        // Get last available frame
        const sl_oc::video::Frame frame = cap_0.getLastFrame();

        // ----> If the frame is valid we can display it
        if (frame.data != nullptr)
        {
#ifdef TEST_FPS
            if (lastFrameTs != 0)
            {
                // ----> System time
                double now = static_cast<double>(getSteadyTimestamp()) / 1e9;
                double elapsed_sec = now - lastTime;
                lastTime = now;
                std::cout << "[System] Frame period: " << elapsed_sec << "sec - Freq: " << 1. / elapsed_sec << " Hz" << std::endl;
                // <---- System time

                // ----> Frame time
                double frame_dT = static_cast<double>(frame.timestamp - lastFrameTs) / 1e9;
                std::cout << "[Camera] Frame period: " << frame_dT << "sec - Freq: " << 1. / frame_dT << " Hz" << std::endl;
                // <---- Frame time
            }
            lastFrameTs = frame.timestamp;
#endif
            if (!handle_frame(cap_0, frame, params))
            {
                std::cerr << "ERROR!!, quitting" << std::endl;
                quit = true;
            }
        }
        // <---- If the frame is valid we can display it

        // ----> Keyboard handling
        int key = cv::waitKey(5);
        if (key == 'q' || key == 'Q') // Quit
            quit = true;
        // <---- Keyboard handling
    }

    return EXIT_SUCCESS;
}
