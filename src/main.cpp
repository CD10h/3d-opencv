#include "camera.h"
#include "ocv_display.hpp"
#include "DepthMatcher.h"
#include <execinfo.h>
#include <signal.h>
#include <cstdio>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/viz/viz3d.hpp>

void handler(int sig) {
    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

int simple_main(int argc, char *argv[]);

int main(int argc, char **argv) {
    signal(SIGSEGV, handler);   // install our handler
    signal(SIGABRT, handler);   // install our handler

//    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_VERBOSE);
    return simple_main(argc, argv);
}

#define TEST_FPS 0

// TODO: Reinitialize if signal lost
// TODO: Write video
// TODO: Depth mapping
// TODO: 3d reconstruction
// TODO: Location tracking? ( no IMU available btw)

// The main function
int simple_main(int argc, char *argv[]) {
    CameraWrapped *camera = new CameraWrapped();
    DepthMatcher odepthMatcher;
    DepthMatcher *depthMatcher = &odepthMatcher;

//    DepthMatcher *depthMatcher = new DepthMatcher(DepthMatcherParams());
    if (!camera->init(-1))
        abort();

    camera->initCalibration();


    int w, h;
    camera->frameSize(w, h);
    auto cal = getCalibration(cv::Size(w / 2, h));


    bool quit = false;
    bool inited = true;
//    cv::viz::Viz3d pc_viewer = cv::viz::Viz3d("Point Cloud");
    cv::Mat pointcloud;
    // Infinite video grabbing loop
    while (!quit) {
        cv::Mat left_rect, right_rect, map_left_x, map_left_y, map_right_x, map_right_y, left_disp_float, left_disp_image;

        if (inited) {
            if (camera->captureFrame() && camera->haveFrame()) {

                camera->GetRectifiedLR(left_rect, right_rect);

                cv::UMat left_umat, right_umat;

                left_umat = left_rect.getUMat(cv::ACCESS_READ);
                right_umat = right_rect.getUMat(cv::ACCESS_READ);

                depthMatcher->calculateDisparity(left_umat, right_umat, left_disp_float);

                depthMatcher->createPointCloud(left_disp_float, cal, pointcloud);


                std::cerr << pointcloud.dims << std::endl;
                std::cerr << "[ ";
                for (int i = 0; i < pointcloud.dims; i++) {
                    std::cerr << pointcloud.size[i] << ", ";
                }
                std::cerr << " ]" << std::endl;

                cv::imshow("winwin", pointcloud);
//                abort();




                // ----> Show frames
                sl_oc::tools::showImage("Right rect.", right_rect, camera->getResolution(), true, "");
                sl_oc::tools::showImage("Left rect.", left_rect, camera->getResolution(), true, "");
                // <---- Show frames

//                // ----> Show disparity image
//                cv::add(left_disp_float, -static_cast<float>(depthMatcher->getMinDisparity() - 1),
//                        left_disp_float); // Minimum disparity offset correction
//                cv::multiply(left_disp_float, 1.f / depthMatcher->getNumDisparities(), left_disp_image, 255.,
//                             CV_8UC1); // Normalization and rescalin


//                cv::applyColorMap(left_disp_image, left_disp_image, cv::COLORMAP_INFERNO);
//                sl_oc::tools::showImage("Disparity", left_disp_image, camera->getResolution(), true, "");


            } else {
                std::cout << "REINIT NEEDED" << std::endl;
                inited = false;
            }
        } else {
            std::cout << "REINIT'ING" << std::endl;
            delete camera;
            camera = new CameraWrapped();
            if (!camera->init(-1)) {
                std::cout << "REINIT_FAILED, will try again" << std::endl;
            } else {
                inited = true;
            }
        }
        // <---- If the frame is valid we can display it

        // ----> Keyboard handling
        int key = cv::waitKey(5);
        if (key == 'q' || key == 'Q') // Quit
            quit = true;
        // ----> Show Point Cloud
////        cv::viz::WCloud cloudWidget(pointcloud, left_rect);
////        cloudWidget.setRenderingProperty(cv::viz::POINT_SIZE, 1);
////        pc_viewer.showWidget("Point Cloud", cloudWidget);
////        pc_viewer.spinOnce(1);
//
//        if (pc_viewer.wasStopped())
//            break;
        // <---- Show Point Cloud
        // <---- Keyboard handling
    }

    return EXIT_SUCCESS;
}
