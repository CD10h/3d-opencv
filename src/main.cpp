#include "camera.h"
#include "ocv_display.hpp"

int simple_main(int argc, char *argv[]);

int main(int argc, char **argv)
{
    return simple_main(argc, argv);
}

#define TEST_FPS 0

// TODO: Reinitialize if signal lost
// TODO: Write video
// TODO: Depth mapping
// TODO: 3d reconstruction
// TODO: Location tracking? ( no IMU available btw)

// The main function
int simple_main(int argc, char *argv[])
{
    CameraWrapped camera;
    if (!camera.init(-1))
        abort();

    bool quit = false;
    // Infinite video grabbing loop
    while (!quit)
    {
        camera.captureFrame();

        // ----> If the frame is valid we can display it
        if (camera.haveFrame())
        {

            auto image = camera.getCombinedStereoFrame();
            sl_oc::tools::showImage("Stream RGB", image, camera.getResolution());
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
