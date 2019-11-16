#include "cv_main.h"
#include "cv_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <unistd.h>


#ifdef USE_SIMULATED_CAMERA	// With the simulated camera
    //These values define the saturation cutoffs
    #define MIN_HSV_SAT 60

    //These values define the brightness value cutoffs
    #define MIN_HSV_VAL 0
    #define MAX_HSV_VAL 256
#else //USE_SIMULATED_CAMERA
    //These values define the saturation cutoffs for use with a real camera
    #define MIN_HSV_SAT 30

    //These values define the brightness value cutoffs for use with a real camera
    #define MIN_HSV_VAL 10
    #define MAX_HSV_VAL 240
#endif //USE_SIMULATED_CAMERA

using namespace cv;

// Current robot state can be found in gvIn:
// only dof, curTime, q, qd, tau, x are updated with 50Hz in cvSetRobotState.
// dof     : degrees of freedom
// curTime : Our current simulator/real time
// tau     : joint torques
// q       : joint position
// dq      : joint ang. velocity
// x       : configuration parameters of end-effector in operational space
GlobalVariables	gvIn;

// Flags
bool	cv_running = false; // while this variable is true the cvLoop will run
bool	select_object = false; // this variable turns true when the user presses the left mouse button to begin the selection of a color in the image
bool	track_object = false; // this variable turns true when the user finishes selecting a color in the image, and turns false if this selection is cleared (pressing 'c')
bool	estimated_histogram = false; // this variable turns true when the histogram of the selected color has been estimated

pthread_mutex_t	mutexGVIn; // mutex for accessing (r/w) gvIn;

void Sleep(struct timespec& startTimeOfCurrentTimeSlice, float dt); //sleeps until the next iteration of the CV loop is due

///////////////////
// OpenCV variables

// OpenCV images ptr
IplImage *image = NULL, *hsv = NULL, *hue = NULL,*sat = NULL,*val = NULL, *mask = NULL, *backproject = NULL,
*backproject_processed = NULL, *message_img = NULL, *s_mask =NULL, *v_mask_dark =NULL, *v_mask_bright =NULL, *sv_mask =NULL;
// OpenCV histogram ptr
CvHistogram *hist = NULL;
// OpenCV points
CvPoint origin;
// OpenCV rectangle of the selected image region
CvRect selection;
// Number of dimensions of the histogram
// If we have only 2 we have a binary backprojection
// If we have more dimensions we could have several values (between black and white) in the backprojection ->
// -> It requires a thresholding!
int hdims = 16;
// Ranges of the histogram
float hranges_arr[] = {0,180};
float* hranges = hranges_arr;
///////////////////

void cvLoop(cmdfunc robotCmd, imgFactory getSimImg=NULL)
{

    std::cout << "Starting cvLoop" << std::endl;
    // Declaration of the local variables in the loop
    char				command_buf[256];			// Command to send to the robot
    GlobalVariables		gv;							// State of the robot
    cv::Mat* simulated_frame=NULL;                  // pointer to the rendered frame (in simulation only)
    IplImage iplimg;	                            // current local copy of converted cv::Mat image
    IplImage*			frame = &iplimg;			// pointer to current image from the camera
    int					c;							// Clicked keyboard
    Float				previous_iter_time = 0.0f;	// Time of the last iteration of the loop
    float				previous_command_time=0.0f;	// Time of the last iteration of the loop that sent a command to the robot
    Circle				crcl;						// Structure to store the parameters of the detected circle in the image
    struct timespec startTimeOfLastSlice = {0,0};
    if(robotCmd==NULL) return;

    // Initialize the mutex that protects the r/w operations on the state variable of the robot
    pthread_mutex_init( &mutexGVIn, NULL );

    // Declaration of the parameters of visual servoing
    float f;						// Focal length
    float img_width;				// Width of sensor image in pixels
    float img_height;				// Height of sensor image in pixels
    float diameter_real;			// Diameter of the circle in meters
    float diameter_desired_px;		// Desired diameter of the circle in the image in pixels
    float dt;						// This is suppose to be the delta time between iterations, but it is just an scaling factor


    // Initialization of the parameters of visual servoing
    dt = VISIONLOOP_PERIOD;
#ifdef USE_SIMULATED_CAMERA	// With the simulated camera
    f=320;
    img_width = 320;
    img_height = 240;
    diameter_real =0.25*110/256;
    diameter_desired_px=55;
#else		// With the real camera
    f=355;
    img_width = 320;
    img_height = 240;
    diameter_real =0.15;
    diameter_desired_px=65;

    // To grab images from the real camera -> Be careful if you have several cams connected!
    CvCapture* capture = 0;

    // This pause is necessary in the ubuntu real puma setup to
    // avoid the System Error 0 that is triggered when creating the capture
    usleep(1);
    usleep(2e5);
    capture = cvCaptureFromCAM( 0 );

    // Set the resolution of the real camera to be 320x240 pixels
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, img_width);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT,img_height);

    // The capture for the camera could not be initialized -> Is the camera connected?
    if( !capture )
    {
        std::cout << "[cvLoop] ERROR: The camera could not be found!" <<std::endl;
        return;
    }
#endif

    // Set the desired feature values in opencv frame (origin in the upper left corner)
    PrVector3 desired_s_opencvf;
    desired_s_opencvf[0] = img_width*0.5;
    desired_s_opencvf[1] = img_height*0.5;
    desired_s_opencvf[2] = diameter_desired_px;

    // Set the parameters for visual servoing
    initVisualServoing(f, img_width, img_height, diameter_real, diameter_desired_px, dt, desired_s_opencvf);

    // Create windows to display images
    cv::namedWindow( BACKPROJ_HIST_IMAGE_WINDOW_NAME, 1 );
    cv::namedWindow( VS_WINDOW_NAME, 1 );

    // Set the mouse callback to select objects on the VS image
    cv::setMouseCallback( VS_WINDOW_NAME, onMouse, 0 );

    cv_running=true;
    // LOOP
    while(cv_running)
    {
        Sleep(startTimeOfLastSlice, dt);
        // Make a local copy of the robot state (protected r/w operation using the mutex)
        pthread_mutex_lock(&mutexGVIn);
        GV_COPY(gv,gvIn);
        pthread_mutex_unlock(&mutexGVIn);

#ifdef USE_SIMULATED_CAMERA
        // get the opengl image
        if (simulated_frame) {delete simulated_frame;} //throw away last frame
        cv::Mat* simulated_frame=getSimImg();
        iplimg = IplImage(*simulated_frame); //Backwards compatibility, TODO: remove
        frame = &iplimg;                    //pointer does not actually change, just to make it clear here where it points to
#else
        // get the camera image
        frame = cvQueryFrame( capture );
#endif
        // Check that the image could be grabbed
        if( !frame )
        {
            fprintf( stderr, "[cvLoop] ERROR: No image could be grabbed from the camera!\n" );
            break;
        }

        // In the first loop iteration and using the first grabbed image we initialize the image containers (allocate all the buffers) and the
        // dilate/erode structure
        if( !image )
        {
            image = cvCreateImage( cvGetSize(frame), 8, 3 );
            image->origin = frame->origin;
            hsv = cvCreateImage( cvGetSize(frame), 8, 3 );
            hue = cvCreateImage( cvGetSize(frame), 8, 1 );
            sat = cvCreateImage( cvGetSize(frame), 8, 1 );
            val = cvCreateImage( cvGetSize(frame), 8, 1 );
            mask = cvCreateImage( cvGetSize(frame), 8, 1 );
            sv_mask = cvCreateImage( cvGetSize(frame), 8, 1 );
            s_mask = cvCreateImage( cvGetSize(frame), 8, 1 );
            v_mask_dark = cvCreateImage( cvGetSize(frame), 8, 1 );
            v_mask_bright = cvCreateImage( cvGetSize(frame), 8, 1 );
            backproject = cvCreateImage( cvGetSize(frame), 8, 1 );
            backproject_processed = cvCreateImage( cvGetSize(frame), 8, 1 );
            hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
        }

        // Set mask and backproject image to white
        cvSet(backproject, cvRealScalar(255.));
        cvSet(mask, cvRealScalar(255.));

        // Copy the grabbed image into "image"
        cvCopy( frame, image, 0 );
        // Convert IplImage image into an OpenCV2 Mat for further use
        cv::Mat img(image);

        // Change the color space of the grabbed image from BGR to HSV
        cvCvtColor( image, hsv, CV_BGR2HSV );

        if( track_object )	// The user already selected a color to track
        {
            // Get only the hue value of the hsv image
            cvSplit( hsv, hue, sat, val, 0 );

            // If we did not estimate the histogram yet
            if(!estimated_histogram)
            {
                // Calculate a hue histogram for the object
                //float max_val = 0.f;
                //int max_val_bin = 0;
                cvSetImageROI( hue, selection );
                cvSetImageROI( mask, selection );
                cvCalcHist( &hue, hist, 0, mask );
                cvResetImageROI( hue );
                cvResetImageROI( mask );
                estimated_histogram = true;
            }

            // Backproject the hue image (last grabbed image in hue color space) using the estimated histogram
            cvCalcBackProject( &hue, backproject, hist );

            // Create mask of pixels with non-sufficient color saturation
            cvThreshold(sat, s_mask, MIN_HSV_SAT, 255, CV_THRESH_BINARY_INV);

            // Create mask of too dark pixels
            cvThreshold(val, v_mask_dark, MIN_HSV_VAL, 255, CV_THRESH_BINARY_INV );

            // Create mask of too bright pixels
            cvThreshold(val, v_mask_bright, MAX_HSV_VAL, 255, CV_THRESH_BINARY );

            // Combine masks
            cvOr(s_mask, v_mask_dark, sv_mask);
            cvOr(sv_mask, v_mask_bright, sv_mask);

            // Convert IplImage backproject into an OpenCV2 Mat
            cv::Mat backprj(backproject);
            cv::Mat backprj_processed(backproject_processed);

            //Threshold the backprojection into a black-and-white estimate of the pixel belonging to the object
            cv::threshold(backprj, backprj_processed, 200, 255, cv::THRESH_BINARY);

            // Apply the combined mask: set to zero the values of the backprojection that are either not
            // colorful enough or too dark/bright
            cvSet(backproject_processed, cvScalar(0), sv_mask);

            //Also filter the binary estimate based on the pixel neighbour's estimates using erode/dilate:
            // The opening operation cleans the backprojection of small noisy pixels and completes holes in the blob
            // More info: http://docs.opencv.org/2.4/doc/tutorials/imgproc/opening_closing_hats/opening_closing_hats.html
            cv::Mat opening_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9,9), cv::Point(5,5));
            cv::morphologyEx(backprj_processed, backprj_processed, cv::MORPH_CLOSE, opening_kernel);

            // The opencv HoughCircle detector is very sensitive to not-roundness.
            // We smooth the borders to get more stable circle detection
            cv::GaussianBlur( backprj_processed, backprj_processed, Size(13, 13), 4, 4 );

            // Profiling: Show framerate of the loop in bottom right corner
            Float time_difference_between_iters  = gv.curTime - previous_iter_time;
            previous_iter_time = gv.curTime;
            char framerate_msg[255];
            sprintf(framerate_msg,"fps: %.3f", 1.0/time_difference_between_iters);
            putText(img,framerate_msg,cvPoint(230,228), FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(0,0,0));

            // Call the function to find a circle in the image
            if( findCircleFeature(img, backprj_processed, crcl))
            {
                // We compute commands if the robot is in 6DOF mode
                if(gv.x.size()==7)
                {
                    /////////////////////////////////////////////////////////////////
                    // Compute the command to send to the robot using Visual Servoing -> calls a function of cv_control
                    controlRobot(crcl, gv.x, img, command_buf);

                    // Send a new command at a max rate of 1/COMMAND_SEND_MIN_TIME
                    if((previous_command_time + COMMAND_SEND_MIN_TIME) < gv.curTime)
                    {

                        robotCmd(command_buf);

                        Float time_difference_between_commands  = gv.curTime - previous_command_time;
                        previous_command_time = gv.curTime;
                        char framerate_cmds_msg[255];
                        sprintf(framerate_cmds_msg,"fpsc: %.3f", 1.0/time_difference_between_commands);
                        putText(img,framerate_cmds_msg,cvPoint(230,238), FONT_HERSHEY_SIMPLEX, 0.3, CV_RGB(255,0,0));
                    }
                }else{
                    fprintf( stderr, "[cvLoop] ERROR: We do not send any command. The robot is not is controllable in its 6 joints!\n" );
                }
            }

        }
        else	// The user did not select any object yet or a previous selection was cleaned -> we are not tracking
        {
            // Pointing to the wall (quaternion(w,x,y,z):0.50, 0.50, -0.50, 0.50)
            sprintf(command_buf,"jgoto %.4f %.4f %.4f %.4f %.4f %.4f",90.0,-135.0,0.0,0.0,45.0,0.0);
            // Pointing to the window (quaternion(w,x,y,z):0.71, 0.0, 0.71, 0.0)
            //sprintf(command_buf,"jgoto %.4f %.4f %.4f %.4f %.4f %.4f",0.0,-45.0,180.0,0.0,-45.0,0.0);
            robotCmd(command_buf);
        }

        if( select_object && selection.width > 0 && selection.height > 0 )
        {
            cvSetImageROI( image, selection );
            cvXorS( image, cvScalarAll(255), image, 0 );
            cvResetImageROI( image );
        }

        // Display a cross at the desired point of the image
        cvLine(image,cvPoint((int)desired_s_opencvf[0],0),cvPoint((int)desired_s_opencvf[0],image->height),CV_RGB(10,10,10),1);
        cvLine(image,cvPoint(0,(int)desired_s_opencvf[1]),cvPoint(image->width,(int)desired_s_opencvf[1]),CV_RGB(10,10,10),1);

        // Display the images in the windows
        cvShowImage( VS_WINDOW_NAME, image );
        cvShowImage( BACKPROJ_HIST_IMAGE_WINDOW_NAME, backproject);
        cvShowImage( BACKPROJ_HIST_PROC_IMAGE_WINDOW_NAME, backproject_processed);
        cvShowImage( SV_MASK_WINDOW_NAME, sv_mask);

        // Wait 5 ms for a key press and to update the windows
        c = cv::waitKey(5);
        switch( (char) c )
        {
        case 27:	// Escape button: Stop the execution of the CV loop
            cv_running = false;
            break;
        case 'c':	// Char 'c': Clear the currently selected color
            track_object = false;
            estimated_histogram = false;
            break;
        default:
            break;
        }
    }

    // Destroy all windows
    cvDestroyAllWindows();

#ifndef USE_SIMULATED_CAMERA		// Using the real camera
    // Release the grabbing capture
    cvReleaseCapture( &capture );
#endif

    // Release all images
    if(frame!=NULL)
        cvReleaseImage(&frame);
    if(image!=NULL)
        cvReleaseImage(&image);
    if(hsv!=NULL)
        cvReleaseImage(&hsv);
    if(hue!=NULL)
        cvReleaseImage(&hue);
    if(mask!=NULL)
        cvReleaseImage(&mask);
    if(backproject!=NULL)
        cvReleaseImage(&backproject);
    if(message_img!=NULL)
        cvReleaseImage(&message_img);

    // Destroy the mutex
    pthread_mutex_destroy(&mutexGVIn);
}

void cvStop(void)
{
    cv_running = false;
}

void cvSetRobotState(GlobalVariables &gv)
{
    if(!cv_running) return;
    pthread_mutex_lock(&mutexGVIn);
    GV_COPY(gvIn,gv);
    pthread_mutex_unlock(&mutexGVIn);
}

void onMouse( int event, int x, int y, int flags, void* param )
{
    // Return if there is no image
    if( !image )
        return;

    if( image->origin )
        y = image->height - y;

    if( select_object )
    {
        selection.x = MIN(x,origin.x);
        selection.y = MIN(y,origin.y);
        selection.width = selection.x + CV_IABS(x - origin.x);
        selection.height = selection.y + CV_IABS(y - origin.y);

        selection.x = MAX( selection.x, 0 );
        selection.y = MAX( selection.y, 0 );
        selection.width = MIN( selection.width, image->width );
        selection.height = MIN( selection.height, image->height );
        selection.width -= selection.x;
        selection.height -= selection.y;
    }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:	// Click with the left button to begin the selection
        origin = cvPoint(x,y);
        selection = cvRect(x,y,0,0);
        select_object = true;
        break;
    case CV_EVENT_LBUTTONUP:	// Release the left button to finish the selection
        select_object = false;


        if( selection.width > 0 && selection.height > 0 )	// The selection is valid
        {
            track_object = true;
            estimated_histogram = false;
        }
        break;
    }
}





void Sleep(struct timespec& startTimeOfCurrentTimeSlice, float dt)
{

    struct timespec now;
    struct timespec delta;
    struct timespec sleeptime;
    struct timespec remaining_sleeptime;

    clock_gettime(CLOCK_MONOTONIC, &now);

    if (startTimeOfCurrentTimeSlice.tv_sec == 0) { //first time we are called, simply set startTimeOfCurrentTimeSlice and return
        startTimeOfCurrentTimeSlice = now;
        return;
    }

    delta.tv_sec = int(dt);
    delta.tv_nsec =  (dt - delta.tv_sec) * 1000000000;
    //compute the remaining time to sleep:
    sleeptime.tv_sec =  delta.tv_sec - (now.tv_sec - startTimeOfCurrentTimeSlice.tv_sec);
    sleeptime.tv_nsec = delta.tv_nsec - (now.tv_nsec - startTimeOfCurrentTimeSlice.tv_nsec);
    while (sleeptime.tv_nsec < 0) {sleeptime.tv_nsec += 1000000000; sleeptime.tv_sec -=1;} //apply any carry
    if (sleeptime.tv_sec < 0 ) { //don't sleep if we ran out of time!
        cout << "[cvloop] info: could not keep up with the timestep once." << endl;
    } else {
        nanosleep(&sleeptime, &remaining_sleeptime);
    }

    clock_gettime(CLOCK_MONOTONIC, &startTimeOfCurrentTimeSlice); //a new timeslice has begun
    return;
}



