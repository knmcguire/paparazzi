/**
 * @file modules/edgeflow_histogram/edgeflow.c
 *
 * Get images, calculates edge feature flow and sends them
 *
 * Works on Linux platforms
 */

//own header
#include "edgeflow.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>


// Video
#include "lib/v4l/v4l2.h"
#include "lib/vision/image.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "udp_socket.h"

// Threaded computer vision
#include <pthread.h>


// Default broadcast IP
#ifndef VIDEO_SOCK_IP
#define VIDEO_SOCK_IP "192.168.1.255"
#endif

// Output socket can be defined from an offset
#ifdef VIDEO_SOCK_OUT_OFFSET
#define VIDEO_SOCK_OUT (5000+VIDEO_SOCK_OUT_OFFSET)
#endif

#ifndef VIDEO_SOCK_OUT
#define VIDEO_SOCK_OUT 5000
#endif

#ifndef VIDEO_SOCK_IN
#define VIDEO_SOCK_IN 4999
#endif

// Downsize factor for video stream
#ifndef VIDEO_DOWNSIZE_FACTOR
#define VIDEO_DOWNSIZE_FACTOR 4
#endif

// From 0 to 99 (99=high)
#ifndef VIDEO_QUALITY_FACTOR
#define VIDEO_QUALITY_FACTOR 50
#endif

// Frame Per Seconds
#ifndef VIDEO_FPS
#define VIDEO_FPS 4.
#endif



void run_edgeflow(void) {}

pthread_t computervision_thread;

volatile uint8_t computervision_thread_status = 0;
volatile uint8_t computer_vision_thread_command = 0;
void *computervision_thread_main(void *data);
void *computervision_thread_main(void *data)
{

    // Video Input
    struct vid_struct vid;
    vid.device = (char *)"/dev/video2";
    vid.w = 1280;
    vid.h = 720;
    vid.n_buffers = 4;
    if (video_init(&vid) < 0) {
        printf("Error initialising video\n");
        computervision_thread_status = -1;
        return 0;
    }

    // Video Grabbing
    struct img_struct *img_new = video_create_image(&vid);

    // Video Resizing
    uint8_t quality_factor = VIDEO_QUALITY_FACTOR;
    uint8_t dri_jpeg_header = 0;
    int microsleep = (int)(1000000. / VIDEO_FPS);

    struct img_struct small;
    small.w = vid.w / VIDEO_DOWNSIZE_FACTOR;
    small.h = vid.h / VIDEO_DOWNSIZE_FACTOR;
    small.buf = (uint8_t *)malloc(small.w * small.h * 2);
    // Video Compression
        uint8_t *jpegbuf = (uint8_t *)malloc(vid.h * vid.w * 2);

        // Network Transmit
        struct UdpSocket *vsock;
        vsock = udp_socket(VIDEO_SOCK_IP, VIDEO_SOCK_OUT, VIDEO_SOCK_IN, FMS_BROADCAST);

        // Create SPD file and make folder if necessary
        FILE *sdp;
        if (system("mkdir -p /data/video/sdp") == 0) {
            sdp = fopen("/data/video/sdp/x86_config-mjpeg.sdp", "w");
            if (sdp != NULL) {
                fprintf(sdp, "v=0\n");
                fprintf(sdp, "m=video %d RTP/AVP 26\n", (int)(VIDEO_SOCK_OUT));
                fprintf(sdp, "c=IN IP4 0.0.0.0");
                fclose(sdp);
            }
        }

        // file index (search from 0)
        int file_index = 0;

        // time
        struct timeval last_time;
        gettimeofday(&last_time, NULL);

        while (computer_vision_thread_command > 0) {


            // compute usleep to have a more stable frame rate
            struct timeval time;
            gettimeofday(&time, NULL);
            int dt = (int)(time.tv_sec - last_time.tv_sec) * 1000000 + (int)(time.tv_usec - last_time.tv_usec);
            if (dt < microsleep) { usleep(microsleep - dt); }
            last_time = time;



            // Grab new frame
            video_grab_image(&vid, img_new);

            // Resize
            resize_uyuv(img_new, &small, VIDEO_DOWNSIZE_FACTOR);
            //_______________________________
            //VISION CODE!!






            //VISION CODE!!!
            //________________________________
            // JPEG encode the image:
                    uint32_t image_format = FOUR_TWO_TWO;  // format (in jpeg.h)
                    uint8_t *end = encode_image(small.buf, jpegbuf, quality_factor, image_format, small.w, small.h, dri_jpeg_header);
                    uint32_t size = end - (jpegbuf);

                    // Send image with RTP
                    // printf("Sending an image ...%u\n", size);
                    send_rtp_frame(
                                vsock,            // UDP
                                jpegbuf, size,    // JPEG
                                small.w, small.h, // Img Size
                                0,                // Format 422
                                quality_factor,   // Jpeg-Quality
                                dri_jpeg_header,  // DRI Header
                                0                // 90kHz time increment
                                );
                    // Extra note: when the time increment is set to 0,
                    // it is automaticaly calculated by the send_rtp_frame function
                    // based on gettimeofday value. This seems to introduce some lag or jitter.
                    // An other way is to compute the time increment and set the correct value.
                    // It seems that a lower value is also working (when the frame is received
                    // the timestamp is always "late" so the frame is displayed immediately).
                    // Here, we set the time increment to the lowest possible value
                    // (1 = 1/90000 s) which is probably stupid but is actually working.
                    // printf("Does this run?\n ");

                }
                printf("Thread Closed\n");
                video_close(&vid);
                computervision_thread_status = -100;
                return 0;

}

start_edgeflow(void)
{
    computer_vision_thread_command = 1;
    int rc = pthread_create(&computervision_thread, NULL, computervision_thread_main, NULL);
    if (rc) {
        printf("ctl_Init: Return code from pthread_create(mot_thread) is %d\n", rc);
    }

}

stop_edgeflow(void)
{

        computer_vision_thread_command = 0;
}
