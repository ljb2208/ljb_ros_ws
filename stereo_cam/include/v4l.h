/*
 * v4l.h
 *
 *  Created on: Mar 15, 2014
 *      Author: lbarnett
 */

#ifndef V4L_H_
#define V4L_H_

#include <asm/types.h>
#include <linux/videodev2.h>
#include <string>
#include <image_transport/image_transport.h>
#include <cv.h>

typedef enum
{
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
} io_method;

struct buffer
{
    void*  start;
    size_t length;
};

struct VideoException : std::exception
{
    VideoException(std::string str) : desc(str) {}
    VideoException(std::string str, std::string detail)
    {
        desc = str + "\n\t" + detail;
    }
    ~VideoException() throw() {}
    const char* what() const throw()
    {
        return desc.c_str();
    }
    std::string desc;
};



class V4lVideo
{
public:
    V4lVideo(const char* dev_name, int width_, int height_, float fps_,bool gs_, io_method io = IO_METHOD_MMAP);
    ~V4lVideo();

    //! Implement VideoSource::Start()
    void Start();

    //! Implement VideoSource::Stop()
    void Stop();

    unsigned Width() const;

    unsigned Height() const;

    size_t SizeBytes() const;

    std::string PixFormat() const;

    bool GrabNext( unsigned char* image, bool wait = true );

    bool GrabNewest( unsigned char* image, bool wait = true );

    bool GrabROSNewest(sensor_msgs::ImagePtr msg);


protected:
    int ReadFrame(unsigned char* image);
    void Mainloop();

    void init_read(unsigned int buffer_size);
    void init_mmap(const char* dev_name);
    void init_userp(const char* dev_name, unsigned int buffer_size);

    void init_device(const char* dev_name, unsigned iwidth, unsigned iheight, unsigned ifps, unsigned v4l_format = V4L2_PIX_FMT_YUYV, v4l2_field field = V4L2_FIELD_INTERLACED);
    void uninit_device();

    void open_device(const char* dev_name);
    void close_device();


    io_method io;
    int       fd;
    buffer*   buffers;
    unsigned  int n_buffers;
    bool running;
    unsigned width;
    unsigned height;
    float fps;
    size_t image_size;
    bool gs;
    bool thumbnail;
    int scaling;

    void yuv2bgr(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b);
    void yuyv2bgr(unsigned char *YUV, unsigned char *RGB, int NumPixels);
    void yuyv2gs(unsigned char *YUV, unsigned char *GS, int NumPixels);
    void yuv2gs(int y, unsigned char *gs);
};


#endif /* V4L_H_ */
