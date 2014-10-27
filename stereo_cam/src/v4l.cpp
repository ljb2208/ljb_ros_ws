/*
 * v4l.cpp
 *
 *  Created on: Mar 15, 2014
 *      Author: lbarnett
 */

#include "../include/v4l.h"


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

using namespace std;


static int xioctl(int fd, int request, void* arg)
{
    int r;
    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

V4lVideo::V4lVideo(const char* dev_name, int width_, int height_, float fps_, bool gs_, int scale_, io_method io)
    : io(io), fd(-1), buffers(0), n_buffers(0), running(false)
{
	width = width_;
	height = height_;
	fps = fps_;
    open_device(dev_name);
    gs = gs_;
    scale = scale_;
    init_device(dev_name, width, height, fps);

    scaleWidth = width / scale;
    scaleHeight = height / scale;

    if (scale > 1)
    	scaler = new image_scaler(scale);
    else
    	scaler = NULL;

    Start();
}

V4lVideo::~V4lVideo()
{
    if(running)
    {
        Stop();
    }

    uninit_device();
    close_device();
}

unsigned V4lVideo::Width() const
{
    return width;
}

unsigned V4lVideo::Height() const
{
    return height;
}

size_t V4lVideo::SizeBytes() const
{
    return image_size;
}

std::string V4lVideo::PixFormat() const
{
    // TODO: compute properly
    return "YUYV422";
}

bool V4lVideo::GrabROSNewest(sensor_msgs::ImagePtr msg) {

	unsigned char* image = (unsigned char*) malloc(height * width * 3);
	unsigned char* convertImage;

	GrabNewest(image, true);

	if (gs) {
		convertImage = (unsigned char*) malloc(height * width);
		convertToGS(image, convertImage);
		msg->encoding = "mono8";
		msg->step = scaleWidth;
	}
	else {
		convertImage = (unsigned char*) malloc(height * width * 3);
		convertToBGR(image, convertImage);
		msg->encoding = "bgr8";
		msg->step = scaleWidth * 3;
	}

	msg->data.resize(scaleHeight * msg->step);
	memcpy((unsigned char*)(&msg->data[0]), convertImage, scaleHeight * msg->step);
	msg->height = scaleHeight;
	msg->width = scaleWidth;
	msg->is_bigendian = false;

	free(image);
	free(convertImage);

	return true;
}

bool V4lVideo::convertToGS(unsigned char* image, unsigned char *convertImage) {

	if (scaler == NULL) {
		yuyv2gs(image, convertImage, height * width);
	} else {
		unsigned char* intConvertImage = (unsigned char*) malloc(height * width);
		yuyv2gs(image, intConvertImage, height * width);

		scaler->gs2thumbnail(intConvertImage, convertImage, height, width);
		free(intConvertImage);
	}

	return true;
}

bool V4lVideo::convertToBGR(unsigned char* image, unsigned char *convertImage) {

	if (scaler == NULL) {
		yuyv2bgr(image, convertImage, height * width);
	} else {
		unsigned char* intConvertImage = (unsigned char*) malloc(height * width * 3);
		yuyv2bgr(image, intConvertImage, height * width);

		scaler->bgr2thumbnail(intConvertImage, convertImage, height, width);
		free(intConvertImage);
	}
	return true;
}

bool V4lVideo::GrabNext( unsigned char* image, bool wait )
{
    for (;;)
    {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO (&fds);
        FD_SET (fd, &fds);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select (fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r)
        {
            if (EINTR == errno)
                continue;

            throw VideoException ("select", strerror(errno));
        }

        if (0 == r)
        {
            throw VideoException("select Timeout", strerror(errno));
        }

        if (ReadFrame(image))
            break;

        /* EAGAIN - continue select loop. */
    }
    return true;
}

bool V4lVideo::GrabNewest( unsigned char* image, bool wait )
{
    // TODO: Implement
    return GrabNext(image,wait);
}

int V4lVideo::ReadFrame(unsigned char* image)
{
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io)
    {
    case IO_METHOD_READ:
        if (-1 == read (fd, buffers[0].start, buffers[0].length))
        {
            switch (errno)
            {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                throw VideoException("read", strerror(errno));
            }
        }

//            process_image(buffers[0].start);
        memcpy(image,buffers[0].start,buffers[0].length);

        break;

    case IO_METHOD_MMAP:
        CLEAR (buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf))
        {
            switch (errno)
            {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                throw VideoException("VIDIOC_DQBUF", strerror(errno));
            }
        }

        assert (buf.index < n_buffers);

//            process_image (buffers[buf.index].start);
        memcpy(image,buffers[buf.index].start,buffers[buf.index].length);


        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
            throw VideoException("VIDIOC_QBUF", strerror(errno));

        break;

    case IO_METHOD_USERPTR:
        CLEAR (buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf))
        {
            switch (errno)
            {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                throw VideoException("VIDIOC_DQBUF", strerror(errno));
            }
        }

        for (i = 0; i < n_buffers; ++i)
            if (buf.m.userptr == (unsigned long) buffers[i].start
                    && buf.length == buffers[i].length)
                break;

        assert (i < n_buffers);

//            process_image ((void *) buf.m.userptr);
        memcpy(image,(void *)buf.m.userptr,buf.length);


        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
            throw VideoException("VIDIOC_QBUF", strerror(errno));

        break;
    }

    return 1;
}

void V4lVideo::Stop()
{
    enum v4l2_buf_type type;

    switch (io)
    {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
            throw VideoException("VIDIOC_STREAMOFF", strerror(errno));

        break;
    }

    running = false;
}

void V4lVideo::Start()
{
    unsigned int i;
    enum v4l2_buf_type type;

    switch (io)
    {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i)
        {
            struct v4l2_buffer buf;

            CLEAR (buf);

            buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory      = V4L2_MEMORY_MMAP;
            buf.index       = i;

            if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                throw VideoException("VIDIOC_QBUF", strerror(errno));
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
            throw VideoException("VIDIOC_STREAMON", strerror(errno));

        break;

    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i)
        {
            struct v4l2_buffer buf;

            CLEAR (buf);

            buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory      = V4L2_MEMORY_USERPTR;
            buf.index       = i;
            buf.m.userptr   = (unsigned long) buffers[i].start;
            buf.length      = buffers[i].length;

            if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                throw VideoException("VIDIOC_QBUF", strerror(errno));
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
            throw VideoException ("VIDIOC_STREAMON", strerror(errno));

        break;
    }

    running = true;
}

void V4lVideo::uninit_device()
{
    unsigned int i;

    switch (io)
    {
    case IO_METHOD_READ:
        free (buffers[0].start);
        break;

    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i)
            if (-1 == munmap (buffers[i].start, buffers[i].length))
                throw VideoException ("munmap");
        break;

    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i)
            free (buffers[i].start);
        break;
    }

    free (buffers);
}

void V4lVideo::init_read(unsigned int buffer_size)
{
    buffers = (buffer*)calloc (1, sizeof (buffer));

    if (!buffers)
    {
        throw VideoException("Out of memory\n");
    }

    buffers[0].length = buffer_size;
    buffers[0].start = malloc (buffer_size);

    if (!buffers[0].start)
    {
        throw VideoException("Out of memory\n");
    }
}

void V4lVideo::init_mmap(const char* dev_name)
{
    struct v4l2_requestbuffers req;

    CLEAR (req);

    req.count               = 4;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_MMAP;

    if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            throw VideoException("does not support memory mapping", strerror(errno));
        }
        else
        {
            throw VideoException ("VIDIOC_REQBUFS", strerror(errno));
        }
    }

    if (req.count < 2)
    {
        throw VideoException("Insufficient buffer memory");
    }

    buffers = (buffer*)calloc(req.count, sizeof(buffer));

    if (!buffers)
    {
        throw VideoException( "Out of memory\n");
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
            throw VideoException ("VIDIOC_QUERYBUF", strerror(errno));

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
            mmap (NULL /* start anywhere */,
                  buf.length,
                  PROT_READ | PROT_WRITE /* required */,
                  MAP_SHARED /* recommended */,
                  fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
            throw VideoException ("mmap");
    }
}

void V4lVideo::init_userp(const char* dev_name, unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;
    unsigned int page_size;

    page_size = getpagesize ();
    buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

    CLEAR (req);

    req.count               = 4;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req))
    {
        if (EINVAL == errno)
        {
            throw VideoException( "Does not support user pointer i/o", strerror(errno));
        }
        else
        {
            throw VideoException ("VIDIOC_REQBUFS", strerror(errno));
        }
    }

    buffers = (buffer*)calloc(4, sizeof(buffer));

    if (!buffers)
    {
        throw VideoException( "Out of memory\n");
    }

    for (n_buffers = 0; n_buffers < 4; ++n_buffers)
    {
        buffers[n_buffers].length = buffer_size;
        buffers[n_buffers].start = memalign (/* boundary */ page_size,
                                   buffer_size);

        if (!buffers[n_buffers].start)
        {
            throw VideoException( "Out of memory\n");
        }
    }
}

void V4lVideo::init_device(const char* dev_name, unsigned iwidth, unsigned iheight, unsigned ifps, unsigned v4l_format, v4l2_field field)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    struct v4l2_streamparm strm;

    unsigned int min;

    if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap))
    {
        if (EINVAL == errno)
        {
            throw VideoException("Not a V4L2 device", strerror(errno));
        }
        else
        {
            throw VideoException ("VIDIOC_QUERYCAP", strerror(errno));
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        throw VideoException("Not a video capture device");
    }

    switch (io)
    {
    case IO_METHOD_READ:
        if (!(cap.capabilities & V4L2_CAP_READWRITE))
        {
            throw VideoException("Does not support read i/o");
        }

        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        if (!(cap.capabilities & V4L2_CAP_STREAMING))
        {
            throw VideoException("Does not support streaming i/o");
        }

        break;
    }


    /* Select video input, video standard and tune here. */

    CLEAR (cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    }
    else
    {
        /* Errors ignored. */
    }

//        {
//            v4l2_frmivalenum fie;
//            CLEAR(fie);
//            fie.width = iwidth;
//            fie.height = iheight;
//            fie.pixel_format = v4l_format;

//            while(1)
//            {

//                if (-1 == xioctl (fd, VIDIOC_ENUM_FRAMEINTERVALS, &fie))
//                    throw VideoException("VIDIOC_ENUM_FRAMEINTERVALS", strerror(errno));

//                cout << fie.type << endl;
//                cout << fie.discrete.numerator << endl;
//                cout << fie.discrete.denominator << endl << endl;
//                fie.index++;
//            }
//        }


    CLEAR (fmt);

    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = iwidth;
    fmt.fmt.pix.height      = iheight;
    fmt.fmt.pix.pixelformat = v4l_format;
    fmt.fmt.pix.field       = field;

    if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt))
        throw VideoException("VIDIOC_S_FMT", strerror(errno));

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    /* Note VIDIOC_S_FMT may change width and height. */
    width = fmt.fmt.pix.width;
    height = fmt.fmt.pix.height;
    image_size = fmt.fmt.pix.sizeimage;


    CLEAR(strm);
    strm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    strm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    strm.parm.capture.timeperframe.numerator = 1;
    strm.parm.capture.timeperframe.denominator = ifps;

    if (-1 == xioctl (fd, VIDIOC_S_PARM, &fmt))
        throw VideoException("VIDIOC_S_PARM", strerror(errno));

    fps = (float)strm.parm.capture.timeperframe.denominator / strm.parm.capture.timeperframe.numerator;

    switch (io)
    {
    case IO_METHOD_READ:
        init_read (fmt.fmt.pix.sizeimage);
        break;

    case IO_METHOD_MMAP:
        init_mmap (dev_name );
        break;

    case IO_METHOD_USERPTR:
        init_userp (dev_name, fmt.fmt.pix.sizeimage);
        break;
    }
}

void V4lVideo::close_device()
{
    if (-1 == close (fd))
        throw VideoException("close");

    fd = -1;
}

void V4lVideo::open_device(const char* dev_name)
{
    struct stat st;

    if (-1 == stat (dev_name, &st))
    {
        throw VideoException("Cannot stat device", strerror(errno));
    }

    if (!S_ISCHR (st.st_mode))
    {
        throw VideoException("Not device");
    }

    fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd)
    {
        throw VideoException("Cannot open device");
    }
}

void V4lVideo::yuyv2bgr(unsigned char *YUV, unsigned char *BGR, int NumPixels) {

  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
      y0 = (unsigned char) YUV[i + 0];
      u = (unsigned char) YUV[i + 1];
      y1 = (unsigned char) YUV[i + 2];
      v = (unsigned char) YUV[i + 3];
      yuv2bgr(y0, u, v, &r, &g, &b);
      BGR[j + 0] = b;
      BGR[j + 1] = g;
      BGR[j + 2] = r;
      yuv2bgr(y1, u, v, &r, &g, &b);
      BGR[j + 3] = b;
      BGR[j + 4] = g;
      BGR[j + 5] = r;
    }
}

void V4lVideo::yuv2bgr(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b) {
	int r1, g1, b1;

	// replaces floating point coefficients
	int c = y-16, d = u - 128, e = v - 128;

	// Conversion that avoids floating point
	r1 = (298 * c           + 409 * e + 128) >> 8;
	g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
	b1 = (298 * c + 516 * d           + 128) >> 8;

	// Computed values may need clipping.
	if (r1 > 255) r1 = 255;
	if (g1 > 255) g1 = 255;
	if (b1 > 255) b1 = 255;

	if (r1 < 0) r1 = 0;
	if (g1 < 0) g1 = 0;
	if (b1 < 0) b1 = 0;

	*r = r1 ;
	*g = g1 ;
	*b = b1 ;
}

void V4lVideo::yuyv2gs(unsigned char *YUV, unsigned char *GS, int NumPixels) {
	int i, j;
	  unsigned char y0, y1, u, v;
	  unsigned char gs;

	  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 2)
	    {
	      y0 = (unsigned char) YUV[i + 0];
	      y1 = (unsigned char) YUV[i + 2];
	      yuv2gs(y0, &gs);
	      GS[j + 0] = gs;
	      yuv2gs(y1, &gs);
	      GS[j + 1] = gs;
	    }
}

void V4lVideo::yuv2gs(int y, unsigned char *gs) {
	*gs = y;
}
