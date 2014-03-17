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

V4lVideo::V4lVideo(const char* dev_name, int width_, int height_, float fps_, io_method io)
    : io(io), fd(-1), buffers(0), n_buffers(0), running(false)
{
	width = width_;
	height = height_;
	fps = fps_;
    open_device(dev_name);
    init_device(dev_name, width, height, fps);
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

	//msg->height = height;
	//msg->width = width;
	//msg->step = width * 3;
	unsigned char* image = (unsigned char*) malloc(height * width * 3);
	unsigned char* rgbimage = (unsigned char*) malloc(height * width * 3);

	GrabNewest(image, true);

	yuyv2rgb(image, rgbimage, height * width);

	free(image);

	msg->height = height;
	msg->width = width;
	msg->step = width * 3;
	msg->data.resize(height * width * 3);
	msg->encoding = "rgb8";
	msg->is_bigendian = false;

	memcpy((unsigned char*)(&msg->data[0]), rgbimage, height * width * 3);

	free(rgbimage);


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

const unsigned char uchar_clipping_table[] = {
  0,0,0,0,0,0,0,0, // -128 - -121
  0,0,0,0,0,0,0,0, // -120 - -113
  0,0,0,0,0,0,0,0, // -112 - -105
  0,0,0,0,0,0,0,0, // -104 -  -97
  0,0,0,0,0,0,0,0, //  -96 -  -89
  0,0,0,0,0,0,0,0, //  -88 -  -81
  0,0,0,0,0,0,0,0, //  -80 -  -73
  0,0,0,0,0,0,0,0, //  -72 -  -65
  0,0,0,0,0,0,0,0, //  -64 -  -57
  0,0,0,0,0,0,0,0, //  -56 -  -49
  0,0,0,0,0,0,0,0, //  -48 -  -41
  0,0,0,0,0,0,0,0, //  -40 -  -33
  0,0,0,0,0,0,0,0, //  -32 -  -25
  0,0,0,0,0,0,0,0, //  -24 -  -17
  0,0,0,0,0,0,0,0, //  -16 -   -9
  0,0,0,0,0,0,0,0, //   -8 -   -1
  0,1,2,3,4,5,6,7,
  8,9,10,11,12,13,14,15,
  16,17,18,19,20,21,22,23,
  24,25,26,27,28,29,30,31,
  32,33,34,35,36,37,38,39,
  40,41,42,43,44,45,46,47,
  48,49,50,51,52,53,54,55,
  56,57,58,59,60,61,62,63,
  64,65,66,67,68,69,70,71,
  72,73,74,75,76,77,78,79,
  80,81,82,83,84,85,86,87,
  88,89,90,91,92,93,94,95,
  96,97,98,99,100,101,102,103,
  104,105,106,107,108,109,110,111,
  112,113,114,115,116,117,118,119,
  120,121,122,123,124,125,126,127,
  128,129,130,131,132,133,134,135,
  136,137,138,139,140,141,142,143,
  144,145,146,147,148,149,150,151,
  152,153,154,155,156,157,158,159,
  160,161,162,163,164,165,166,167,
  168,169,170,171,172,173,174,175,
  176,177,178,179,180,181,182,183,
  184,185,186,187,188,189,190,191,
  192,193,194,195,196,197,198,199,
  200,201,202,203,204,205,206,207,
  208,209,210,211,212,213,214,215,
  216,217,218,219,220,221,222,223,
  224,225,226,227,228,229,230,231,
  232,233,234,235,236,237,238,239,
  240,241,242,243,244,245,246,247,
  248,249,250,251,252,253,254,255,
  255,255,255,255,255,255,255,255, // 256-263
  255,255,255,255,255,255,255,255, // 264-271
  255,255,255,255,255,255,255,255, // 272-279
  255,255,255,255,255,255,255,255, // 280-287
  255,255,255,255,255,255,255,255, // 288-295
  255,255,255,255,255,255,255,255, // 296-303
  255,255,255,255,255,255,255,255, // 304-311
  255,255,255,255,255,255,255,255, // 312-319
  255,255,255,255,255,255,255,255, // 320-327
  255,255,255,255,255,255,255,255, // 328-335
  255,255,255,255,255,255,255,255, // 336-343
  255,255,255,255,255,255,255,255, // 344-351
  255,255,255,255,255,255,255,255, // 352-359
  255,255,255,255,255,255,255,255, // 360-367
  255,255,255,255,255,255,255,255, // 368-375
  255,255,255,255,255,255,255,255, // 376-383
};
const int clipping_table_offset = 128;

/** Clip a value to the range 0<val<255. For speed this is done using an
 * array, so can only cope with numbers in the range -128<val<383.
 */
unsigned char V4lVideo::CLIPVALUE(int val)
{
  // Old method (if)
/*   val = val < 0 ? 0 : val; */
/*   return val > 255 ? 255 : val; */

  // New method (array)
  return uchar_clipping_table[val+clipping_table_offset];
}

void V4lVideo::YUV2RGB(const unsigned char y,
        const unsigned char u,
        const unsigned char v,
        unsigned char* r,
        unsigned char* g,
        unsigned char* b)
{
  const int y2=(int)y;
  const int u2=(int)u-128;
  const int v2=(int)v-128;
  //std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;


  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  int r2 = y2 + ( (v2*37221) >> 15);
  int g2 = y2 - ( ((u2*12975) + (v2*18949)) >> 15 );
  int b2 = y2 + ( (u2*66883) >> 15);
  //std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;


  // Cap the values.
  *r=CLIPVALUE(r2);
  *g=CLIPVALUE(g2);
  *b=CLIPVALUE(b2);
}


void V4lVideo::yuyv2rgb(unsigned char *YUV, unsigned char *RGB, int NumPixels) {
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6)
    {
      y0 = (unsigned char) YUV[i + 0];
      u = (unsigned char) YUV[i + 1];
      y1 = (unsigned char) YUV[i + 2];
      v = (unsigned char) YUV[i + 3];
      YUV2RGB (y0, u, v, &r, &g, &b);
      RGB[j + 0] = r;
      RGB[j + 1] = g;
      RGB[j + 2] = b;
      YUV2RGB (y1, u, v, &r, &g, &b);
      RGB[j + 3] = r;
      RGB[j + 4] = g;
      RGB[j + 5] = b;
    }
}
