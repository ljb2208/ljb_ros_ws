/*
 * v4ldriver.h
 *
 *  Created on: Mar 14, 2014
 *      Author: lbarnett
 */

#ifndef V4LDRIVER_H_
#define V4LDRIVER_H_


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>

#include <linux/videodev2.h>

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

enum io_method {
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR,
};

struct buffer {
	void *start;
	size_t length;
};


class v4l_driver {
public:
	v4l_driver(const char* dev);
	virtual ~v4l_driver();

	int read_frame(void);
	void close_device(void);
	int open_device(void);

private:
	const char   *dev_name;
	enum         io_method io;
	int          fd;
	buffer       *buffers;
	unsigned int n_buffers;
	int          process_buf;
	int          frame_count;
	int          set_format;
	unsigned int width;
	unsigned int height;
	unsigned int fps;
	unsigned int timeout;
	unsigned int timeouts_max;

	// Added to track processing a subset of the captures
	int captured_count;
	int processed_count;
	int capture_offset;

	/* Allowed formats: V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_MJPEG, V4L2_PIX_FMT_H264
	*  The default will not be used unless the width and/or height is specified
	*  but the user does not specify a pixel format */
	unsigned int pixel_format;

	int xioctl(int fh, int request, void *arg);
	void process_image(const void *p, int size);
	void grab_frames(void);
	void stop_capturing(void);
	void start_capturing(void);
	void uninit_device(void);
	void init_read(unsigned int buffer_size);
	void init_mmap(void);
	void init_userp(unsigned int buffer_size);
	void init_device(void);
	void errno_exit(const char* msg);

};

#endif /* V4LDRIVER_H_ */
