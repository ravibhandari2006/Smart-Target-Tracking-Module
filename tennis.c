#include <stdio.h>
#include <err.h>
#include <X11/keysym.h>
#include <sys/timeb.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include<time.h>
#include<stdlib.h>
#include <stdint.h>   /* Standard types */
#include <string.h>   /* String function definitions */
#include <unistd.h>   /* UNIX standard function definitions */
#include <fcntl.h>    /* File control definitions */
#include <errno.h>    /* Error number definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>

FILE *fp;
char dat[20];
int fd = 0;
int baudrate = B115200;
int framectr=1;

int serialport_write(int fd, const char* str)
{
    int len = strlen(str);
    int n = write(fd, str, len);
    if( n!=len ) 
      return -1;
    return n;
}

int serialport_init(const char* serialport, int baud)
{
    struct termios toptions;
    int fd;
    fd = open(serialport, O_RDWR | O_NOCTTY);
    if (fd == -1) 
    {
      perror("init_serialport: Unable to open port ");
      return -1;
    }
    if (tcgetattr(fd, &toptions) < 0) 
    {
      perror("init_serialport: Couldn't get term attributes");
      return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) 
    {
      case 4800:   brate=B4800;   break;
      case 9600:   brate=B9600;   break;
      case 19200:  brate=B19200;  break;
      case 38400:  brate=B38400;  break;
      case 57600:  brate=B57600;  break;
      case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);
    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) 
    {
      perror("init_serialport: Couldn't set term attributes");
      return -1;
    }
    return fd;
}

void print_time(char *label)
{
	static struct timeb prev = {0,0};
	struct timeb cur;
	double diff = 0;
	ftime(&cur);
	if (prev.time) {
		diff  =  cur.time    - prev.time;
		diff += (cur.millitm - prev.millitm)/1000.0;
	}
	fprintf(stderr, "%30s  start = %d.%-3hu (+%5.3f)\n",
		label, (int)cur.time, cur.millitm, diff);
	prev = cur;
}

void cvOpen(const CvArr *src, CvArr *dst, IplConvKernel *element)
{
	cvErode (src, dst, element, 1);
	cvDilate(src, dst, element, 1);
}

void cvClose(const CvArr *src, CvArr *dst, IplConvKernel *element)
{
	cvDilate(src, dst, element, 1);
	cvErode (src, dst, element, 1);
}

IplImage *process(IplImage **_img)
{
	fprintf(stderr, "Processing image:\n");
	IplImage *img = *_img;

	/* Convert to HSV */
	print_time("Converting to HSV");
	CvSize size = cvGetSize(img);
	IplImage *hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
	cvCvtColor(img, hsv, CV_BGR2HSV);

	/* Generate mask */
	CvMat *mask = cvCreateMat(size.height, size.width, CV_8UC1);
	//cvInRangeS(hsv, cvScalar(0.11*256, 0.60*256, 0.20*256, 0),cvScalar(0.14*256, 1.00*256, 1.00*256, 0), mask);
	cvInRangeS(hsv, cvScalar(0,0.6*256,0.6*256,0),cvScalar(0.21*256,256,256,0), mask);
	cvReleaseImage(&hsv);

	/* Perform morphological ops */
	print_time("Performing morphologies");
	IplConvKernel *se21 = cvCreateStructuringElementEx(21, 21, 10, 10, CV_SHAPE_RECT, NULL);
	IplConvKernel *se11 = cvCreateStructuringElementEx(11, 11, 5,  5,  CV_SHAPE_RECT, NULL);
	cvClose(mask, mask, se21);
	cvOpen(mask, mask, se11);
	cvReleaseStructuringElement(&se21);
	cvReleaseStructuringElement(&se11);

	/* Hough transform */
	IplImage *hough_in = cvCreateImage(size, 8, 1);
	cvCopy(mask, hough_in, NULL);
	int rows=size.height;
	int cols=size.width;
	int j,k;
	int breakflag=0;
	for(j=0;j<rows;j++) {
	  for(k=0;k<cols;k++)
	    {
		CvScalar val=cvGet2D(hough_in,j,k);
		if(val.val[0]==255)
		{
		  sprintf(dat,"%d-%d:",k,j);
		  int rc = serialport_write(fd, dat);
	          if(rc==-1) return 0;
		  fprintf(fp,"%d %d\n",k,j);	
		  breakflag=1;
		  break;
		}
	    }
	  if(breakflag)
	    break;
	}
	return hough_in;
}

void test_live(CvCapture *cam)
{
	fp=fopen("output.txt","w");
	//initiate serial port
    	fd = serialport_init("/dev/ttyACM0", baudrate);
    	if(fd==-1) return;
    	usleep(3000 * 1000 );
	while (1) {
		if(framectr)
		{
		  IplImage *img=cvQueryFrame(cam);
		  IplImage *out = process(&img);
		  cvShowImage("img", img);
		  cvShowImage("out", out);
		  if (cvWaitKey(10) == XK_q) {
			fclose(fp);
			close(fd);
			return;
		  }
		  cvReleaseImage(&out);
		}
		framectr=(framectr+1)%2;
    		usleep(1000 * 70 );
	}
}

int main(int argc, char **argv)
{
	/* create windows */
	cvNamedWindow("out",0);
	cvMoveWindow("out",200,200);
	cvNamedWindow("img",0);
	cvMoveWindow("img",200,200);
	cvInitSystem(argc, argv);
	CvCapture *cam = cvCreateCameraCapture(1);
	test_live(cam);
	return 0;
}
