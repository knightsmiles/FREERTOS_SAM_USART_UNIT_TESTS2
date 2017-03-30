/**********************************************************
* serial_port.c
* This file is some Linux serial port code by Scott Wimberley
***********************************************************/
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include "packet.h"

char CRLFString[] = "\r\n";
/**********************************************************
* int open_port(char *port)
* Open the requested serial port. The string pointer arg
* contains the port requested.
***********************************************************/
int open_port(char *port)
{
    int fd; /* File descriptor for the port */
    fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY); /* O_NDELAY = DCD is dont care */

    if (fd < 0)
    {
        /* Could not open the port. */
        perror("open_port: Unable to open port\n ");
    }
    else
        fcntl(fd, F_SETFL, 0); /* blocking reads */

    return (fd);
}

/**********************************************************
* int set_options(int fd)
*
* Set up the serial port with the following options:
*
* 8N1, raw mode, software flow control, 115200 baud
***********************************************************/
int set_options(int fd)
{
    struct termios options;

    /* get the current options */
    tcgetattr(fd, &options);

    /* parity No parity (8N1) */
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;



    /* set raw input, 1 second timeout, software flow control  */
    options.c_cflag |= (CLOCAL | CREAD);
    /* I don't yet know if I need the next line */
    /* options.c_cflag &= ~(CRTSCTS); */
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* raw input */
    options.c_oflag &= ~OPOST; /* raw output */
    options.c_iflag |= (IXON | IXOFF | IXANY);
    options.c_cc[VMIN] = 0; /* minimum number of characters to read */
    options.c_cc[VTIME] = 10; /* Time to wait for data (tenths of seconds) */

    cfsetispeed(&options, 115200); /* set input baud -- Linux function */
    cfsetospeed(&options, 115200); /* set output baud -- Linux function */
    /* set the options */
    tcsetattr(fd, TCSANOW, &options);

	return 0;
}


int send_str_to_port(int fd, char *str, int count)
{
    int ret;
    size_t actual = 0;

    char *buffer;
	buffer = str;
	/* send the string */
	do
	{
	ret = write(fd,buffer , count - actual);

	if (ret < 0)
	{
	    if (errno == EAGAIN) /* no data yet, try again */
	    {
	        usleep(1000);
	        continue;
	    }

	}

	actual  += ret;
	buffer  += ret;
	} while (actual < count);

	return actual;
}

int send_pkt_to_port(int fd, packet_t *packet)
{
    int ret;
    size_t actual = 0;
    size_t count = sizeof(packet_t);

    unsigned char *buffer;
	buffer = packet;
	/* send the string */
	do
	{
	ret = write(fd,buffer , count - actual);

	if (ret < 0)
	{
	    if (errno == EAGAIN) /* no data yet, try again */
	    {
	        usleep(1000);
	        continue;
	    }

	}

	actual  += ret;
	buffer  += ret;
	} while (actual < count);

	return actual;
}



/*
* Consider the following as an option to this function
*
* int fd;
* int bytes;
* ioctl(fd, FIONREAD, &bytes);
*/
/*
int serial_read(int fd, char *str, int num)
{
    struct timeval timeout;
	fd_set readfds;
	int ret;
	int	actual;

	FD_ZERO(&readfds);
	FD_SET(fd, &readfds);
	timeout.tv_sec = 1; *//* if no result within 1 second */
/*	timeout.tv_usec = 0;
	actual = 0; */
	/* Wait for input to become ready or until the time out; the first parameter is
	 1 more than the largest file descriptor in any of the sets  */
/*	if (select(fd + 1, &readfds, NULL, NULL, &timeout) == 1)
	  { */
		  /* fd is ready for reading */
		/*  actual = read(fd, str, MAXLINELENGTH);
	  }
	ret = chk_err(actual, str);
	if(ret < 0)
		return -1; *//* error */

/*    return actual;

}
*/

int serial_read(int fd, char *str)
{
	int bytes;
	int actual;

	ioctl(fd, FIONREAD, &bytes); // get the number of bytes in the input buffer
	if(bytes > 0)
	  {
		actual = read(fd, str, bytes);
		return actual;
	  }

	return bytes;
}

int wait_reply(int fd, unsigned char *pak)
{
	int  nbytes = 0;       /* Number of bytes read */
	nbytes = serial_read(fd, pak);
	if(nbytes < 0)
		return -1; /* error */

	return nbytes;
}

void ms_delay(int num)
{
	int x;
	for(x = 0; x < num; x++)
		usleep(1000);
}

/* reads from keypress, doesn't echo */
int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}
