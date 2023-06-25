#include <stdio.h>
#include <getopt.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include<signal.h>


#ifdef ANDROID
#include <termios.h>
#else
#include <sys/termios.h>
#endif

int iii = 0;
#define LOG_ERR(fmt, arg...) printf ("\033[0;32;31m""FAILED""\033[m :  "fmt"\n" , ## arg)
#define LOG_OK(fmt, arg...) printf ("\033[0;32;32m""OK    ""\033[m :  "fmt"\n" , ## arg)
#define INFO_PRINT(tag, cmd, size) do{printf("%s  : ", tag);\
				for ( iii = 0; iii < size; iii++){printf(" 0x%02x", cmd[iii]);}	printf("\n");}while(0);

#define BAUDRATE B115200


static void sig_alrm(int signal)
{
	if ( signal == SIGALRM ) {
		 LOG_ERR("get version timeout (10s)");
	}
	exit(-1);
}

static int recv_event(int fd, u_int8_t* buf, int size)
{
	int remain, r;
	int count = 0;

	if (size <= 0)
		return -1;

	alarm(10);	 			
	if(signal(SIGALRM, sig_alrm) == SIG_ERR){  
		   exit(-1);	
	}
	
	while (1) {
		r = read(fd, buf, 1);
		if (r <= 0)
			return -1;
		if (buf[0] == 0x04) {
			break;
		}
	}
	count++;

	while (count < 3) {
		r = read(fd, buf + count, 3 - count);
		if (r <= 0)
			return -1;
		count += r;
	}

	if (buf[2] < (size - 3))
		remain = buf[2];
	else
		remain = size - 3;

	while ((count - 3) < remain) {
		r = read(fd, buf + count, remain - (count - 3));
		if (r <= 0)
			return -1;
		count += r;
	}

	return count;
}

static int send_cmd(int fd, u_int8_t *cmd, int size)
{
	int ret;	
	ret = write(fd, cmd, size);
	if (ret != size) {
		return -1;
	}

	if (ret < 0)
		return -1;

	return 0;
}

static int get_chip_version(int fd)
{
	int err;
	int count;
	u_int8_t cmd[5] = {0x01, 0x00, 0xfc, 0x01, 0x19};
	u_int8_t rsp[50];

	INFO_PRINT("send ",cmd, 5);
	err = send_cmd(fd, cmd, 5);
	if (err < 0) {
		return err;
	}

	count = recv_event(fd, rsp, 50);
	if (count > 0){
		INFO_PRINT("recv ", rsp, count);
		LOG_OK("");
	}else {	
		LOG_ERR("");
		return -1;
	}	
	
	return 0;
}


static int get_chip_addr(int fd)
{
	int err;
	int count;
	u_int8_t cmd[6] = {0x01, 0x09, 0xfc, 0x02, 0x00, 0x00};
	u_int8_t rsp[50];

	INFO_PRINT("send ",cmd, 6);
	err = send_cmd(fd, cmd, 6);
	if (err < 0) {
		return err;
	}

	count = recv_event(fd, rsp, 50);
	if (count > 0){
		INFO_PRINT("recv ", rsp, count);
		LOG_OK("");
	}else {	
		LOG_ERR("");
		return -1;
	}	
	
	return 0;
}



static int init_uart(char *dev)
{
	struct termios ti;
	int fd;
	unsigned long flags = 0;

	fd = open(dev, O_RDWR | O_NOCTTY);
	if (fd < 0) {
		LOG_ERR("uart open");
		return -1;
	}

	tcflush(fd, TCIOFLUSH);

	if (tcgetattr(fd, &ti) < 0) {
		LOG_ERR("uart tcgetattr");
		return -1;
	}

	cfmakeraw(&ti);

	ti.c_cflag |= CLOCAL;
	ti.c_cflag |= CRTSCTS;

	if (tcsetattr(fd, TCSANOW, &ti) < 0) {
		LOG_ERR("uart tcsetattr");
		close(fd);
		return -1;
	}

	if (cfsetospeed(&ti, BAUDRATE) < 0)
		return -errno;

	if (cfsetispeed(&ti, BAUDRATE) < 0)
		return -errno;

	if (tcsetattr(fd, TCSANOW, &ti) < 0)
		return -errno;
	
		
	tcflush(fd, TCIOFLUSH);

	if (ioctl(fd, TIOCMGET, &flags) < 0) {
		LOG_ERR("uart TIOCMGET");
		close(fd);
		return -1;
	}

	flags |= TIOCM_RTS;
	if (ioctl(fd, TIOCMSET, &flags) < 0) {
		LOG_ERR("uart TIOCMSET");
		close(fd);
		return -1;
	}

	tcflush(fd, TCIOFLUSH);

	return fd;
}

static void usage( char *progname )
{
	printf("Usage: %s [options] \n", progname);
	printf( "  -v, --version:\t\t get chip version( example: qlbt -v /dev/*tty* ) \n" );
	printf( "  -a, --address:\t\t get chip address( example: qlbt -a /dev/*tty* ) \n" );	
	exit( 0 );
}


int main(int argc, char *argv[])
{
	int	command = 0;
	int fd = 0;
	
	struct option	longopts[] = {
		{ "version", 1, NULL, 'v' },
		{ "address", 1, NULL, 'a' },
		{ "help", 0, NULL, 'h' },		
	};

	if ( argc == 1 ) {
		usage( argv[0] );
		return 0;
	}

	while ( (command = getopt_long( argc, argv, "v:a:h", longopts, NULL ) ) != -1 ) {
		switch ( command ) {
		case 'v':
			if ((fd = init_uart(optarg)) < 0) {
				exit(1);
			}
		
			get_chip_version(fd);
			close(fd);
			return 0;
			
		case 'h':
		default:
			usage( argv[0] );
			break;
		}
	}
	
    usage( argv[0] );
	return 0;
}
