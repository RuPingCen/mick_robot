#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <unistd.h> /* UNIX standard function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

/*
 * @brief Open serial port with the given device name
 *
 * @return The file descriptor on success or -1 on error.
 */
int open_port(char *port_device)
{
    int fd; /* File descriptor for the port */

    fd = open(port_device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("open_port: Unable to open /dev/ttyS0 - ");
    }
    else
        fcntl(fd, F_SETFL, 0);

    return (fd);
}

int main()
{
    struct termios options;

    int fd=open_port("/dev/ttyACM0");
    if(fd==-1){
        return -1;
    }

    tcgetattr(fd, &options);


    //Set the baud rates to 38400...
    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);


    //Enable the receiver and set local mode...
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE; /* Mask the character size bits */
    options.c_cflag |= CS8; /* Select 8 data bits */

    //No parity
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;


    //Set the new options for the port...
    tcsetattr(fd, TCSANOW, &options);

    int ax,ay,az,gx,gy,gz;
    char buf[1024];
    char* pos=buf;

    while(1){
        ssize_t n=read(fd, pos, 1);
        if(n==1){
            if(*pos=='\n' ){
                *(pos+1)=0;
                sscanf(buf,"%d,%d,%d,%d,%d,%d", &ax, &ay, &az, &gx, &gy, &gz);
                printf("acc/gyro: %d %d %d %d %d %d\n", ax, ay, az, gx, gy, gz);
                pos=buf;
            }else{
                pos++;
            }
        }
    }


    close(fd);
}
————————————————
版权声明：本文为CSDN博主「Chen-Lee」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/qq_16583687/article/details/55259778