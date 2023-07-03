#include "serial.h"

Serial::Serial()
{
}
Serial::~Serial()
{
}

int Serial::BaudRate(int baudrate)
{
    switch (baudrate)
    {
    case 2400:
        return (B2400);
    case 4800:
        return (B4800);
    case 9600:
        return (B9600);
    case 19200:
        return (B19200);
    case 38400:
        return (B38400);
    case 57600:
        return (B57600);
    case 115200:
        return (B115200);
    case 460800:
        return (B460800);
    default:
        return (B9600);
    }
}

int Serial::SetPara(int serialfd, int speed, int databits, int stopbits, int parity)
{
    struct termios termios_new;
    bzero(&termios_new, sizeof(termios_new)); //�ȼ���memset(&termios_new,sizeof(termios_new));
    cfmakeraw(&termios_new);                  //���ǽ��ն�����Ϊԭʼģʽ
    termios_new.c_cflag = BaudRate(speed);
    termios_new.c_cflag |= CLOCAL | CREAD;
    //  termios_new.c_iflag = IGNPAR | IGNBRK;

    termios_new.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 0:
        termios_new.c_cflag |= CS5;
        break;
    case 1:
        termios_new.c_cflag |= CS6;
        break;
    case 2:
        termios_new.c_cflag |= CS7;
        break;
    case 3:
        termios_new.c_cflag |= CS8;
        break;
    default:
        termios_new.c_cflag |= CS8;
        break;
    }

    switch (parity)
    {
    case 0:                             //as no parity
        termios_new.c_cflag &= ~PARENB; //Clear parity enable
                                        //  termios_new.c_iflag &= ~INPCK; /* Enable parity checking */  //add by fu
        break;
    case 1:
        termios_new.c_cflag |= PARENB; // Enable parity
        termios_new.c_cflag &= ~PARODD;
        break;
    case 2:
        termios_new.c_cflag |= PARENB;
        termios_new.c_cflag |= ~PARODD;
        break;
    default:
        termios_new.c_cflag &= ~PARENB; // Clear parity enable
        break;
    }
    switch (stopbits) // set Stop Bit
    {
    case 1:
        termios_new.c_cflag &= ~CSTOPB;
        break;
    case 2:
        termios_new.c_cflag |= CSTOPB;
        break;
    default:
        termios_new.c_cflag &= ~CSTOPB;
        break;
    }

    //set fd to nonblock to reduce system soft interrupt cost
    int flags;
    flags = fcntl(serialfd, F_GETFL, 0);
    flags &= ~O_NONBLOCK;
    fcntl(serialfd, F_SETFL, flags);

    tcflush(serialfd, TCIFLUSH);                       //  清除输入缓存
    tcflush(serialfd, TCOFLUSH);                       // 清除输出缓存
    termios_new.c_cc[VTIME] = 0;                       // MIN与 TIME组合有以下四种：1.MIN = 0 , TIME =0  有READ立即回传 否则传回 0 ,不读取任何字元
    termios_new.c_cc[VMIN] = 40;                        //    2、 MIN = 0 , TIME >0  READ 传回读到的字元,或在十分之一秒后传回TIME 若来不及读到任何字元,则传回0
    tcflush (serialfd, TCIFLUSH);                      //    3、 MIN > 0 , TIME =0  READ 会等待,直到MIN字元可读
    return tcsetattr(serialfd,TCSANOW,&termios_new);   //    4、 MIN > 0 , TIME > 0 每一格字元之间计时器即会被启动 READ 会在读到MIN字元,传回值或
}

int Serial::WriteData(int fd, const char *data, int datalength)
{
    if (fd < 0)
    {
        return -1;
    }
    int len = 0, total_len = 0, send = 0; //modify8.
    //sleep(2);
    //printf("WriteData fd = %d , datalength = %d\n", fd, datalength);
    for (total_len = 0; total_len < datalength;)
    {
        len = 0;
        send = datalength - total_len > 1024 ? 1024 : datalength - total_len;
        len = write(fd, data + total_len, send);
        if (len > 0)
        {
            printf("WriteData fd = %d ,len =%d,total_len = %d datalength = %d send = %d\n", fd, len, total_len + len, datalength,send);
            usleep(100000);
            total_len += len;
        }
        else if (len == -1)
        {
            printf("len = %d\n",len);
            usleep(100000);
            continue;
        }
        else if (len <= 0)
        {
            perror("read");
            len = -1;
            break;
        }
    }
    return len;

}

int Serial::ReadData(int fd, unsigned char *data, int datalength)
{
    if (fd < 0)
    {
        return -1;
    }
    int len = 0;
    memset(data, 0, datalength);

    int max_fd = 0;
    fd_set readset = {0};
    struct timeval tv = {0};

    FD_ZERO(&readset);
    FD_SET((unsigned int)fd, &readset);
    max_fd = fd + 1;
    tv.tv_sec = 0;
    tv.tv_usec = 2000;
    if (select(max_fd, &readset, NULL, NULL, &tv) < 0)
    {
        printf("ReadData: select error\n");
    }
    int nRet = FD_ISSET(fd, &readset);
    if (nRet)
    {
        len = read(fd, data, datalength);
    }
    return len;
}

void Serial::ClosePort(int fd)
{
    if (fd > 0)
    {
        tcsetattr(fd, TCSADRAIN, &termios_old);
        ::close(fd);
    }
}

int Serial::OpenPort(const char *device_port)
{
    int fd;

    fd = open(device_port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        printf("Open %s failed, handle = %d\n",device_port, fd);
        return -1;
    }
    else
    {
        printf("Open %s success, handle = %d\n", device_port, fd);
    }
    tcgetattr(fd, &termios_old);
    return fd;
}
