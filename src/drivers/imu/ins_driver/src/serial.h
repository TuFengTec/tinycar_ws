#ifndef _SERIAL_H_
#define _SERIAL_H_
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
// enum
// {
//     COM0 = 0,
//     COM1,
//     COM2,
//     COM3,
//     ttyUSB0,
//     ttyUSB1,
//     ttyUSB2,
//     YIS10,
//     M8N,
//     GS1810
// };
class Serial
{
  public:
    Serial();
    ~Serial();

    int OpenPort(const char *device_port);
    int SetPara(int serialfd, int speed, int databits = 8, int stopbits = 1, int parity = 0);
    int WriteData(int fd, const char *data, int datalength);
    int ReadData(int fd, unsigned char *data, int datalength);
    void ClosePort(int fd);
    int BaudRate(int baudrate);
  private:
    struct termios termios_old;
};
#endif
