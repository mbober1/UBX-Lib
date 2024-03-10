#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include "ubx_lib.hpp"

int setup_serial_port(const char *port, const speed_t baudrate)
{
  int serial_port = open(port, O_RDWR);
  struct termios tty;

  if (serial_port < 0) 
  {
    return -1;
  }

  if(tcgetattr(serial_port, &tty) != 0) 
  {
    return -1;
  }

  tty.c_cflag &= ~PARENB;         // No parity
  tty.c_cflag &= ~CSTOPB;         // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE;          // Clear all bits that set the data size 
  tty.c_cflag |= CS8;             // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;        // Disable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;          // Canonical mode
  tty.c_lflag &= ~ECHO;           // Disable echo
  tty.c_lflag &= ~ECHOE;          // Disable erasure
  tty.c_lflag &= ~ECHONL;         // Disable new-line echo
  tty.c_lflag &= ~ISIG;           // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 64;

  cfsetispeed(&tty, baudrate);
  cfsetospeed(&tty, baudrate);

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) 
  {
    return 1;
  }

  return serial_port;
}

int main()
{
  uint8_t read_buf [256];

  int serial_port = setup_serial_port("/dev/ttyUSB0", B9600);

  if (serial_port < 0)
  {
    printf("Error %i from open: %s\n", errno, strerror(errno));
    return 1;
  }
  else
  {
    printf("Serial port opened successfully\n");
  }

  struct posllh posllh_data;

  while (true)
  {
    int n = read(serial_port, &read_buf, sizeof(read_buf));

    if (n > 0)
    {
      // printf("Read %d bytes -> ", n);

      // for (size_t i = 0; i < n; i++)
      // {
      //   printf("%02X ", read_buf[i]);
      // }
      // printf("\n");

      bool status = gps_parse(read_buf, n, &posllh_data);
      if (status)
      {
        printf("iTOW: %u, lon: %d, lat: %d, height: %d, hMSL: %d, hAcc: %u, vAcc: %u\n", 
          posllh_data.iTOW, posllh_data.lon, posllh_data.lat, posllh_data.height, posllh_data.hMSL, posllh_data.hAcc, posllh_data.vAcc);
      }
      else
      {
        // printf("GPS data was not parsed\n");
      }
    }

    memset(&read_buf, '\0', sizeof(read_buf));
  }

  close(serial_port);
  return 0;
}