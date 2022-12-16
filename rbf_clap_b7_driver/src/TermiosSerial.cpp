//
// Created by ozkan on 16.03.2022.
//

#include "TermiosSerial.h"

TermiosSerial::TermiosSerial(const char *port_name, int baudrate) : serial_port{open(port_name, O_RDWR)} {
  if (config_tty(baudrate)){
    return;
  }

}

int TermiosSerial::config_tty(int baudrate) {
  // Read in existing settings, and handle any error
  if (tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    return 1;
  }

  tty.c_cflag &= ~PARENB;       // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;       // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE;        // Clear all bits that set the data size
  tty.c_cflag |= CS8;           // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS;      // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL;// Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;                                                       // Disable echo
  tty.c_lflag &= ~ECHOE;                                                      // Disable erasure
  tty.c_lflag &= ~ECHONL;                                                     // Disable new-line echo
  tty.c_lflag &= ~ISIG;                                                       // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                     // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);// Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST;// Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR;// Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;// Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate
  speed_t termios_baudrate{0};

  switch (baudrate) {
    case 4800 : termios_baudrate = B4800; break;
    case 9600 : termios_baudrate = B9600; break;
    case 19200 : termios_baudrate = B19200; break;
    case 38400 : termios_baudrate = B38400; break;
    case 115200 : termios_baudrate = B115200; break;
    case 460800 : termios_baudrate = B460800; break;
    case 230400 : termios_baudrate = B230400; break;
  }
  if (!termios_baudrate){
    std::cout << "Non-standard baud rate!\n";
    return 1;
  }
//  cfsetispeed(&tty, B115200);
//  cfsetospeed(&tty, B115200);

  cfsetispeed(&tty, B230400);
  cfsetospeed(&tty, B230400);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    return 1;
  }
  return 0;
}

int TermiosSerial::Read(void *buffer) {
    return read(serial_port, buffer, static_cast<ssize_t>(512));
}

void TermiosSerial::Write(const void * buffer, size_t lenght) const{
  write(serial_port, buffer, lenght);
}

void TermiosSerial::Close() {
    close(serial_port);
}

TermiosSerial::~TermiosSerial() {
  close(serial_port);
}
