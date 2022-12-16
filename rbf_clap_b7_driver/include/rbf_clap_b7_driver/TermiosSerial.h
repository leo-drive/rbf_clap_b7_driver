//
// Created by ozkan on 16.03.2022.
//

#ifndef SERIAL_PORT__TERMIOSSERIAL_H_
#define SERIAL_PORT__TERMIOSSERIAL_H_


#include <cstdio>
#include <cstring>
#include <iostream>
// Linux headers
#include <cerrno>   // Error integer and strerror() function
#include <fcntl.h>  // Contains file controls like O_RDWR
#include <termios.h>// Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

class TermiosSerial {

 public:
  TermiosSerial(const char *port_name, int baudrate);
  ~TermiosSerial();

  int Read(void *buffer);
  void Write(const void * buffer, size_t lenght) const;
  void Close();

 private:
  int serial_port;
  struct termios tty{};

  int config_tty(int baudrate);
};

#endif//SERIAL_PORT__TERMIOSSERIAL_H_
