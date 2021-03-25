#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

#include <unistd.h>

int get_registered_uav_count();
void reg_serial_with_uav(int id, const char *port_name, int baudrate);
void serial_puts(int id, char *s, size_t size);
int serial_getc(int id, char *c);

#endif
