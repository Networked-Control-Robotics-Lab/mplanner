#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

void serial_init(char *port_name, int baudrate);
void serial_puts(char *s, size_t size);

#endif
