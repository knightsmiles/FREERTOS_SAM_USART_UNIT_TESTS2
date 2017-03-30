#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

int set_options(int fd);
int serial_read(int fd, char *str);
int wait_reply(int fd, unsigned char *pak);

#endif // SERIAL_H_INCLUDED
