#include "lcdfont_medium.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

int main(void)
{
    int fd = open("dump.bin", O_CREAT|O_RDWR);
    if (fd > 0)
    {
        write(fd, font_medium, sizeof(font_medium));
        close(fd);
    }
}
