/*
    font: Generates binary font data from a header file.
    Copyright (C) 2016 Richard Taylor <richard@artaylor.co.uk>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/  
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
