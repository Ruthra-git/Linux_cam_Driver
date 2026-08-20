#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#define FRAME_SIZE (640*480*2)

int main(void)
{
    int fd;

    uint8_t *frame;

    FILE *fp;

    fd = open("/dev/video0", O_RDWR);

    if (fd < 0) {
        perror("open");
        return 1;
    }

    frame = mmap(NULL,
                 FRAME_SIZE,
                 PROT_READ,
                 MAP_SHARED,
                 fd,
                 0);

    if (frame == MAP_FAILED) {

        perror("mmap");

        close(fd);

        return 1;
    }

    printf("First 32 bytes\n");

    for (int i = 0; i < 32; i++)
        printf("%02X ", frame[i]);

    printf("\n");

    fp = fopen("frame.raw", "wb");

    fwrite(frame,
           1,
           FRAME_SIZE,
           fp);

    fclose(fp);

    munmap(frame,
           FRAME_SIZE);

    close(fd);

    return 0;
}