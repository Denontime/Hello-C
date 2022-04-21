#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <string.h>


int run = 1;

void signal_handler(int sig)
{
    run = 0;
}

int main(int argc, char *argv[])
{
    int fd;
    int index;
    char *data;
    const char *filepath = "sharedfile";

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    if ((fd = open(filepath, O_CREAT|O_RDWR, (mode_t)00700)) == -1) {
        perror("open");
        exit(EXIT_FAILURE);
    }

    data = mmap(NULL, 12288, PROT_WRITE|PROT_READ, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED) {
        perror("mmap");
        exit(EXIT_FAILURE);
    }

    // We must see 'Goose' at the beginning of memory-mapped file.
    // while (run)
    // {
    //     for (index = 0; index < 200; index++)
    //     {
    //         fprintf(stdout, "%c", data[index]);
    //     }
    //     printf("\n");
    //     sleep(1);
    // }
    while (run)
    {
        time_t now;
        time(&now);
        char *tmp = ctime(&now);
        printf("%s    ", tmp);
        for (index= 0; index < strlen(tmp); index++) 
        {
            data[index+20] = tmp[index];
        }
        printf("%s\n", data);
        sleep(3);
    }

    if (msync(data, 12288, MS_SYNC) == -1) {
        perror("Error sync to disk");
    } 

    if (munmap(data, 12288) == -1) {
        close(fd);
        perror("Error un-mmapping");
        exit(EXIT_FAILURE);
    }

    close(fd);

    return 0;
}