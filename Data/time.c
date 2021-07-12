#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

int main(void) {
    struct timeval tmnow;
    struct tm *tm;
    char buf[30], usec_buf[6];
    gettimeofday(&tmnow, NULL);
    tm = localtime(&tmnow.tv_sec);
    strftime(buf,30,"%Y:%m:%dT%H:%M:%S", tm);
    strcat(buf,".");
    sprintf(usec_buf,"%d",(int)tmnow.tv_usec);
    strcat(buf,usec_buf);
    printf("%s",buf);
    return 0;
}