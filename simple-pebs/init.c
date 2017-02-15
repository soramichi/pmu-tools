#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "simple-pebs.h"

int main(int argc, char* argv[]){
  int fd, ret;
  struct simple_pebs_parameter param;

  if(argc < 5){
    fprintf(stderr, "usage: %s pebs_event(hex) reset_value buffer_size output_mode\n", argv[0]);
    return -1;
  }
  else{
    param.pebs_event = strtol(argv[1], NULL, 16);
    param.reset_value = (unsigned)atoi(argv[2]);
    param.buffer_size = (unsigned)atoi(argv[3]);
    param.output_mode = (unsigned)atoi(argv[4]);
  };
  
  fd = open("/dev/simple-pebs", O_RDONLY);
  
  if (fd < 0){
    perror("/dev/simple-pebs open");
    return -1;
  }

  ret = ioctl(fd, SIMPLE_PEBS_INIT, &param);
  if(ret == -1){
    perror("ioctl(fd, SIMPLE_PEBS_INIT, &param)");
  }

  return 0;
}
