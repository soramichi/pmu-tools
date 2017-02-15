#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "simple-pebs.h"

int main(){
  int fd, ret;
  struct simple_pebs_parameter param = {
    .pebs_event = 0x20D1,
    .reset_value = 0x64,
    .buffer_size = 64 * 1024
    .output_mode = 0,
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
