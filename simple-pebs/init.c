#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "simple-pebs.h"

const unsigned int UOPS_RETIRED_ALL = 0x1C2;
const unsigned int MEM_LOAD_UOPS_RETIRED_L3_MISS = 0x20D1;
const unsigned int page_size = 4096;

int main(){
  int fd, ret;
  struct simple_pebs_parameter param = {
    .pebs_event = MEM_LOAD_UOPS_RETIRED_L3_MISS,
    .reset_value = 100,
    .buffer_size = (page_size << 8),
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
