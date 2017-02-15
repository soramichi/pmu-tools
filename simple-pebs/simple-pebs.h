#define SIMPLE_PEBS_BASE 	0x7000
#define SIMPLE_PEBS_SET_CPU    	(SIMPLE_PEBS_BASE + 1)
#define SIMPLE_PEBS_GET_SIZE   	(SIMPLE_PEBS_BASE + 2)
#define SIMPLE_PEBS_GET_OFFSET 	(SIMPLE_PEBS_BASE + 3)
#define SIMPLE_PEBS_START	(SIMPLE_PEBS_BASE + 4)
#define SIMPLE_PEBS_STOP	(SIMPLE_PEBS_BASE + 5)
#define SIMPLE_PEBS_RESET	(SIMPLE_PEBS_BASE + 6)
#define SIMPLE_PEBS_INIT	(SIMPLE_PEBS_BASE + 7)

struct simple_pebs_parameter {
  unsigned int pebs_event;
  unsigned int reset_value;
  unsigned int buffer_size;
  int output_mode;
};
