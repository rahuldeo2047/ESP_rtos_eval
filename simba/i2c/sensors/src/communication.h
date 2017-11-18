
#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#define COMM_THRD_NAME ("comm_thrd")
inline char * comm_thrd_get_thrd_name()
{
  return (COMM_THRD_NAME);
}

inline int comm_thrd_get_channel_id(int *chnid_p)
{
   *chnid_p = 0x02;
   return (0);
}

void *comm_thrd(void *arg_p);


#endif
