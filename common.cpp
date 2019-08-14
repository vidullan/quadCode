#include "common.h"

uint64_t getTimeUsec(){
    struct timespec t;
    clock_gettime(CLOCK_REALTIME,&t);
    return (t.tv_sec)*1e6+(t.tv_nsec)/1e3;
}