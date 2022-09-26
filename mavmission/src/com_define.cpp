
#include <com_define.h>

uint64_t get_time_usec()	// Time 获取系统时间
{	
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec * 1000000 + _time_stamp.tv_usec;
}

float satfunc(float data, float Max)
{
    if( abs(data)>Max )
        return (data>0)?Max:-Max;
    else
        return data;
}
