/*
Modified : 2015/01/05

You have to add 'rtlib.a' to additional dependencies.
*/
#ifdef __linux__
#include <time.h>
#include <assert.h>

/*
 * CLOCK_REALTIME
 * CLOCK_PROCESS_CPUTIME_ID
 * CLOCK_THREAD_CPUTIME_ID
 * CLOCK_MONOTONIC
 *
 * http://www.guyrutenberg.com/2007/09/22/profiling-code-using-clock_gettime/
 */

timespec t_start, t_end;
/*
 * Start function samples
 */
void clock_start(void){
	clock_gettime(CLOCK_REALTIME,&t_start);
}

/*
 * This function samples current time and calculate elapsed time
 * after start function called.
 * And stop function returns the elapsed time in usec resolution.
 */
long int clock_stop(void){
	clock_gettime(CLOCK_REALTIME,&t_end);
	long int ts,te;
	ts = t_start.tv_sec*1000000u + t_start.tv_nsec/1000u;
	te = t_end.tv_sec*1000000u + t_end.tv_nsec/1000u;
	return (te-ts);
}

/*
 * How accurate the clock is.
 * Returned value represent the update frequency of clock.
 */
unsigned int get_update_frequency(void)
{
    timespec ts;

    if (clock_getres(CLOCK_REALTIME, &ts) < 0)
    {
        // log error
        std::terminate();
    }

    assert(!ts.tv_sec);

    // this is the precision of the clock, we want the number of updates per second, which is
    // 1 / ts.tv_nsec * 1,000,000,000
    static const unsigned int billion = 1000000000;

    return billion / static_cast<unsigned int>(ts.tv_nsec);
}

#elif __WINDOWS__
#include <time.h>
#include <assert.h>
/*
 * CLOCK_REALTIME_PRECISE
 * CLOCK_PROCESS_CPUTIME_ID
 * CLOCK_THREAD_CPUTIME_ID
 * CLOCK_MONOTONIC_PRECISE
 *
 * http://www.guyrutenberg.com/2007/09/22/profiling-code-using-clock_gettime/
 */
timespec t_start, t_end;

void clock_start(void){
	clock_gettime(CLOCK_REALTIME_PRECISE,&t_start);
}

/*
 * This function samples current time and calculate elapsed time
 * after start function called.
 * And stop function returns the elapsed time in usec resolution.
 */
long int  clock_stop(void){
	clock_gettime(CLOCK_REALTIME_PRECISE,&t_end);
	long int ts,te;
	ts = t_start.tv_sec*1000000u + t_start.tv_nsec/1000u;
	te = t_end.tv_sec*1000000u + t_end.tv_nsec/1000u;
	return (te-ts);
}

/*
 * How accurate the clock is.
 * Returned value represent the update frequency of clock.
 */
unsigned int get_update_frequency(void)
{
    timespec ts;

    if (clock_getres(CLOCK_REALTIME, &ts) < 0)
    {
        // log error
        std::terminate();
    }

    assert(!ts.tv_sec);

    // this is the precision of the clock, we want the number of updates per second, which is
    // 1 / ts.tv_nsec * 1,000,000,000
    static const unsigned int billion = 1000000000;

    return billion / static_cast<unsigned int>(ts.tv_nsec);
}



#endif

#if 0  	//example

start();
//do something.
for(int i=0;i<10000000;i++)
{
	int a = 1;
}

long int elapsed = stop();
printf("%ld usec\n",elapsed);

#endif
