#pragma once

#include <chrono>


using namespace std;
using namespace chrono;


/** Implements a watchdog for periodically updating tasks.

The monitored task must be a class which meets the following requirments:

* Implements a method named update_counter(), which takes no arguments and returns 
    a size_t. The change in this value between two calls should represent the number of 
    updates completed by the task in that period.


@tparam T class meeting the requirements for a monitored task.
*/
template<class T>
class TaskMonitor
{
public:
    /** Create a new task monitor.
    
    @param task Task instance to monitor
    @param exp_rate Expected correct update rate for the task
    */
    TaskMonitor(T& task, float exp_rate): _task(task), _rate(exp_rate)
    {
        _count_prev = task.update_counter();
        _t_prev = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    }

    /** Collect the the change in the tasks update count.
    */
    void update()
    {        
        auto count = task.update_counter();
        auto t = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        _rate_est = (count - _count_prev) * 1e3 / static_cast<float>(t - _t_prev);
    }

    /** Provides an estimate of the monitored tasks rate, in updates per second.

    @return Rate, in Hz
    */
    float rate(){ return _rate_est; }
    
    /** Provides an estimate of the standard deviation
        of the monitored tasks rate 

    @return Estimated standard deviation
    */
    float dev() = delete; // Not yet implemented

    /** Checks if the measured cycle time is near the expected rate.
    
    Uses the latest estimate of the average update rate as the real value,
    and checks if the difference between that and the expected value
    given to the constructor is less than the specified tolerance.

    @param tol Tolerance, in Hz, on the rate check. Default 1.

    @return True if the rate estimate is within the tolerance of the expected value.
    */
    bool ok(float tol = 1.0)
    {
        auto diff = rate() - _rate;
        return (diff < tol) && (diff > -tol);
    }

protected:
    T& _task; /// Monitored task
    const float _rate; /// Expected update rate
    size_t _count_prev; /// Previous update counter value
    unsigned long long _t_prev; /// Time of last update, in ms
    float _rate_est, _std_est; /// Rate and deviation estimates
}