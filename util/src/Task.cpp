#include "Task.hpp"

bool Task::start()
{
    if(!alive)
    {
        auto res = this->start_impl();
        if(res)
        {
            alive = true;
        }
        return res;
    }
    return true;
}


void Task::stop()
{
    alive = false;
    this->stop_impl();
}