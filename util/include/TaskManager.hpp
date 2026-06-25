#pragma once

#include "Task.hpp"
#include <map>
#include <thread>
#include <utility>
#include <chrono>

/** Container for tasks executed in a program. 

*/
class TaskManager
{
public: 
    TaskManager(): tick_period(0.1), tick_counter(0) {}

    /** Get a pointer of specific type by name.
    
    @tparam Task type to cast the given task to.

    @param name Name of the task to get.

    @return Pointer to the named task.
    */
    template<typename T>
    std::shared_ptr<T> get(const std::string& name){ return std::dynamic_pointer_cast<T>(tasks[name]); }

    std::shared_ptr<Task> get(const std::string& name){ return get<Task>(name); }

    
    /** Add a task, autofilling its name.
    
    @tparam Type of the task to add.

    @param task Task to add
    */
    template<typename T>
    void add(std::shared_ptr<T> task)
    {
        tasks[task->name] = task; 
        executors[task->name] = _task_runner<T>;
    }

    /** Create a new task to add.
    
    @tparam Type of the task to add.

    @param Args Constructor arguments for the task to add.
    */
    template<typename T, typename... Ts>
    void create(Ts&&... Args){ add(std::make_shared<T>(Args...)); }

    /** Starts all tasks.
    
    Should be called once all tasks are created and added.
    */
    void launch();

    /** Stops all tasks.
    
    Should be on program termination for a clean shutdown.
    */
    void kill();

    /** Set the tick frequency.
    
    Effectively sets the maximum cycle rate of all tasks.

    @param f Tick frequency to set.
    */
    void set_tick(float f);

    /** Helpers for compatibility with existing map iteration code in main.
    */
    const auto begin(){ return tasks.begin(); }
    const auto end(){ return tasks.end(); }
    const auto find(const std::string& name){ return tasks.find(name); }

    std::vector<std::string> names();

protected:
    std::map<std::string, std::shared_ptr<Task>> tasks;
    std::map<std::string, void(*)(std::shared_ptr<Task>, TaskManager*)> executors;
    std::map<std::string, std::unique_ptr<std::thread>> threads;
    bool running;
    float tick_period;
    size_t tick_counter;
    std::chrono::time_point<std::chrono::steady_clock> last_tick;

    /** Get the number of ticks since program start. 
    */
    size_t get_ticks();

    template<typename T>
    static void _task_runner(std::shared_ptr<Task> task, TaskManager* manager)
    {
        auto t = std::static_pointer_cast<T>(task);
        size_t last_tick_count = 0;
        while(manager->running)
        {
            auto tick = manager->get_ticks();
            if(t->is_alive() && (tick != last_tick_count))
            {
                last_tick_count = tick;
                t->step();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};


typedef TaskManager task_executor; // Typedef for compatibility with old stuff I wrote