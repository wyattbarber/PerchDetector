#include <iostream>
#include "Logging.hpp"
#include <map>
#include <thread>
#include <initializer_list>


/** Base interface for a task

*/
class Task {
public:

    Task(const char* name) : name(name)
    {
        alive = false;
    }

    /** Logs a status message.
    
    Will generate one line of formatted log output
    from the given value, with the form
    [INFO][<task name>] <value>

    @param values Stream capable values to log   
    */
    template<typename... Ts>
    void info(const Ts&... values)
    {
        ((Logger::instance() << "[INFO][" << name << "] ") << ... << values) << std::endl;
    }
    
    /** Logs a warning message.
    
    Will generate one line of formatted log output
    from the given value, with the form
    [WARNING][<task name>] <value>
    
    @param values Stream capable values to log   
    */
    template<typename... Ts>
    void warning(const Ts&... values)
    {
        ((Logger::instance() << "[WARNING][" << name << "] ") << ... << values) << std::endl;
    }
    
    /** Logs an error message.
    
    Will generate one line of formatted log output
    from the given value, with the form
    [ERROR][<task name>] <value>
    
    @param values Stream capable values to log   
    */
    template<typename... Ts>
    void error(const Ts&... values)
    {
        ((Logger::instance() << "[ERROR][" << name << "] ") << ... << values) << std::endl;
    }

    bool start();
    void stop();
    
    /** Do not call directly, use start().

    Must be implemented to begin task operation.
    Will be called from a different thread than step().

    @return True if the task was started.
    */
    virtual bool start_impl() = 0;
    
    /** Do not call directly, use stop().
    
    Must be implemented to end task operation.
    Will be called from a different thread than step(),
    and start() may be called again after.
    */
    virtual void stop_impl() = 0;

    /** Execute the task cyclic process.
    
    Called repeatedly by task executor thread, 
    even before start is called and after stop is called.
    */
    virtual void step() = 0;

    /** Check if task has been started.
    
    @return true if the task was started without error.
    */
    bool is_alive(){ return alive; }

    const char* name;

protected:
    bool alive;
};

/** Container for executing tasks.

Maps task names to pairs of task, thread, 
where the thread is the thread in which the task is executing.
*/
typedef std::pair<Task*, std::thread*> task_executor;


/** Starts tasks in new threads.

Creates a new thread to execute each task, and assembles 
a map of task names to tasks and threads.

@param tasks Pointers to tasks to execute.
*/
task_executor run_tasks(std::initializer_list<Task*> tasks);


/** Kills all tasks.

Ends all tasks and destroys their threads.

@param exe map of tasks and threads
*/
void kill_tasks(task_executor& exe);