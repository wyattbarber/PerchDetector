#pragma once

#include <iostream>
#include "Logging.hpp"
#include <initializer_list>
#include <iostream>
#include <vector>
#include <map>
#include <chrono>


/** Base interface for a task

*/
class Task : public std::enable_shared_from_this<Task>
{
public:
    typedef void(*command_executor_t)(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args); /// Method type for handling task-specific CLI actions

    Task(const char* name, std::initializer_list<std::shared_ptr<Task>> dependencies) : 
        name(name),
        depends_on{dependencies}
    {
        alive = false;
        rate_est = 0.0;
        tick_called_once = false;
        tick_called_twice = false;
    }

    /** Must be called after construction before any other methods.
    
    Implements functionality that cannot be done
    in constructor.
    */
    void init();
    
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

    virtual bool start();
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

    /** Get list of commands this task implements.
    
    Gets a vector containing all command names supported
    by this task. Each name is a command that can be called
    by the call() method.

    @return All command names implemented by this task.
    */
    std::vector<std::string> list_commands();
    

    /** Check if this task implements a command.
    
    @param cmd Name of the command to check for

    @return True if the named command is implemented.
    */
    bool implements(const std::string& cmd);

    /** Calls a command implemented by this task.
    
    @param cmd Name of the command to call
    @param in Stream to provide user input
    @param out Stream to provide output to user
    @param args Arguments given to the command as strings
    */
    void call(const std::string& cmd, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

    /** Start task and its dependencies. 

    @return True if this task and all dependencies started without error.
    */
    bool autostart();

    /** Stop tasks and all that depend on it. 
    */
    void autostop();

    /** Get the update rate for this task.
    
    If the task type is tracking the update rate
    with tick(), then this method will return that 
    rate in Hz (calls to tick() per second) and true
    in the valid flag.

    If the rate is not being tracked, the valid flag 
    will be false and the rate 0;

    @return rate, valid pair
    */
    std::pair<float, bool> rate(){ return {rate_est, tick_called_twice}; }

    const char* name;

protected:
    /** Adds a CLI action for this task type.
    
    Registers an action that the program user may call from the CLI
    during program operation.

    A method is given to execute the command, that takes 3 arguments:
    
    1. A pointer to the task on which the command is being executed
    
    2. A stream that provides input from the user

    3. A stream to give output to the user

    4. The arguments given by the user, as a vector of strings.

    This function signature is typedef'd as Task::command_executor_t.

    @param cmd Name of the command that the user will run
    @param executor Method of the task that will run the command
    */
    void declare_cli_command(const char* cmd, command_executor_t executor);

    /** Marks this task as a dependency of the given task.
    
    @param task Task that depends on this one.
    */
    void declare_dependency_of(std::shared_ptr<Task> task);

    /** To be used by the task implementation for rate tracking.
    
    A call to this method will mark a completion of one cycle
    and an update of the rate measurement.
    */
    void tick();

    bool alive;
    std::map<std::string, command_executor_t> commands;
    std::vector<std::shared_ptr<Task>> depends_on, depended_by;
    bool tick_called_once, tick_called_twice;
    std::chrono::steady_clock::time_point last_tick_call;
    float rate_est;
};



