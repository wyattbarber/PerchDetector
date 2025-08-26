// Boilerplate generated with ChatGPT. Support for file output and DataLogger class added by me.

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <mutex>
#include <memory>
#include <string>
#include <chrono>

class Logger {
public:
    // Get the singleton instance
    static Logger& instance() {
        static Logger _instance;
        return _instance;
    }

    // Deleted methods to prevent copying
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // Templated stream operator
    template <typename T>
    Logger& operator<<(const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(has_file)
            file << value;
        else
            std::cout << value;
        return *this;
    }

    // Support for manipulators like std::endl
    Logger& operator<<(std::ostream& (*manip)(std::ostream&)) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(has_file)
            file << manip;
        else
            std::cout << manip;
        return *this;
    }

    void set_file(const char* path)
    {
        file.open(path);
        has_file = true;
    }

private:
    Logger() : has_file(false)
    {}

    // Ensure file is closed at program exit
    ~Logger()
    {
        if(has_file)
        {
            file.flush();
            file.close();
        }
    }

    std::mutex mutex_;
    bool has_file;
    std::ofstream file;
};


class DataLogger {
public:
    // Get the singleton instance
    static DataLogger& instance() {
        static DataLogger _instance;
        return _instance;
    }

    // Deleted methods to prevent copying
    DataLogger(const DataLogger&) = delete;
    DataLogger& operator=(const DataLogger&) = delete;

    /** Log a value change.

    @param value Updated value to log
    @param args Channel names. Multiple names can be given to group signals, they will be joined in the logged line.
    */
    template<typename T, typename... Args>
    void log_value(T& value, Args... args)
    {
        auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        auto channel = _join_names(args...);
        std::lock_guard<std::mutex> lock(mutex_);
        file << "[" << ts << "] " << channel << " @ " << value << std::endl;
    }


    void set_file(const char* path)
    {
        file.open(path);
        valid = true;
    }

private:
    template<typename... Args>
    std::string _join_names(const std::string& name, Args... args)
    {
        return _join_names(args...) + ":" + name;
    }
    std::string _join_names(const std::string& name)
    {
        return name;
    }

    DataLogger() : valid(false)
    {}

    // Ensure file is closed at program exit
    ~DataLogger()
    {
        if(valid)
        {
            file.flush();
            file.close();
        }
    }

    std::mutex mutex_;
    bool valid;
    std::ofstream file;
};


