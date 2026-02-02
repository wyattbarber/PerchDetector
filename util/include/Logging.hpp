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




