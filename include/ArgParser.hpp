#pragma once

#include <cstring>

/** Helper class to parse the expected arguments from the command line
 * 
 * Parses arguments expecting a set of arguments for one of two modes:
 * 
 * * Headless mode, where data is not visualized, and stats are optionally logged to a file
 * 
 * * GUI mode, where visual data is memory mapped for display by another program
 * 
 * 
 * 
 * Required for GUI mode:
 * 
 * --image-file: File name for memory mapped image data
 * 
 * --depth-file: File name for memory mapped depth data
 * 
 * --data-file: File name for memory mapped process stats
 * 
 * 
 * Optional for either mode:
 * 
 * --stats-file: File name to record numerical data
 * 
 * --log-file: File name to save text log data
 */
class ArgParser
{
    public:
    ArgParser(int argc, char** argv)
    {

    }

    /** Provided arguments give a valid application configuration. */
    bool valid();

    /** Provided arguments are for headless operation. */
    bool headless();

    /** Filename for memory mapped image visualization. */
    const char* image_map_file();

    /** Filename for memory mapped depth visualization. */
    const char* depth_map_file();

    /** Filename for memory mapped numeric data display. */
    const char* data_map_file();

    /** Filename for text logs. */
    const char* log_file();

    /** Filename for numeric logs. */
    const char* stats_file();

    protected:

    char* _image_map, _depth_map, _data_map;
    char* _log_file, _stats_file;

};