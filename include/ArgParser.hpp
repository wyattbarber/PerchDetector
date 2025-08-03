#pragma once

#include <cstring>
#include <iostream>

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
        bool log_def = false, stats_def = false, image_def = false, depth_def = false, data_def = false;
        int i = 1;
        while(i < argc)
        {
            std::cout << "Argument " << argv[i] << " is " << argv[i+1] << std::endl;
            if(strcmp(argv[i], "--log-file") == 0)
            {
                _log_file = argv[i+1];
                log_def = true;
                i += 2;
            }
            else if(strcmp(argv[i], "--stats-file") == 0)
            {
                _stats_file = argv[i+1];
                stats_def = true;
                i += 2;
            }
            else if(strcmp(argv[i], "--image-file") == 0)
            {
                _image_map = argv[i+1];
                image_def = true;
                i += 2;
            }
            else if(strcmp(argv[i], "--depth-file") == 0)
            {
                _depth_map = argv[i+1];
                depth_def = true;
                i += 2;
            }
            else if(strcmp(argv[i], "--data-file") == 0)
            {
                _data_map = argv[i+1];
                data_def = true;
                i += 2;
            }
            else
            {
                i += 1;
            }
        }

        if(image_def || depth_def || data_def)
        {
            _headless = false;
            _valid = image_def && depth_def;
            if(!_valid) std::cerr << "Image and depth mapping files must be specified in GUI mode." << std::endl;
        }
        else
        {
            _headless = true;
            _valid = true;
        }
    }

    /** Provided arguments give a valid application configuration. */
    bool valid(){ return _valid; }

    /** Provided arguments are for headless operation. */
    bool headless(){ return _headless; }

    /** Filename for memory mapped image visualization. */
    const char* image_map_file(){ return _image_map; }

    /** Filename for memory mapped depth visualization. */
    const char* depth_map_file(){ return _depth_map; }

    /** Filename for memory mapped numeric data display. */
    const char* data_map_file(){ return _data_map; }

    /** Filename for text logs. */
    const char* log_file(){ return _log_file; }

    /** Filename for numeric logs. */
    const char* stats_file(){ return _stats_file; }

    protected:
    bool _valid, _headless;
    char* _image_map, * _depth_map, * _data_map;
    char* _log_file, * _stats_file;

};