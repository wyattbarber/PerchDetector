#pragma once

#include <ImageSender.hpp>
#include <CameraWrapper.hpp>
#include <ArgParser.hpp>

/** Helpers for communication to a GUI

*/
namespace visualization
{
    extern std::unique_ptr<ImageSender> image, depth;

    int setup(ArgParser& args, std::shared_ptr<CameraWrapper> camera_left, std::shared_ptr<CameraWrapper> camera_right);

    void teardown();

    /** Handles getting parameters from NiceGUI sliders.
    
    Opens and memory maps a file, and reads parameters from it.
    Expects the following format:

    * 1 byte lock, non-zero if a process is using the data

    * 1 byte minDisparity

    * 2 bytes numDisparities

    * 2 bytes blockSize

    * 2 bytes P1

    * 2 bytes P2

    * 2 bytes disp12MaxDiff

    * 2 bytes preFilterCap

    * 2 bytes uniquenessRatio

    * 2 bytes speckleWindowSize

    * 2 bytes speckleRange    

    */
    class Params
    {
        public:
            Params(const char* file) : file(file)
            {
                valid = false;
                _lock_prev = false;
            }

            void start();
            
            void stop();

            bool check();

            bool opened(){ return valid; }

            void lock(){ ((uint8_t*)map)[0] = 0xFF; }

            void unlock(){ ((uint8_t*)map)[0] = 0; }

            int minDisparity(){ return ((uint8_t*)map)[1]; }

            int numDisparities(){ return ((uint16_t*)map)[1]; }

            int blockSize(){ return ((uint16_t*)map)[2]; }

            int P1(){ return ((uint16_t*)map)[3]; }

            int P2(){ return ((uint16_t*)map)[4]; }

            int disp12MaxDiff(){ return ((uint16_t*)map)[5]; }

            int preFilterCap(){ return ((uint16_t*)map)[6]; }

            int uniquenessRatio(){ return ((uint16_t*)map)[7]; }
            
            int speckleWindowSize(){ return ((uint16_t*)map)[8]; }

            int speckleRange(){ return ((uint16_t*)map)[9]; }

        protected:
            const size_t size = 20;
            const char* file;
            int fd;
            void* map;
            bool valid;
            bool _lock_prev;
    };

    extern std::unique_ptr<Params> params;
}