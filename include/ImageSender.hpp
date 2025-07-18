#pragma once

#include <sys/socket.h>
#include <netinet/in.h>
#include <opencv2/core/mat.hpp>

/** Transmits images over sockets for other applications to view.
*/
class ImageSender
{
    public:
    /** Create a new image sender.
    
    @param port Port to listen for clients on
    */
    ImageSender(const unsigned port): port(port), accepted(false){}

    /** Starts the server 
    */
    void start();

    /** Closes the server
    */
    void stop();

    /** Sends a new image with timestamp
    
    @param image Image data to send
    */
    void transmit(const cv::Mat& image);

    protected:
    int server, client;
    sockaddr_in addr;
    const unsigned port;
    bool accepted;
};