#include <ImageSender.hpp>
#include <unistd.h>

void ImageSender::start()
{
    server = socket(AF_INET, SOCK_STREAM, 0);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;

    bind(server, (const sockaddr*)&addr, sizeof(addr));
    listen(server, 5);
}


void ImageSender::stop()
{
    close(server);
}


void ImageSender::transmit(const cv::Mat& image)
{
    if(!accepted)
    {
        client = accept(server, nullptr, nullptr);
        accepted = true;
    }

    send(client, image.data, (image.dataend - image.data), 0);
}