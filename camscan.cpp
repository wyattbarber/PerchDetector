#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

#include <libcamera/libcamera.h>


using namespace libcamera;
using namespace std::chrono_literals;


void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
        return;

    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();

    for (auto bufferPair : buffers) {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();

        std::cout << '\t' << " seq: " << std::setw(6) << std::setfill('0') << metadata.sequence << " planes: " << metadata.planes().size() << " bytesused: ";

        unsigned int nplane = 0;
        for (const FrameMetadata::Plane &plane : metadata.planes())
        {
            std::cout << plane.bytesused;
            if (++nplane < metadata.planes().size()) std::cout << "/";
        }

        std::cout << std::endl;
    }
}



template<typename T>
void disp_camera_config(T& camera)
{
    camera->acquire();
    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration( { StreamRole::Viewfinder } );
    StreamConfiguration &streamConfig = config->at(0);
    std::cout << '\t' << "Default viewfinder configuration is: " << streamConfig.toString() << std::endl;
    std::vector<PixelFormat> formats = streamConfig.formats().pixelformats();
    for (int i = 0; i < formats.size(); i++) {
       PixelFormat format = formats[i];
       std::cout << '\t' << "Found pixel format: " << format.toString() << std::endl;
    }
    streamConfig.pixelFormat = PixelFormat::fromString("YVU420");    
    camera->configure(config.get());

    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);
    for (StreamConfiguration &cfg : *config) {
        int ret = allocator->allocate(cfg.stream());
        if (ret < 0) {
            std::cerr << '\t' << "Can't allocate buffers" << std::endl;
            return;
        }

        size_t allocated = allocator->buffers(cfg.stream()).size();
        std::cout << '\t' << "Allocated " << allocated << " buffers for stream" << std::endl;
    }

    Stream *stream = streamConfig.stream();
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
    std::vector<std::unique_ptr<Request>> requests;
    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request)
        {
            std::cerr << '\t' << "Can't create request" << std::endl;
            return;
        }
	    std::cout << '\t' << "Created request " << request->toString() << std::endl;
        const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0)
        {
            std::cerr << '\t' << "Can't set buffer for request"
                << std::endl;
            return;
        }

        requests.push_back(std::move(request));
    }

    camera->requestCompleted.connect(requestComplete);

    camera->start();
    for (std::unique_ptr<Request> &request : requests)
        camera->queueRequest(request.get());

    std::this_thread::sleep_for(3000ms);

    camera->stop();
    allocator->free(stream);
    delete allocator;
    camera->release();
    camera.reset();
}


int main(int argc, char** argv)
{
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    for (auto &camera : cm->cameras())
    {
        std::cout << camera->id() << std::endl;
        // Display data about the cameras default configuration
        disp_camera_config(camera);
    }
    if(cm->cameras().empty())
        std::cout << "No cameras detected in this system." << std::endl;

    cm->stop();

    return 0;
}
