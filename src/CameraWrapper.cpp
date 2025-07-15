#include <CameraWrapper.hpp>
#include <thread>

using namespace libcamera;
using namespace std::chrono_literals;

void CameraWrapper::acquire()
{
    camera->acquire();
}


void CameraWrapper::release()
{
    camera->release();
    camera.reset();
}


void CameraWrapper::configure(unsigned width, unsigned height)
{
    config = camera->generateConfiguration( { StreamRole::Viewfinder } );
    StreamConfiguration &streamConfig = config->at(0);
    std::cout << name << ": Default viewfinder configuration is: " << streamConfig.toString() << std::endl;

    streamConfig.size.width = width;
    streamConfig.size.height = height;
    config->validate();
    std::cout << name << ": Validated viewfinder configuration is: " << streamConfig.toString() << std::endl;
    camera->configure(config.get());

    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);
    for (StreamConfiguration &cfg : *config) {
        int ret = allocator->allocate(cfg.stream());
        if (ret < 0) {
            std::cerr << name << ": Can't allocate buffers" << std::endl;
            return;
        }

        size_t allocated = allocator->buffers(cfg.stream()).size();
        std::cout << name << ": Allocated " << allocated << " buffers for stream" << std::endl;
    }

    stream = streamConfig.stream();
    buffers = &allocator->buffers(stream);

    for (unsigned int i = 0; i < buffers->size(); ++i) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request)
        {
            std::cerr << name << ": Can't create request" << std::endl;
            return;
        }

        const std::unique_ptr<FrameBuffer> &buffer = (*buffers)[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0)
        {
            std::cerr << name << ": Can't set buffer for request" << std::endl;
            return;
        }

        requests.push_back(std::move(request));
    }
}


void CameraWrapper::start()
{
    camera->start();
}


void CameraWrapper::stop()
{
    camera->stop();
}


void CameraWrapper::capture_start()
{
    camera->queueRequest(requests[0].get());
}


FrameBuffer* CameraWrapper::capture_wait()
{
    // Block thread until frame is available
    while(requests[0]->status() != Request::RequestComplete)
        std::this_thread::sleep_for(10ms);

    const std::map<const Stream *, FrameBuffer *> &buffers = requests[0]->buffers();
    
    FrameBuffer* buffer;    
    for (auto bufferPair : buffers) {
        buffer = bufferPair.second;
    }
    return buffer;
}   