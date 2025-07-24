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
    allocator->free(stream);
    delete allocator;
    camera->release();
    camera.reset();
}


void CameraWrapper::configure()
{
    config = camera->generateConfiguration( { StreamRole::Viewfinder } );
    std::cout << name << ": Default viewfinder configuration is: " << config->at(0).toString(); << std::endl;
    camera->configure(config->at(0));

    allocator = new FrameBufferAllocator(camera);
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
}


void CameraWrapper::start()
{
    camera->start();

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
    camera->queueRequest(requests[0].get());
}


void CameraWrapper::stop()
{
    camera->stop();
}

   