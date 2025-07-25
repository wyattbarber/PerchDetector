#include <CameraWrapper.hpp>
#include <thread>

using namespace libcamera;
using namespace std::chrono_literals;


static void requestComplete(Request *request)
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
    
    for (unsigned int i = 0; i < buffers->size(); ++i) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request)
        {
            std::cerr << name << ": Can't create request" << std::endl;
            return;
        }
        request.setCookie((uint64_t)this); // Set request cookie to be a pointer to self

        const std::unique_ptr<FrameBuffer> &buffer = (*buffers)[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0)
        {
            std::cerr << name << ": Can't set buffer for request" << std::endl;
            return;
        }

        requests.push_back(std::move(request));

        FrameBuffer::Plane& plane = buffer->planes(0);
        std::cout << name << : ": Mapping plane of size " << plane.length << " at offset " << plane.offset << std::endl;
        void plane_map = mmap(0, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), plane.offset);
        if(map == MAP_FAILED)
        {
            std::cerr << name << ": Failed to map memory: " << strerror(errno) << std::endl;
            return;
        }
        map.push_back(plane_map);
    }

}


void CameraWrapper::start()1
{
    camera->start();

    for(auto request : requests)
    {
        camera->queueRequest(request.get());
    }
}


void CameraWrapper::stop()
{
    camera->stop();
}

   