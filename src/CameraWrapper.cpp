#include <CameraWrapper.hpp>
#include <thread>
#include <unordered_map>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <errno.h>
#include <cstring>


using namespace libcamera;
using namespace std::chrono_literals;

static std::unordered_map<uint64_t, std::pair<CameraWrapper*, uint8_t>> cookie_to_camera;

/** Generates a unique cookie value for a CameraWrapper instance.

The callers should cleanup the generated cookie with delete_cookie
when they go out of scope.

@return New cookie value
*/
static uint64_t generate_cookie()
{
    uint64_t out = 0;
    for(auto& it : cookie_to_camera)
    {
        if(it.first > out)
        {
            out = it.first;
        }
    }
    return out + 1;
}


static void store_cookie(uint64_t cookie, CameraWrapper* camera, uint8_t request_id)
{
    cookie_to_camera[cookie] = std::make_pair(camera, request_id);
}


static std::pair<CameraWrapper*, uint8_t>& resolve_cookie(uint64_t cookie)
{
    return cookie_to_camera[cookie];
}

static void delete_cookie(uint64_t cookie)
{
    cookie_to_camera.erase(cookie);
}


static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
        return;

    std::pair<CameraWrapper*, uint8_t>& caller = resolve_cookie(request->cookie());
    FrameBuffer* buffer = request->buffers().begin()->second;
    const FrameMetadata &metadata = buffer->metadata();

    std::cout << '\t' << " seq: " << metadata.sequence << " planes: " << metadata.planes().size() << ", bytes used: ";
    unsigned int nplane = 0;
    for (const FrameMetadata::Plane &plane : metadata.planes())
    {
        std::cout << plane.bytesused << " (fd " << buffer->planes().at(0).fd.get() << ")";
        if (++nplane < metadata.planes().size()) std::cout << "/";
    }
    std::cout << std::endl;

    caller.first->set_freshest(caller.second);

    request->reuse(Request::ReuseBuffers);
    std::cout << "Restarting request" << std::endl;
    caller.first->get_camera()->queueRequest(request);
}


void CameraWrapper::acquire()
{
    camera->acquire();
}


void CameraWrapper::release()
{   
    for(const auto& request : requests)
    {
        delete_cookie(request->cookie());
    }
    allocator->free(stream);
    delete allocator;
    camera->release();
    camera.reset();
}


void CameraWrapper::configure()
{
    config = camera->generateConfiguration( { StreamRole::Viewfinder } );
    std::cout << name << ": Default viewfinder configuration is: " << config->at(0).toString() << std::endl;
    config->at(0).pixelFormat = PixelFormat::fromString("YVU420"); 
    config->validate();   
    width = config->at(0).size.width;
    height = config->at(0).size.height;
    std::cout << name << ": Validated viewfinder configuration is: " << config->at(0).toString() << std::endl;
    camera->configure(config.get());

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

    stream = config->at(0).stream();
    buffers = &allocator->buffers(stream);
    stride = stream->configuration().stride;

    for (unsigned int i = 0; i < buffers->size(); ++i) {
        std::unique_ptr<Request> request = camera->createRequest(generate_cookie());
        store_cookie(request->cookie(), this, i);
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

        const FrameBuffer::Plane& plane = buffer->planes().at(0);
        std::cout << name << ": Mapping plane of size " << plane.length << " at offset " << plane.offset << std::endl;
        bytes = plane.length;
        void* plane_map = mmap(0, bytes, PROT_READ , MAP_SHARED, plane.fd.get(), plane.offset);
        if(plane_map == MAP_FAILED)
        {
            std::cerr << name << ": Failed to map memory: " << strerror(errno) << std::endl;
            return;
        }
        map.push_back(plane_map);
        matrices.push_back(cv::Mat(height, width, cv::DataType<uint8_t>::type, plane_map, std::max(stride, width)));
    }

    camera->requestCompleted.connect(requestComplete);
}


void CameraWrapper::start()
{
    camera->start();

    for(const auto& request : requests)
    {
        camera->queueRequest(request.get());
    }
}


void CameraWrapper::stop()
{
    camera->stop();
}


std::shared_ptr<libcamera::Camera> CameraWrapper::get_camera(){ return camera; }
   

std::tuple<size_t, size_t, size_t> CameraWrapper::shape(){ return {height, width, stride}; }


void* CameraWrapper::data()
{
    return map[freshest_buffer];
}


size_t CameraWrapper::size(){ return bytes; }


void CameraWrapper::set_freshest(uint8_t idx){ freshest_buffer = idx; }

