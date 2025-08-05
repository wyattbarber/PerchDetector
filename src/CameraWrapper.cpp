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
#include <tuple>


using namespace libcamera;
using namespace std::chrono_literals;

/** Map of libcamera::Request cookie values to CameraWrapper instances and Request numbers.
 * 
 * Since the request completion callback doesn't know which camera a request came from,
 * additional data is needed to map the completed request signal into the correct context.
 * This map is used to store these accosiations, and is interacted with through the following
 * functions:
 * 
 * * generate_cookie(): Creates and reserves a new unique cookie value
 * 
 * * store_cookie(): Associates a created cookie to a CameraWrapper and Request number
 * 
 * * resolve_cookie(): Get the assiciated pointer and number for a cookie
 * 
 * * delete_cookie(): Removes a cookie and its associations from the map
 * 
 * This variable and the above functions are only used by the camera driver, and are not 
 * needed in application code.
 */
static std::unordered_map<uint64_t, std::pair<CameraWrapper*, uint8_t>> cookie_to_camera;


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
    cookie_to_camera[out+1] = std::make_pair(nullptr, 0); // reserve the new cookie
    return out+1;
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


/** Callback for the completion of frame buffer requests. 
 * 
 * Setup and called by the camera driver. Uses cookie_to_camera and
 * its associated functions to determine which CameraWrapper instance
 * a completed request is associated to.
 */
static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
        return;

    // Get the associated CameraWrapper and the index of this request in its request vector
    CameraWrapper* camera;
    uint8_t req_idx;
    std::tie(camera, req_idx) = resolve_cookie(request->cookie());
    // Tell the wrapper that this index is the latest and the next should be queued for updating.
    camera->set_freshest(req_idx);
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
    // Generate the default configuration and change pixel format to YVU.
    // This provides a 3 plane formate with intensity in plane 0, so that
    // extracting grayscale is fast and simple.

    config = camera->generateConfiguration( { StreamRole::Viewfinder } );
    std::cout << name << ": Default viewfinder configuration is: " << config->at(0).toString() << std::endl;
    config->at(0).pixelFormat = PixelFormat::fromString("YVU420"); 
    config->validate();   
    width = config->at(0).size.width;
    height = config->at(0).size.height;
    std::cout << name << ": Validated viewfinder configuration is: " << config->at(0).toString() << std::endl;
    camera->configure(config.get());

    // Allocate all buffers for this config. The number of buffers used is defined by the configuration.
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

    // Create a request object for each buffer, which will be used to 
    // trigger the camera to fill the buffer with new data.
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

        // Create a memory mapped array from the planes file descriptor.
        // Only plane 0 is used from each buffer, since we are only using grayscale images.
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
    camera->queueRequest(requests[0].get());
    running = true;
}


void CameraWrapper::stop()
{
    running = false;
    camera->stop();
}


void CameraWrapper::set_freshest(uint8_t idx)
{ 
    freshest_buffer = idx; 
    uint8_t next = idx+1;
    if( next == locked_idx)
    {
        // Skip this buffer if it is locked.
        next += 1;
    }
    if( next >= requests.size() )
    {
        // Loop around the end of the request vector, skipping index 0 if it is locked.
        next = (locked_idx == 0) ? 1 : 0;
    }    

    requests[next]->reuse(Request::ReuseBuffers);
    if(running)
    {
        // std::cout << "Request " << (int)idx << " finished, restarting request " << (int)next << std::endl;
        camera->queueRequest(requests[next].get());
    }
}


void CameraWrapper::lock()
{
    locked_idx = freshest_buffer;
}


void CameraWrapper::unlock()
{
    locked_idx = -1;
}