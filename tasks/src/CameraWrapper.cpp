#include <CameraWrapper.hpp>
#include <Logging.hpp>
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
    {
        Logger::instance() << "[WARNING][camera-requestComplete] Request cancelled" << std::endl;
        return;
    }    
    // Get the associated CameraWrapper and the index of this request in its request vector
    CameraWrapper* camera;
    uint8_t req_idx;
    std::tie(camera, req_idx) = resolve_cookie(request->cookie());
    // Tell the wrapper that this index is the latest and the next should be queued for updating.
    camera->set_freshest(req_idx);
}


bool CameraWrapper::connect()
{
    // Check all cameras
    for (auto const &camera : cm->manager()->cameras())
    {
        if(check(camera->id()))
        {
            // This cameras id matches what this wrapper should attach to
            info("Attaching to detected camera ", camera->id() );
            this->camera = cm->manager()->get(camera->id());
            return true;
        }
    }
    error("No camera matching the given criteria was detected.");
    return false;
}


void CameraWrapper::configure()
{
    // Generate the default configuration and change pixel format to YVU.
    // This provides a 3 plane formate with intensity in plane 0, so that
    // extracting grayscale is fast and simple.

    config = camera->generateConfiguration( { StreamRole::Viewfinder } );
    info("Default viewfinder configuration is: ", config->at(0).toString());

    config->at(0).pixelFormat = PixelFormat::fromString("YVU420"); 
    config->validate();   
    // width = config->at(0).size.width;
    // height = config->at(0).size.height;
    info("Validated viewfinder configuration is: ", config->at(0).toString());
    camera->configure(config.get());

    // Allocate all buffers for this config. The number of buffers used is defined by the configuration.
    allocator = new FrameBufferAllocator(camera);
    for (StreamConfiguration &cfg : *config) {
        int ret = allocator->allocate(cfg.stream());
        if (ret < 0) {
            error(name, ": Can't allocate buffers");
            return;
        }

        size_t allocated = allocator->buffers(cfg.stream()).size();
        info("Allocated ", allocated, " buffers for stream");
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
            error("Can't create request");
            return;
        }

        const std::unique_ptr<FrameBuffer> &buffer = (*buffers)[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0)
        {
            error("Can't set buffer for request");
            return;
        }

        requests.push_back(std::move(request));

        // Create a memory mapped array from the planes file descriptor.
        // Only plane 0 is used from each buffer, since we are only using grayscale images.
        const FrameBuffer::Plane& plane = buffer->planes()[0];
        info("Mapping plane of size ", plane.length, " at offset ", plane.offset);
        bytes = plane.length;
        void* plane_map = mmap(0, bytes, PROT_READ , MAP_SHARED, plane.fd.get(), plane.offset);
        if(plane_map == MAP_FAILED)
        {
            error("Failed to map memory: ", strerror(errno));
            return;
        }
        map.push_back(plane_map);
    }

    camera->requestCompleted.connect(requestComplete);
}


bool CameraWrapper::start_impl()
{
    if(!connect()) return false;

    camera->acquire();
    configure();
    camera->start();
    camera->queueRequest(requests[0].get());
    return true;
}


void CameraWrapper::stop_impl()
{
    camera->stop();
    for(const auto& request : requests)
    {
        delete_cookie(request->cookie());
    }
    allocator->free(stream);
    delete allocator;
    camera->release();
    camera.reset();
}


void CameraWrapper::set_freshest(uint8_t idx)
{ 
    tick();
    
    // Queue next request to begin capture of next frame
    uint8_t next = idx+1;
    if( next >= requests.size())
    {
        next = 0;
    }    
    requests[next]->reuse(Request::ReuseBuffers);
    if(is_alive())
    {
        camera->queueRequest(requests[next].get());
    }

    // Copy this reuests data to datasource interface
    swap_data(*reinterpret_cast<uint8_t(*)[Width*Height]>(map[idx]));
}
