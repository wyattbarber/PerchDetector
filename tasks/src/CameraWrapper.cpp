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


template<typename T>
std::string opt_string(const std::optional<T>& opt)
{
    if(opt) {
        return std::to_string(*opt);
    } else {
        return "???";
    }
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
    info("Camera configured with default controls:");
    for(const auto & item : camera->controls())
    {
        info('\t', item.first->name(), " ", item.second.def().toString(), " ", item.second.toString());
    }

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
        void* plane_map = mmap(0, plane.length, PROT_READ , MAP_SHARED, plane.fd.get(), plane.offset);
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

    // Initialize basic settings
    camera->acquire();
    configure();
    camera->start();
    camera->queueRequest(requests[0].get());
    request_busy = true;
    // Let auto-exposue and awb settings converge for 60 frames before fixing them
    unsigned i = 0;
    int32_t exposure_time;
    float analog_gain, red_gain, blue_gain;
    while(i < 60)
    {
        // Wait for completion
        while(request_busy){}
        exposure_time = *(requests[i%requests.size()]->metadata().get(controls::ExposureTime));
        analog_gain = *(requests[i%requests.size()]->metadata().get(controls::AnalogueGain));
        red_gain = (*(requests[i%requests.size()]->metadata().get(controls::ColourGains)))[0];
        blue_gain = (*(requests[i%requests.size()]->metadata().get(controls::ColourGains)))[1];
        ++i;
        request_busy = true;
        requests[i%4]->reuse(Request::ReuseBuffers);
        camera->queueRequest(requests[i%requests.size()].get());
    }
    info("Checking converged auto-exposure and auto-white-balance settings.");
    info("\tExposureTime ", exposure_time);
    info("\tAnalogueGain ", analog_gain);
    info("\tColourGains {", red_gain, ", ", blue_gain, "}");
    
    // Submit settings to manager
    cm->submit_settings(name, exposure_time, analog_gain, red_gain, blue_gain);
    settings_finalized = false;

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


void CameraWrapper::step()
{
    if(!settings_finalized)
    {
        if(cm->n_settings_submitted() >= 2)
        {
            int32_t exposure_time;
            float analog_gain, red_gain, blue_gain;
            std::tie(exposure_time, analog_gain, red_gain, blue_gain) = cm->get_global_settings();
            info("Setting fixed auto-exposure and auto-white-balance settings.");
            info("\tExposureTime ", exposure_time);
            info("\tAnalogueGain ", analog_gain);
            info("\tColourGains {", red_gain, ", ", blue_gain, "}");
            for(auto& request : requests)
            {
                request->controls().set(controls::AeEnable, false);
                request->controls().set(controls::AwbEnable, false);
                request->controls().set(controls::ExposureTime, exposure_time);
                request->controls().set(controls::AnalogueGain, analog_gain);
                request->controls().set(controls::ColourGains, {red_gain, blue_gain});
            };
            settings_finalized = true;
        }
    }
    else
    {
        if(!request_busy)
        {
            tick();
            request_busy = true;
            requests[next]->reuse(Request::ReuseBuffers);
            camera->queueRequest(requests[next].get());
        }
        ++next;
        if(next >= requests.size())
        {
            next = 0;
        } 
    }
}


void CameraWrapper::set_freshest(uint8_t idx)
{    
    // Copy this requests data to datasource interface
    request_busy = false;
    auto update = allocate_next();
    uint8_t* src = (uint8_t*)map[idx];
    uint8_t* dst = update->data;
    for(unsigned i = 0; i < Height; ++i)
    {
        // Copy individual rows to remove stride
        memcpy((void*)dst, (void*)src, Width);
        dst += Width;
        src += stride;
    }
    swap_data();
}
