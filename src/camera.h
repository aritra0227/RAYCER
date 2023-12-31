#ifndef CAMERA_H
#define CAMERA_H

#include "utilities.h"

#include "colour.h"
#include "hittable.h"
#include "material.h"
#include <thread>
#include <mutex>
#include <queue>
using namespace std;

#define THREAD_COUNT 20

static unsigned int NUM_TASK;

static std::vector<colour> COLOUR_VEC;
std::mutex task_mutex;
std::queue<int> TASK_Q;
std::vector<std::thread> PIXEL_THREADS(THREAD_COUNT);

class camera
{
public:
    //Image
    double aspect_ratio = 16.0 / 9.0;
    int image_width = 400;
    int samples_per_pixel = 10; // Count of random samples for each pixel
    int max_depth = 10;         // Maximum number of ray bounces into scene

    double vfov = 90;                   // Vertical view angle (field of view)
    point3 lookfrom = point3(0, 0, -1); // Point camera is looking from
    point3 lookat = point3(0, 0, 0);    // Point camera is looking at
    vec3 vup = vec3(0, 1, 0);           // Camera-relative "up" direction

    double defocus_angle = 0; // Variation angle of rays through each pixel
    double focus_dist = 10;   // Distance from camera lookfrom point to plane of perfect focus
    // colour background_colour = colour(1.0, 1.0, 1.0);
    colour background_colour = colour(0.70, 0.80, 1.00);
    bool contains_external_light_source = false;

    /**
     * CAUTION: Multithreaded implementation!!!
     * A Thread is assigned to each row and 
     * no locks are used other than for accessing the task queue
    */
    void render(const hittable &world)
    {
        initialize();

        cout << "P3\n"
             << image_width << ' ' << image_height << "\n255\n";
        for (int j = 0; j < image_height; ++j)
        {
            TASK_Q.push(j);
            NUM_TASK++;
        }
        for (int k = 0; k < THREAD_COUNT; ++k)
        {
            clog << "thread number: " << k + 1 << " initialized \n";
            PIXEL_THREADS[k] = std::thread(&camera::assign_thread_task, this, std::ref(world));
        }
        for (auto &th : PIXEL_THREADS)
        {
            th.join();
        }

        for (colour pixel : COLOUR_VEC)
        {
            write_colour(std::cout, pixel, samples_per_pixel);
        }
        clog << "\rDone.                 \n";
    }

    void assign_thread_task(const hittable &world)
    {
        while (true)
        {
            int pixel_column;
            int curr_q_size;
            {
                std::lock_guard<std::mutex> lock(task_mutex);
                if (TASK_Q.empty())
                {
                    curr_q_size = 0;
                    break; //exit because no more tasks available
                }
                pixel_column = TASK_Q.front();
                TASK_Q.pop();
                clog << "\rScanlines remaining: " << --NUM_TASK << ' ' << flush;
            }
            this->colour_pixel(pixel_column, world);
        }
    }
    void colour_pixel(int pixel_column, const hittable &world)
    {
        for (int i = 0; i < image_width; ++i)
        {
            colour pixel_color(0, 0, 0);
            for (int sample = 0; sample < samples_per_pixel; ++sample)
            {
                ray r = get_ray(i, pixel_column);
                pixel_color += ray_colour(r, max_depth, world);
            }
            COLOUR_VEC[(pixel_column * image_width) + i] = pixel_color;
        }
        return;
    }

private:
    int image_height;     // Rendered image height
    point3 camera_center; // Camera center
    point3 pixel00_loc;   // Location of pixel 0, 0
    vec3 pixel_delta_u;   // Offset to pixel to the right
    vec3 pixel_delta_v;   // Offset to pixel below
    vec3 u, v, w;         // Camera frame basis vectors
    vec3 defocus_disk_u;  // Defocus disk horizontal radius
    vec3 defocus_disk_v;  // Defocus disk vertical radius

    void initialize()
    {
        image_height = static_cast<int>(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;
        COLOUR_VEC.resize(image_height * image_width); //allocate vector for number of pixels
        // PIXEL_THREADS.resize(image_height);
        camera_center = lookfrom;

        // Determine viewport dimensions.
        auto theta = degrees_to_radians(vfov);
        auto h = tan(theta / 2);
        auto viewport_height = 2 * h * focus_dist;
        double viewport_width = viewport_height * (static_cast<double>(image_width) / image_height);

        // Calculate the u,v,w unit basis vectors for the camera coordinate frame.
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        vec3 viewport_u = viewport_width * u;   // Vector across viewport horizontal edge
        vec3 viewport_v = viewport_height * -v; // Vector down viewport vertical edge

        // Calculate the horizontal and vertical delta vectors from pixel to pixel.
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // Calculate the location of the upper left pixel.
        vec3 viewport_upper_left = camera_center - (focus_dist * w) - viewport_u / 2 - viewport_v / 2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        // Calculate the camera defocus disk basis vectors.
        double defocus_radius = focus_dist * tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }
    colour ray_colour(const ray &r, int depth, const hittable &world)
    {
        hit_record rec;
        if (depth <= 0)
            return colour(0, 0, 0);
        bool world_hit = false;
        {
            world_hit = world.hit(r, interval(0.001, infinity), rec);
        }
        if (world_hit)
        {
            ray scattered;
            colour attenuation;
            colour light_emitted = rec.mat->emit_light();
            if (rec.mat->scatter(r, rec, attenuation, scattered))
                return (attenuation * ray_colour(scattered, depth - 1, world)) + light_emitted;
            return light_emitted;
        }
        else
            return background_colour;
    }

    ray get_ray(int i, int j) const
    {
        // Get a randomly sampled camera ray for the pixel at location i,j from camera defocus dist

        point3 pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
        point3 pixel_sample = pixel_center + pixel_sample_square();

        point3 ray_origin = (defocus_angle <= 0) ? camera_center : defocus_disk_sample();
        point3 ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
    }
    vec3 pixel_sample_square() const
    {
        // Returns a random point in the square surrounding a pixel at the origin.
        double px = -0.5 + random_double();
        double py = -0.5 + random_double();
        return (px * pixel_delta_u) + (py * pixel_delta_v);
    }

    point3 defocus_disk_sample() const
    {
        // Returns a random point in the camera defocus disk.
        vec3 p = random_in_unit_disk();
        return camera_center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }
};

#endif