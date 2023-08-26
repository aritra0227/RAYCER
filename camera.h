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

#define THREAD_COUNT 1

static std::vector<colour> COLOUR_VEC;
// std::mutex world_mut;
std::mutex task_mutex;
std::queue<std::pair<int, int>> TASK_Q;
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

    void render(const hittable &world)
    {
        initialize();

        cout << "P3\n"
             << image_width << ' ' << image_height << "\n255\n";
        for (int j = 0; j < image_height; ++j)
        {
            for (int i = 0; i < image_width; ++i)
            {
                TASK_Q.push(std::make_pair(i, j));
            }
        }
        for (int k = 0; k < THREAD_COUNT; ++k)
        {
            clog << "thread number: "<< k+1 << " initialized \n";
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
            std::pair<int, int> task_index;
            int curr_q_size;
            {
                std::lock_guard<std::mutex> lock(task_mutex);
                if (TASK_Q.empty())
                {
                curr_q_size = 0;
                    break; //exit because no more tasks available
                }
                curr_q_size = TASK_Q.size();
                task_index = TASK_Q.front();
                TASK_Q.pop();
            }
            this->colour_pixel(task_index.first, task_index.second, world);
            clog << "\rScanlines remaining: " << ((image_height * image_width) - curr_q_size) << ' ' << flush;
        }
    }
    void colour_pixel(int i, int j, const hittable &world)
    {
        colour pixel_color(0, 0, 0);
        for (int sample = 0; sample < samples_per_pixel; ++sample)
        {
            ray r = get_ray(i, j);
            // std::lock_guard<std::mutex> guard(world_mut);
            pixel_color += ray_colour(r, max_depth, world);
        }
        COLOUR_VEC[j * image_width + i] = pixel_color;

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
            // std::lock_guard<std::mutex> guard(world_mut);
            world_hit = world.hit(r, interval(0.001, infinity), rec);
        }
        if (world_hit)
        {
            ray scattered;
            colour attenuation;
            if (rec.mat->scatter(r, rec, attenuation, scattered))
                return attenuation * ray_colour(scattered, depth - 1, world);
            return colour(0, 0, 0);
        }
        vec3 unit_direction = unit_vector(r.direction());
        double a = 0.5 * (unit_direction.y() + 1.0); //converts from range [-1,1] to [0,1]
        return (1.0 - a) * colour(1.0, 1.0, 1.0) + a * colour(0.5, 0.7, 1.0);
        // return colour(1.0,1.0,1.0);
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