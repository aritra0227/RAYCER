#ifndef BVH_H
#define BVH_H

#include "acceleration.h"
#include "interval.h"

/**
 * TLAS: Top Level Acceleration Structure
 * Should mimic hittable_list since that is what we will be replacing:
*/

class BVH : public accel
{
private:
    static const int num_plane_set_normals = 3;
    static const vec3 plane_set_normals[num_plane_set_normals];
    /**
    * bounding box enclosing an object
    */
    struct bbox
    {
        double bounds[num_plane_set_normals][2]; // [i][0] = min, [i][1] = max
        bbox()
        {
            for (size_t i = 0; i < num_plane_set_normals; ++i)
            {
                bounds[i][0] = infinity;
                bounds[i][1] = -infinity;
            }
        }
        bool hit(double *NdotOrig, double *NdotDir, double &tnear, double &tfar)
        {
            for (size_t i = 0; i < num_plane_set_normals; ++i)
            {
                double tn = (bounds[i][0] - NdotOrig[i]) / NdotDir[i];
                double tf = (bounds[i][1] - NdotOrig[i]) / NdotDir[i];
                if (NdotDir[i] < 0)
                    std::swap(tn, tf);
                if (tn > tnear)
                    tnear = tn;
                if (tf < tfar)
                    tfar = tf;
                if (tnear > tfar)
                    return false;
            }
            return true;
        }
    };
    bbox *objects_bounds;

public:
    BVH() {}
    void add(shared_ptr<hittable> object)
    {
        objects.push_back(object);
    }
    // must be called after adding all objects
    void compute_object_bounds()
    {
        objects_bounds = new bbox[objects.size()];
        //calculate bounds for each object
        for (size_t i = 0; i < objects.size(); ++i)
        {
            for (size_t j = 0; j < num_plane_set_normals; ++j)
            {
                objects[i]->compute_bounds(plane_set_normals[j], objects_bounds[i].bounds[j][0], objects_bounds[i].bounds[j][1]);
            }
        }
    }
    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        //plane intersection equation: f(d) = (d - N.O)/(N.RD), ;;; . is for dot product
        //tnearest = f(dnearest), tfarthest = f(dfarthest);;; N = Normal, RD = ray direction
        //precompute N.O and N.RD
        double NdotOrig[num_plane_set_normals];
        double NdotDir[num_plane_set_normals];
        for (size_t i = 0; i < num_plane_set_normals; ++i)
        {
            NdotOrig[i] = dot(plane_set_normals[i], r.origin());
            NdotDir[i] = dot(plane_set_normals[i], r.direction());
        }
        //now just iterate for now to find the closest bbox:
        double tclosest_object_so_far = ray_t.max;
        bool hit_any_boxes = false;
        size_t closest_object_index;
        std::clog << "NUMBER OF OBJECTS " << objects.size() << std::endl;
        for (size_t i = 0; i < objects.size(); ++i)
        {
            double tnear = -infinity, tfar = infinity;
            if (objects_bounds[i].hit(NdotOrig, NdotDir, tnear, tfar))
            {
                if (tnear < tclosest_object_so_far)
                {
                    tclosest_object_so_far = tnear;
                    hit_any_boxes = true;
                    closest_object_index = i;
                }
            }
        }
        if (hit_any_boxes)
        {
            // return objects[closest_object_index]->hit(r, interval(ray_t.min, tclosest_object_so_far), rec);
            return objects[closest_object_index]->hit(r, ray_t, rec);
        }
        return false;
    }
};
const vec3 BVH::plane_set_normals[BVH::num_plane_set_normals] = {
    vec3(1, 0, 0),
    vec3(0, 1, 0),
    vec3(0, 0, 1)};
    // vec3((sqrtf(3) / 3.f), (sqrtf(3) / 3.f), (sqrtf(3) / 3.f)),
    // vec3(-sqrtf(3) / 3.f, sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    // vec3(-sqrtf(3) / 3.f, -sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    // vec3(sqrtf(3) / 3.f, -sqrtf(3) / 3.f, sqrtf(3) / 3.f)};

#endif