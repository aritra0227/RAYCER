#ifndef BVH_H
#define BVH_H

#include "acceleration.h"
#include "interval.h"
#include "material.h"
/**
 * TLAS: Top Level Acceleration Structure
 * Should mimic hittable_list since that is what we will be replacing:
*/

class BVH : public accel
{
private:
    static const int num_plane_set_normals = 7;
    static const vec3 plane_set_normals[num_plane_set_normals];
    /**
    * bounding box enclosing an object
    */
    struct bbox
    {
        interval bounds[num_plane_set_normals];
        bbox() {}
        bool hit(double *NdotOrig, double *NdotDir, double &tnear, double &tfar, size_t &pi)
        {
            for (size_t i = 0; i < num_plane_set_normals; ++i)
            {
                double tn = (bounds[i].min - NdotOrig[i]) / NdotDir[i];
                double tf = (bounds[i].max - NdotOrig[i]) / NdotDir[i];
                if (NdotDir[i] < 0)
                    std::swap(tn, tf);
                if (tn > tnear)
                    tnear = tn, pi = i;
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
                objects[i]->compute_bounds(plane_set_normals[j], objects_bounds[i].bounds[j].min, objects_bounds[i].bounds[j].max);
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
        bool hit_any_objects = false;
        // std::clog << "NUMBER OF OBJECTS " << objects.size() << std::endl;
        hit_record temp_rec;
        for (size_t i = 0; i < objects.size(); ++i)
        {
            double tnear = ray_t.min, tfar = tclosest_object_so_far;
            size_t pi;
            if (objects_bounds[i].hit(NdotOrig, NdotDir, tnear, tfar, pi))
            {
                if (tnear < tclosest_object_so_far) //cloest bounding box hit
                {
                    if (objects[i]->hit(r, ray_t, temp_rec))
                    {
                        hit_any_objects = true;
                        tclosest_object_so_far = temp_rec.t;
                        rec = temp_rec;
                    }
                }
            }
        }
        return hit_any_objects;
    }
};
const vec3 BVH::plane_set_normals[BVH::num_plane_set_normals] = {
    vec3(1, 0, 0),
    vec3(0, 1, 0),
    vec3(0, 0, 1),
    vec3((sqrtf(3) / 3.f), (sqrtf(3) / 3.f), (sqrtf(3) / 3.f)),
    vec3(-sqrtf(3) / 3.f, sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    vec3(-sqrtf(3) / 3.f, -sqrtf(3) / 3.f, sqrtf(3) / 3.f),
    vec3(sqrtf(3) / 3.f, -sqrtf(3) / 3.f, sqrtf(3) / 3.f)};

#endif