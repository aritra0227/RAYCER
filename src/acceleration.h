#ifndef ACCELERATION_H
#define ACCELERATION_H
#include "hittable.h"
#include <vector>
using namespace std;
/**
 * Acceleration Structure.
 * Currently being used for tlas
 * but can probably reuse for Bottom level as well
*/

class accel : public hittable
{
public:
    virtual bool hit(const ray &r, interval ray_t, hit_record &rec) const = 0;
    virtual ~accel(){};
    std::vector<shared_ptr<hittable>> objects;
};

#endif