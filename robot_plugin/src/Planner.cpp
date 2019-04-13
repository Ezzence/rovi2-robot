#include "Planner.h"
#include <math.h>
 
#include <rws/RobWorkStudio.hpp>
#include <qtimer.h>
#include <rw/loaders.hpp>


using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::sensor;

using namespace rws;


Planner::Planner(rw::models::WorkCell::Ptr wc, rw::kinematics::State::Ptr state, rw::models::Device::Ptr device)
{
    _wc = wc;
    _state = state;
    _device = device;
}

Planner::~Planner()
{
}
