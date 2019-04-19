#include "Planner.h"

#include <math.h>
#include <qtimer.h>


#include <rw/loaders.hpp>


using namespace rw;
using namespace rw::math;


Planner::Planner(rw::models::WorkCell::Ptr wc, rw::kinematics::State::Ptr state, rw::models::Device::Ptr device)
{
    _wc = wc;
    _state = state;
    _device = device;

    //_collisionDet = new proximity::CollisionDetector(wc.get(), rwlibs::proximitystrategies::ProximityStrategyYaobi::make());
    _collisionDet = new proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());


}

Planner::~Planner()
{
}

bool Planner::initRRT()
{
    _constraint = pathplanning::QConstraint::make(_collisionDet, _device, *_state);
    //_detector = new proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    // the edge constraint tests the constraint on edges, eg. edge between two configurations
    _edgeconstraint = pathplanning::QEdgeConstraint::make(pathplanning::QConstraint::make(_collisionDet, _device, *_state), math::MetricFactory::makeEuclidean<Q>(), 0.01);
    //_edgeconstraint = rw::pathplanning::QEdgeConstraintIncremental::makeDefault(_constraint, _device);

    _pConstraint = pathplanning::PlannerConstraint::make(_constraint, _edgeconstraint);

    // A sampler of collision free configurations for the device.
    _cfreeQ = pathplanning::QSampler::makeConstrained(pathplanning::QSampler::makeUniform(_device), _constraint);

    // An RRTConnect based point-to-point path planner.
    _planner = rwlibs::pathplanners::RRTQToQPlanner::makeConnect(_pConstraint, _cfreeQ, math::MetricFactory::MetricFactory::makeEuclidean<Q>(), 0.1);


    // Plan 10 paths to sampled collision free configurations.
    /*for (int cnt = 0; cnt < 10; cnt++) {
        const Q next = _cfreeQ->sample();
        const bool ok = _planner->query(pos, next, path);
        if (!ok) {
            std::cout << "Path " << cnt << " not found.\n";
            //return false;
        } else {
            pos = next;
        }
    }*/



    return true;
}

bool Planner::plan(Q target)
{
    bool success = plan(target, _path);
    return success;
}

bool Planner::plan(Q target, trajectory::QPath &path)
{
    Q pos = _device->getQ(*_state);
    //rw::trajectory::Path<Q> path;
    _planner->query(pos, target, path);

    debugPath(path);

    if(path.empty())
    {
        return false;
    }
    // Map the configurations to a sequence of states.
    const std::vector<kinematics::State> states = models::Models::getStatePath(*_device, path, *_state);

    // Write the sequence of states to a file.
    loaders::PathLoader::storeVelocityTimedStatePath(*_wc, states, "rrt-path-planning.rwplay");

    return true;
}

void Planner::debugPath(trajectory::QPath &path)
{
    for(auto val : path)
    {
        ROS_INFO_STREAM("path: " << val[0] << " " << val[1] << " " << val[2] << " " << val[3] << " " << val[4] << " " << val[5]);
    }
    ROS_INFO_STREAM("path size: " << path.size());
}
