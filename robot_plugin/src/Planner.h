
#ifndef PLANNER_H
#define PLANNER_H

#include <QThread>
#include <QCoreApplication>

#include <ros/ros.h>
#include <rw/models.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/pathplanning.hpp>
#include <rw/proximity.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace rw;
using namespace rw::math;

class Planner : public QThread
{
    Q_OBJECT
public:
    Planner(){
        int boo;
    }
    // weird bug with robwork signature _ZTV7Planner was because this class was not added into the cmake with the QT_WRAPPER_CPP
    Planner(rw::models::WorkCell::Ptr wc, rw::kinematics::State::Ptr state, rw::models::Device::Ptr device, QObject* parent = nullptr);
    ~Planner(){}

    void run();

    bool initRRT();
    bool plan(Q target, rw::trajectory::QPath &path);

    void debugPath(rw::trajectory::QPath &path);

    rw::trajectory::QPath _path;

public slots:

    void callPlan(Q target);

signals:


private:


    models::WorkCell::Ptr _wc;
    kinematics::State::Ptr _state;
    models::Device::Ptr _device;

    proximity::CollisionDetector::Ptr _collisionDet;

    // The q constraint is to avoid collisions.
    rw::pathplanning::QConstraint::Ptr _constraint;

    pathplanning::QEdgeConstraint::Ptr _edgeconstraint;
    pathplanning::PlannerConstraint _pConstraint;

    proximity::CollisionDetector::Ptr _detector;

    pathplanning::QSampler::Ptr _cfreeQ;
    pathplanning::QToQPlanner::Ptr _planner;
    // A sampler of collision free configurations for the device.
    //rw::pathplanning::QSampler::Ptr cfreeQ = QSampler::makeConstrained(QSampler::makeUniform(device), constraint);


};

#endif

