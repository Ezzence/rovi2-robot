
#ifndef PLANNER_H
#define PLANNER_H

#include <QThread>
#include <QCoreApplication>
#include <QElapsedTimer>

#include <ros/ros.h>
#include <rw/models.hpp>
#include <rw/math/Q.hpp>
#include <rw/math/MetricFactory.hpp>
#include <rw/pathplanning.hpp>
#include <rw/proximity.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rwlibs/pathplanners/rrt/RRTNode.hpp>
#include <rwlibs/pathplanners/rrt/RRTTree.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace rw;
using namespace rw::math;
using namespace rwlibs::pathplanners;

class Planner : public QThread
{
    Q_OBJECT
public:
    Planner(){

    }
    // weird bug with robwork signature _ZTV7Planner was because this class was not added into the cmake with the QT_WRAPPER_CPP
    Planner(rw::models::WorkCell::Ptr wc, rw::kinematics::State::Ptr state, rw::models::Device::Ptr device, QObject* parent = nullptr);
    ~Planner(){}

    void run();

    bool initRRT();

    void debugPath(rw::trajectory::QPath &path);
    void debugTree(RRTTree<Q>& tree);

    rw::trajectory::QPath _path;

    enum ExtendResult { Trapped, Reached, Advanced };
    enum PlanSelect { RW_RRT = 0, RRT = 1, ARRT = 2};


public slots:

    void callPlan(Q target, int planSelect);

signals:


private:

    // default robwork RRT
    bool doQueryRWRRT(Q target, rw::trajectory::QPath &path);

    // RRT
    bool doQueryRRT(const Q start, const Q goal, trajectory::QPath& result);
    bool inCollision(const Q &q);
    bool inCollision(RRTNode<Q>* a, const Q &b);
    const Q chooseTarget(Q goal);
    RRTNode<Q>* nearestNeighbor(const RRTTree<Q>& tree, const Q& q);
    ExtendResult extend(RRTTree<Q>& tree, const Q& q, RRTNode<Q>* qNearNode);
    ExtendResult growTree(RRTTree<Q>& tree, const Q& q);
    void getPath(const RRTTree<Q>& startTree, const RRTTree<Q>& goalTree, trajectory::QPath& result);

    double _resolution = 0.01;
    double _epsilon = 0.1;
    float _pGoal = 0.1f;
    size_t _planIterator = 0;

    // ARRT
    bool doQueryARRT(const Q start, const Q goal, trajectory::QPath& result);
    double growTreeARRT(RRTTree<Q>& tree, const Q& goal);
    const Q chooseTargetARRT(const Q& start, const Q& goal);
    const Q extendARRT(RRTTree<Q>& tree, const Q& goal, const Q& qTarget, RRTNode<Q>* & parent);
    std::vector<RRTNode<Q>*> kNearestNeighbours(const Q& qTarget, size_t k, RRTTree<Q>& tree);
    //const Q generateExtenstion(RRTTree<Q>& tree, const Q& q);
    double getPathCost(trajectory::QPath& path);

    QElapsedTimer _elapsedTimer;
    RRTTree<Q>* _bestTree = nullptr;
    double _cost = DBL_MAX;
    double _costEpsilon = 0.01;
    double _distanceHeuristic = 0.5;
    double _distanceDelta = 0;
    double _costHeuristic = 0.5;
    double _costDelta = 0;
    size_t _maxAttempts = 50000;
    size_t _k = 4;
    const Q* _tempQ1;
    const Q* _tempQ2;



    std::random_device _rd;
    std::mt19937 _gen;

    models::WorkCell::Ptr _wc;
    kinematics::State::Ptr _state;
    models::Device::Ptr _device;

    proximity::CollisionDetector::Ptr _collisionDet;

    Metric<Q>::Ptr _metric;

    // The q constraint is to avoid collisions.
    rw::pathplanning::QConstraint::Ptr _constraint;

    pathplanning::QEdgeConstraint::Ptr _edgeconstraint;
    pathplanning::PlannerConstraint _pConstraint;

    proximity::CollisionDetector::Ptr _detector;

    pathplanning::QSampler::Ptr _sampler;
    pathplanning::QToQPlanner::Ptr _planner;
    // A sampler of collision free configurations for the device.
    //rw::pathplanning::QSampler::Ptr cfreeQ = QSampler::makeConstrained(QSampler::makeUniform(device), constraint);


};

#endif

