
#ifndef PLANNER_H
#define PLANNER_H

#include <QThread>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QDateTime>
#include <cstdlib>
#include <random>

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

class Planner : public QObject
{
    Q_OBJECT
public:
    Planner(){

    }
    // weird bug with robwork signature _ZTV7Planner was because this class was not added into the cmake with the QT_WRAPPER_CPP
    Planner(rw::models::WorkCell::Ptr wc, rw::kinematics::State::Ptr state, rw::models::Device::Ptr device);
    ~Planner()
    {
        //qRegisterMetaType<Q>("Q");

        srand(static_cast<unsigned int>(QDateTime::currentMSecsSinceEpoch()));
    }

    void run();

    bool initRRT();

    void debugPath(rw::trajectory::QPath &path);
    void debugTree(RRTTree<Q>& tree);

    enum ExtendResult { Trapped, Reached, Advanced };
    enum PlanSelect { RW_RRT = 0, RRT = 1, ARRT = 2, ARRT2 = 3, ARRTC = 4};
    int _planType = 999;        ///< (not valid plan type at start)
    bool _iterative = false;    ///< should be true if a plan is already being executed on the robot

    trajectory::QPath _path;
    trajectory::QPath _tmpPath;

    models::WorkCell::Ptr _wc;
    kinematics::State::Ptr _state;
    models::Device::Ptr _device;
    Metric<Q>::Ptr _metric;

    // Important Experiment Parameters
    // ARRT
    int MAX_TIME = 0;
    double _cost = DBL_MAX;


public slots:

    void callPlan(Q start, Q target, int planSelect);
    bool inCollision(const Q &q);

    static double getPathCost(trajectory::QPath& path, rw::math::Metric<Q>& metric);

signals:

    void signalPlanChange();

private:

    double randQ();

    // default robwork RRT
    bool doQueryRWRRT(Q start, Q target, trajectory::QPath &path);

    // RRT
    bool doQueryRRT(const Q start, const Q goal, trajectory::QPath& result);
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

    QElapsedTimer _elapsedTimer;
    RRTTree<Q>* _bestTree = nullptr;
    double _costEpsilon = 0.01;
    double _distanceHeuristic = 0.5;
    double _distanceDelta = 0;
    double _costHeuristic = 0.5;
    double _costDelta = 0;
    size_t _maxAttempts = 1000;
    size_t _k = 4;
    const Q* _tempQ1;
    const Q* _tempQ2;



    std::random_device _rd;
    std::mt19937 _gen;

    proximity::CollisionDetector::Ptr _collisionDet;

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

