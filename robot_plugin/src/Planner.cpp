#include "Planner.h"

#include <math.h>
#include <qtimer.h>

#include <boost/foreach.hpp>

#include <rw/loaders.hpp>


using namespace rw;
using namespace rw::math;

typedef rwlibs::pathplanners::RRTNode<rw::math::Q> Node;
typedef rwlibs::pathplanners::RRTTree<rw::math::Q> Tree;
typedef rw::trajectory::QPath Path;
typedef Planner::ExtendResult ExtendResult;


Planner::Planner(models::WorkCell::Ptr wc, kinematics::State::Ptr state, models::Device::Ptr device, QObject *parent) : QThread(parent), _gen(_rd())
{
    _wc = wc;
    _state = state;
    _device = device;

    //_collisionDet = new proximity::CollisionDetector(wc.get(), rwlibs::proximitystrategies::ProximityStrategyYaobi::make());
    _collisionDet = new proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    qRegisterMetaType<Q>("Q");

    // TEST PATH!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//    _path.push_back(Q(6, 0, -1.5, 0, -1.5, 0, 0));
//    _path.push_back(Q(6, 0, -1.5, 0, -1.5, 0.3, 0));
//    _path.push_back(Q(6, 0, -1.5, 0, -1.5, 0.6, 0));
//    _path.push_back(Q(6, 0, -1.5, 0, -1.5, 0.3, 0));
//    _path.push_back(Q(6, 0, -1.5, 0, -1.5, 0.0, 0));
//    _path.push_back(Q(6, 0, -1.5, 0, -1.5, -0.3, 0));
//    _path.push_back(Q(6, 0, -1.5, 0, -1.5, -0.6, 0));
//    _path.push_back(Q(6, 0, -1.5, 0, -1.5, -0.3, 0));
//    _path.push_back(Q(6, 0, -1.5, 0, -1.5, 0, 0));
    // -----------------------------------------------

}

void Planner::run()
{
    while(true)
    {
        QCoreApplication::processEvents();
    }
}

bool Planner::initRRT()
{
    _constraint = pathplanning::QConstraint::make(_collisionDet, _device, *_state);
    //_detector = new proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

    _metric = MetricFactory::makeEuclidean<Q>();

    // the edge constraint tests the constraint on edges, eg. edge between two configurations
    _edgeconstraint = pathplanning::QEdgeConstraint::make(pathplanning::QConstraint::make(_collisionDet, _device, *_state), _metric, _resolution);
    //_edgeconstraint = rw::pathplanning::QEdgeConstraintIncremental::makeDefault(_constraint, _device);

    _pConstraint = pathplanning::PlannerConstraint::make(_constraint, _edgeconstraint);

    // A sampler of collision free configurations for the device.
    _sampler = pathplanning::QSampler::makeConstrained(pathplanning::QSampler::makeUniform(_device), _constraint);

    // An RRTConnect based point-to-point path planner.
    _planner = rwlibs::pathplanners::RRTQToQPlanner::makeConnect(_pConstraint, _sampler, _metric, 0.1);


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

void Planner::callPlan(Q target, int planSelect)
{
    _path.clear();
    bool success = false;
    switch (planSelect)
    {
    case PlanSelect::RW_RRT:
    {
        success = doQueryRWRRT(target, _path);
        break;
    }
    case PlanSelect::RRT:
    {
        success = doQueryRRT(_device->getQ(*_state), target, _path);
        break;
    }
    default:
    {
        ROS_WARN_STREAM("ERROR: planner not found!");
        break;
    }
    }
    ROS_INFO_STREAM("planner finished, success: " << success);
}

bool Planner::doQueryRWRRT(Q target, trajectory::QPath &path)
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
    //const std::vector<kinematics::State> states = models::Models::getStatePath(*_device, path, *_state);

    // Write the sequence of states to a file.
    //loaders::PathLoader::storeVelocityTimedStatePath(*_wc, states, "rrt-path-planning.rwplay");

    return true;
}

bool Planner::doQueryRRT(const Q start, const Q goal, Path& result)
{
    if (inCollision(start)) {
        std::cout<<"Start is in collision"<<std::endl;
        return false;
    }

    if (inCollision(goal)) {
        std::cout<<"Goal is in collision"<<std::endl;
        return false;
    }

    // removed simple line connection

    Tree startTree(start);
    Tree goalTree(goal);

    while (true) {  // TODO: termination condition
        const Q qAttr = chooseTarget(goal);
        if (qAttr.empty()){
            RW_THROW("Sampler must always succeed.");
        }

        // If both trees manage to connect, then return the resulting
        // path.
        if (growTree(startTree, qAttr) == Reached && growTree(goalTree, qAttr) == Reached)
        {
            getPath(startTree, goalTree, result);
            debugPath(result);
            return true;
        }
        ++_planIterator;

        // DEBUG
        //ROS_INFO_STREAM("START");
        //BOOST_FOREACH(Node* node, goalTree.getNodes()) {
        //    ROS_INFO_STREAM(node->getValue()[0] << " " << node->getValue()[1] << " " << node->getValue()[2] << " " << node->getValue()[3] << " " << node->getValue()[4] << " " << node->getValue()[5]);
        //}
        //ROS_INFO_STREAM(qAttr[0] << " " << qAttr[1] << " " << qAttr[2] << " " << qAttr[3] << " " << qAttr[4] << " " << qAttr[5]);
       //ROS_INFO_STREAM(startTree.getNodes().second - startTree.getNodes().first);
        //ROS_INFO_STREAM("PLANNING ITERATOR " << planIterator);
    }

    return false;
}

bool Planner::inCollision(const Q& q)
{
    return _pConstraint.getQConstraint().inCollision(q);
}

// 'node' is known to be collision free, but 'b' is not.
bool Planner::inCollision(Node* a, const Q& b)
{
    return (_pConstraint.getQConstraint().inCollision(b) || _pConstraint.getQEdgeConstraint().inCollision(a->getValue(), b));
}

const Q Planner::chooseTarget(const Q goal)
{
    std::uniform_real_distribution<float> dis(0, 1.f);
    float p = dis(_gen);
    if(p < _pGoal)
    {
        return goal;
    }
    else
    {
        return _sampler->sample();
    }
}

Node* Planner::nearestNeighbor(const Tree& tree, const Q& q)
{
    double minLength = DBL_MAX;
    Node* minNode = NULL;

    BOOST_FOREACH(Node* node, tree.getNodes()) {
        const double length = _metric->distance(q, node->getValue());
        if (length < minLength) {
            minLength = length;
            minNode = node;
            //ROS_INFO_STREAM("ONE: " << length);
        }
    }

    RW_ASSERT(minNode);
    return minNode;
}

ExtendResult Planner::extend(Tree& tree, const Q& q, Node* qNearNode)
{
    //ROS_INFO_STREAM("ONE: " << qNearNode->getValue()[0] << " " << qNearNode->getValue()[1] << " " << qNearNode->getValue()[2] << " " << qNearNode->getValue()[3] << " " << qNearNode->getValue()[4] << " " << qNearNode->getValue()[5]);
    //ROS_INFO_STREAM("TWO: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5]);
    const Q& qNear = qNearNode->getValue();
    const Q delta = q - qNear;
    const double dist = _metric->distance(delta);
    //ROS_INFO_STREAM("TWO: " << dist);

    if (dist <= _epsilon) {
        if (!inCollision(qNearNode, q)) {
            tree.add(q, qNearNode);
            return Reached;
        } else {
            return Trapped;
        }
    } else {
        const Q qNew = qNear + (_epsilon / dist) * delta;
        //ROS_INFO_STREAM(qNew.norm2());
        if (!inCollision(qNearNode, qNew)) {
            tree.add(qNew, qNearNode);
            return Advanced;
        } else {
            return Trapped;
        }
    }
}

ExtendResult Planner::growTree(Tree& tree, const Q& q)
{
    Node* qNearNode = nearestNeighbor(tree, q);
    ExtendResult extendResult = extend(tree, q, qNearNode);
    //ROS_INFO_STREAM(extendResult);
    return extendResult;
}

void Planner::getPath(const Tree& startTree, const Tree& goalTree, Path& result)
{
    Path revPart;
    Tree::getRootPath(*startTree.getLast().getParent(), revPart);
    result.insert(result.end(), revPart.rbegin(), revPart.rend());
    Tree::getRootPath(goalTree.getLast(), result);
}
