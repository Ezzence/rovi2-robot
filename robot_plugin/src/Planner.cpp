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


Planner::Planner(models::WorkCell::Ptr wc, kinematics::State::Ptr state, models::Device::Ptr device) :  _gen(_rd())
{
    _wc = wc;
    _state = state;
    _device = device;

    //qRegisterMetaType<Q>("Q");

    srand(static_cast<unsigned int>(QDateTime::currentMSecsSinceEpoch()));

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

bool Planner::initRRT()
{
    //_collisionDet = new proximity::CollisionDetector(wc.get(), rwlibs::proximitystrategies::ProximityStrategyYaobi::make());
    _collisionDet = new proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());

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

void Planner::debugTree(RRTTree<Q> &tree)
{
    BOOST_FOREACH(Node* node, tree.getNodes()) {
        ROS_INFO_STREAM(node->getValue()[0] << " " << node->getValue()[1] << " " << node->getValue()[2] << " " << node->getValue()[3] << " " << node->getValue()[4] << " " << node->getValue()[5]);
    }
}

void Planner::callPlan(Q start, Q target, int planSelect)
{
    //_path.clear();
    _planType = planSelect;
    _planIterator = 0;
    _closestDistance = DBL_MAX;
    _distanceHeuristic = 1.0;
    _costHeuristic = 0;
    _firstTime = INT_MAX;
    bool success = false;
    switch (planSelect)
    {
    case PlanSelect::RW_RRT:
    {
        success = doQueryRWRRT(start, target, _path);
        break;
    }
    case PlanSelect::RRT:
    {
        success = doQueryRRT(start, target, _path);
        break;
    }
    case PlanSelect::ARRT:
    {
        success = doQueryARRT(start, target, _path);
        break;
    }
    case PlanSelect::ARRTC:
        success = doQueryARRTC(start, target, _path);
        break;
    default:
    {
        ROS_WARN_STREAM("ERROR: planner not found!");
        break;
    }
    }
    ROS_INFO_STREAM("planner finished, success: " << success << " iteration: " << _planIterator);
}

double Planner::randQ()
{
    return double(rand())/RAND_MAX*12.52 - 6.26;
}

bool Planner::logCost(double cost)
{
    file.open("/home/ada/cost.txt", std::ios::out | std::ios::app);
    if (file.fail()){
        ROS_INFO_STREAM("FAILED TO WRITE FILE");
        return false;
    }
    //make sure write fails with exception if something is wrong
    file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);
    file << cost << std::endl;
    file.close();
    return true;
}

bool Planner::logTime(int time)
{
    file2.open("/home/ada/first.txt", std::ios::out | std::ios::app);
    if (file2.fail()){
        ROS_INFO_STREAM("FAILED TO WRITE FILE");
        return false;
    }
    //make sure write fails with exception if something is wrong
    file2.exceptions(file2.exceptions() | std::ios::failbit | std::ifstream::badbit);
    file2 << time << std::endl;
    file2.close();
    return true;
}

bool Planner::logIteration(size_t iter)
{
    file3.open("/home/ada/iter.txt", std::ios::out | std::ios::app);
    if (file3.fail()){
        ROS_INFO_STREAM("FAILED TO WRITE FILE");
        return false;
    }
    //make sure write fails with exception if something is wrong
    file3.exceptions(file3.exceptions() | std::ios::failbit | std::ifstream::badbit);
    file3 << iter << std::endl;
    file3.close();
    return true;
}

bool Planner::doQueryRWRRT(Q start, Q target, trajectory::QPath &path)
{
    Q pos = start;
    //rw::trajectory::Path<Q> path;
    path.clear();
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
            result.clear();
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

bool Planner::doQueryARRT(const Q start, const Q goal, trajectory::QPath &result)
{
    if (inCollision(start)) {
        std::cout<<"Start is in collision"<<std::endl;
        return false;
    }

    if (inCollision(goal)) {
        std::cout<<"Goal is in collision"<<std::endl;
        return false;
    }

    Tree* startTree;
    if(!_iterative){
        _cost = DBL_MAX;
    }
    _elapsedTimer.start();

    while (true)
    {
        ROS_INFO_STREAM("ARRT: doQuery");
        startTree = new Tree(start);
        double costed = growTreeARRT(*startTree, goal);

        if(costed != 0)
        {
            //ROS_INFO_STREAM("ARRT: DEBUG3");
            if(_bestTree != nullptr){
                delete _bestTree;
            }
            _bestTree = startTree;
            Path reverse;
            _bestTree->getRootPath(_bestTree->getLast(), reverse);
            if(!_iterative){
                ROS_INFO_STREAM("------- NON ITERATIVE -------------------------------------");
                result = Path();        // TODO: _path is empty here
                result.insert(result.end(), reverse.rbegin(), reverse.rend());  // path needs to be reversed
            }
            else
            {
                ROS_INFO_STREAM("------- ITERATIVE -------------------------------------");
                _tmpPath = Path();
                _tmpPath.insert(_tmpPath.end(), reverse.rbegin(), reverse.rend());  // path needs to be reversed
                emit signalPlanChange();
            }
            ROS_INFO_STREAM("tree: " << _bestTree->size() << " prev cost: " << _cost << " new: " << costed << "-------------");
            //debugPath(result);

            _cost = (1 - _costEpsilon)*costed;
            _distanceHeuristic = std::max(double(0), _distanceHeuristic - _distanceDelta);
            _costHeuristic = std::min(1.0, _costHeuristic + _costDelta);
        }
        else
        {
            if(_planIterator > 0){
                if(_log){
                    logCost(_cost); logTime(_firstTime); logIteration(_planIterator);
                }
                return true;
            }
            else{
                _debugTree = startTree;
                ROS_INFO_STREAM("NO PLAN!!! debug tree set");
                return false;
            }
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
}

double Planner::growTreeARRT(RRTTree<Q> &tree, const Q& goal)
{
    Q qNew = tree.getRoot().getValue();
    while(qNew != goal)
    {
        if(qNew != Q() && _closestDistance < 1.0){
            //ROS_INFO_STREAM("size: " << tree.size());
        }
        //ROS_INFO_STREAM("ARRT: grow");
        const Q qTarget = chooseTargetARRT(tree.getRoot().getValue(), goal);
        if (qTarget != Q()){
            Node* parent;
            qNew = extendARRT(tree, goal, qTarget, parent);
            if(qNew != Q()){
                tree.add(qNew, parent);
            }
        }
        if(_elapsedTimer.elapsed() > MAX_TIME && !(_log && _planIterator == 0)) // make sure we get at least one path if logging
        {
            return 0;
        }
        else if(_elapsedTimer.elapsed() > MAX_TIME + 5000)     // still abort if way too much time
        {
            return 0;
        }
    }

    // debug
    //_debugTree = &tree;
    //ROS_INFO_STREAM("debug tree set");
    // ----
    if(_firstTime == INT_MAX){
        _firstTime = int(_elapsedTimer.elapsed());
    }
    Path path;
    tree.getRootPath(tree.getLast(), path);
    return getPathCost(path, *_metric);
}

const Q Planner::chooseTargetARRT(const Q &start, const Q &goal)
{
    //ROS_INFO_STREAM("ARRT: choose");
    std::uniform_real_distribution<float> dis(0, 1.f);
    float p = dis(_gen);
    if(p < _pGoal)
    {
        return goal;
    }
    else
    {
        //Q qNew = _sampler->sample();
        Q qNew = Q(6, randQ(), randQ(), randQ(), randQ(), randQ(), randQ());
        size_t attempts = 0;
        while(true)
        {
            if(_metric->distance(start, qNew) + _metric->distance(qNew, goal) < _cost && !inCollision(qNew)){   // this way, the path cost is checked before collision (faster)
                break;
            }
            qNew = Q(6, randQ(), randQ(), randQ(), randQ(), randQ(), randQ());
            //ROS_INFO_STREAM("SAMPLE: " << qNew[0] << " " << qNew[1] << " " << qNew[2] << " " << qNew[3] << " " << qNew[4] << " " << qNew[5]);
            ++attempts;
            if(attempts > _maxAttempts){
                //ROS_WARN_STREAM("WARNING: no sample found within threshold");
                qNew = goal;         // in paper: return Q();
                attempts = 0;
                return Q();
            }
        }
        //ROS_INFO_STREAM("ARRT: SAMPLED");
        return qNew;
    }
}

const Q Planner::extendARRT(RRTTree<Q> &tree, const Q& goal, const Q &qTarget, Node* & parent)
{
    std::vector<Node*> QNear = kNearestNeighbours(qTarget, _k, tree);

    // sort QNear descending
    _tempQ1 = &qTarget; _tempQ2 = &tree.getRoot().getValue();
    std::sort(QNear.begin(), QNear.end(), [this](const Node* a, const Node* b){
       return _distanceHeuristic*_metric->distance(a->getValue(), *_tempQ1) + _costHeuristic*_metric->distance(*_tempQ2, a->getValue())
               > _distanceHeuristic*_metric->distance(b->getValue(), *_tempQ1) + _costHeuristic*_metric->distance(*_tempQ2, b->getValue());
    });

    while(!QNear.empty())
    {
        Node* qTree = QNear.back();

        const Q delta = qTarget - qTree->getValue();
        const double dist = _metric->distance(delta);

        const Q qNew = dist <= _epsilon ? qTarget : (qTree->getValue() + (_epsilon/dist)*delta);
        //ROS_INFO_STREAM("ARRT: VAL: " << qNew[0] << " " << qNew[1] << " " << qNew[2] << " " << qNew[3] << " " << qNew[4] << " " << qNew[5]);
        if (!inCollision(qTree, qNew))
        {
            Path path;
            tree.getRootPath(*qTree, path);
            double cost = getPathCost(path, *_metric) + _metric->distance(qTree->getValue(), qNew) + _metric->distance(qNew, goal);
            if(cost < _cost){
                parent = qTree;
                //ROS_INFO_STREAM("ARRT: EXTEND");
                return qNew;
            }
        }
        QNear.pop_back();
    }
    return Q();
}

std::vector<RRTNode<Q> *> Planner::kNearestNeighbours(const Q &qTarget, size_t k, RRTTree<Q> &tree)
{
    std::vector<Node*> minNodes;
    for(size_t i = 0; i < k; ++i)
    {
        double minLength = DBL_MAX;
        Node* minNode = NULL;

        BOOST_FOREACH(Node* node, tree.getNodes()) {
            const double length = _metric->distance(qTarget, node->getValue());
            if (length < minLength && std::find(minNodes.begin(), minNodes.end(), node) == minNodes.end())
            {
                minLength = length;
                minNode = node;
            }
        }
        if(minNode != NULL){
            minNodes.insert(minNodes.begin(), minNode);
        }
    }

    return minNodes;
}

double Planner::getPathCost(trajectory::QPath &path)
{
    double cost = 0;
    for(size_t i = 0; i < path.size() - 1; ++i)
    {
        cost += _metric->distance(path.at(i), path.at(i+1));
    }
    return cost;
}

//const Q Planner::generateExtenstion(RRTTree<Q> &tree, const Q &q)
//{
//
//}

double Planner::getPathCost(trajectory::QPath &path, rw::math::Metric<Q>& metric)
{
    double cost = 0;
    for(size_t i = 0; i < path.size() - 1; ++i)
    {
        cost += metric.distance(path.at(i), path.at(i+1));
    }
    return cost;
}

bool Planner::doQueryARRTC(const Q start, const Q goal, trajectory::QPath &result)
{
    if (inCollision(start)) {
        std::cout<<"Start is in collision"<<std::endl;
        return false;
    }

    if (inCollision(goal)) {
        std::cout<<"Goal is in collision"<<std::endl;
        return false;
    }

    if(!_iterative){
        _cost = DBL_MAX;
    }
    _elapsedTimer.start();

    while(true)
    {
        ROS_INFO_STREAM("ARRTC: doQuery");
        if(_startTree != nullptr){
            delete _startTree;
        }
        if(_goalTree != nullptr){
            delete _goalTree;
        }
        _startTree = new Tree(start);
        _goalTree = new Tree(goal);
        double costed = growTreeARRTC(*_startTree, *_goalTree, start, goal);

        if(costed != 0)
        {
            Path path;
            getPath(*_startTree, *_goalTree, path);
            if(!_iterative){
                ROS_INFO_STREAM("----- NON ITERATIVE ------------");
                result = Path();
                result.insert(result.end(), path.begin(), path.end());
                ROS_INFO_STREAM("size: " << result.size());
            }
            else
            {
                ROS_INFO_STREAM("----- ITERATIVE ----------------");
                _tmpPath = Path();
                _tmpPath.insert(_tmpPath.end(), path.begin(), path.end());
                emit signalPlanChange();
                ROS_INFO_STREAM("size: " << _tmpPath.size());
            }
            ROS_INFO_STREAM("tree: " << _startTree->size() + _goalTree->size() << " prev cost: " << _cost << " new: " << costed << "-------------");
            //debugPath(result);

            _cost = (1 - _costEpsilon)*costed;
            _distanceHeuristic = std::max(double(0), _distanceHeuristic - _distanceDelta);
            _costHeuristic = std::min(1.0, _costHeuristic + _costDelta);
        }
        else
        {
            if(_planIterator > 0){
                if(_log){
                    logCost(_cost); logTime(_firstTime); logIteration(_planIterator);
                }
                return true;
            }
            else{
                return false;
            }
        }

        ++_planIterator;
    }
}

double Planner::growTreeARRTC(RRTTree<Q> &startTree, RRTTree<Q> &goalTree, const Q &start, const Q &goal)
{
    Q qNew = Q();
    RRTTree<Q>* tree = nullptr;
    RRTTree<Q>* tree2 = nullptr;
    bool connected = false;
    while(!connected)
    {
        // randomly choose start or goal tree
        tree = nullptr;
        tree2 = nullptr;
        if(rand() % 2 == 0){
            tree = &startTree;
            tree2 = &goalTree;
        }else{
            tree = &goalTree;
            tree2 = &startTree;
        }
        const Q qTarget = chooseTargetARRT(tree->getRoot().getValue(), goal);
        if (qTarget != Q()){
            Node* parent;
            qNew = extendARRT(*tree, tree2->getRoot().getValue(), qTarget, parent);
            if(qNew != Q()){
                tree->add(qNew, parent);
                connected = connect(*tree2, *tree, qNew);
                //ROS_INFO_STREAM("ARRT: DEBUG");
            }
        }
        if(_elapsedTimer.elapsed() > MAX_TIME && !(_log && _planIterator == 0)) // make sure we get at least one path if logging
        {
            return 0;
        }
        else if(_elapsedTimer.elapsed() > MAX_TIME + 5000)     // still abort if way too much time
        {
            return 0;
        }
    }

    if(_firstTime == INT_MAX){
        _firstTime = int(_elapsedTimer.elapsed());
    }
    Path path1, path2;
    // TODO: investigate
    // tree 2 was appended last, so the Node for connecting to tree1 must be found
    tree->getRootPath(*nearestNeighbor(*tree, tree2->getLast().getValue()), path2);
    tree2->getRootPath(tree2->getLast(), path1);
    return getPathCost(path1) + getPathCost(path2);

}

bool Planner::connect(RRTTree<Q> &tree, RRTTree<Q> &otherTree, Q &target)
{
    Node* qNearNode = nearestNeighbor(tree, target);
    ExtendResult r = ExtendResult::Advanced;
    bool hasAdvanced = false;
    while(r == ExtendResult::Advanced)
    {
        r = extendARRTC(tree, otherTree, target, qNearNode);
        if(r == ExtendResult::Advanced){
            qNearNode = &tree.getLast();
        }
    }
    if(r == ExtendResult::Reached){
        return true;
    }else{
        return false;
    }

}

Planner::ExtendResult Planner::extendARRTC(RRTTree<Q> &tree, RRTTree<Q> &otherTree, const Q &q, RRTNode<Q> *qNearNode)
{
    const Q& qNear = qNearNode->getValue();
    const Q delta = q - qNear;
    const double dist = _metric->distance(delta);
    //ROS_INFO_STREAM("TWO: " << dist);

    if (dist <= _epsilon) {
        if (!inCollision(qNearNode, q)) {
            Path path;
            tree.getRootPath(*qNearNode, path);
            double cost = getPathCost(path, *_metric) + _metric->distance(qNearNode->getValue(), q) + _metric->distance(q, otherTree.getRoot().getValue());
            if(cost < _cost){
                tree.add(q, qNearNode);
                return Reached;
            }
        }
    }
    else
    {
        const Q qNew = qNear + (_epsilon / dist) * delta;
        //ROS_INFO_STREAM(qNew.norm2());
        if (!inCollision(qNearNode, qNew)) {
            Path path;
            tree.getRootPath(*qNearNode, path);
            double cost = getPathCost(path, *_metric) + _metric->distance(qNearNode->getValue(), qNew) + _metric->distance(qNew, otherTree.getRoot().getValue());
            if(cost < _cost){
                tree.add(qNew, qNearNode);
                return Advanced;
            }
        }
    }
    return Trapped;
}
