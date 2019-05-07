#include "RobotPlugin.hpp"
#include <math.h>
 
#include <rws/RobWorkStudio.hpp>
#include <qtimer.h>
#include <QPushButton>
#include <rw/loaders.hpp>



#define deviceName "UR5"
#define workcellPath "/home/ada/workspace/ros_ws/src/rovi2-robot/robot_plugin/WorkCell/Scene.wc.xml"

using namespace rw::common;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::sensor;

using namespace rws;


RobotPlugin::RobotPlugin():
    RobWorkStudioPlugin("robot plugin", QIcon(":/pa_icon.png"))
{
        // Load ui file
	setupUi(this);


    // initialize ros to start without running roslaunch or rosrun
    char** argv = NULL;
    int argc = 0;
    ros::init(argc, argv,"robot_plugin");

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));
    _timer->start(500);

    // connect GUI elements
    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn2    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(this->btnPlanner, &QPushButton::released, [=](){
        rw::math::Q target(6, boxQ1->value(), boxQ2->value(), boxQ3->value(), boxQ4->value(), boxQ5->value(), boxQ6->value());
        _pathPlanner->MAX_TIME = boxMaxTime->value();
        emit signalPlan(target, this->boxPlanSelect->currentIndex());
    });

    _qtRos = new QtROS();

    // connect QtROS
    connect(this, &RobotPlugin::signalMoveServo, _qtRos, &QtROS::moveServo, Qt::ConnectionType::QueuedConnection);
    connect(this, &RobotPlugin::signalUpdateServo, _qtRos, &QtROS::updateServo, Qt::ConnectionType::QueuedConnection);
    connect(this, &RobotPlugin::signalStopServo, _qtRos, &QtROS::stopServo, Qt::ConnectionType::QueuedConnection);
    connect(this, &RobotPlugin::signalTestServo, _qtRos, &QtROS::testServo, Qt::ConnectionType::QueuedConnection);
    connect(this->btnPTP, &QPushButton::pressed, [=](){
        _qtRos->testPTP(boxQ1->value(), boxQ2->value(), boxQ3->value(), boxQ4->value(), boxQ5->value(), boxQ6->value());

    });
    connect(this->btnServo, &QPushButton::released, [=](){
        rw::math::Q target(6, boxQ1->value(), boxQ2->value(), boxQ3->value(), boxQ4->value(), boxQ5->value(), boxQ6->value());
        emit signalTestServo(target, float(boxTime->value()), float(boxLookahead->value()));

    });

    connect(this->btnExecute, &QPushButton::released, [=](){
        ROS_INFO_STREAM("Executing plan");
        _movingStart = true;
        _movingServo = true;
        //bool done = _qtRos->movePathServo(_pathPlanner->_path, _device, &_state);
    });

    // Connect signal for quit
    connect(this, SIGNAL(quitNow()), _qtRos, SLOT(quitNow()));

    // Connect signal for moving robot home
    connect(this, SIGNAL(moveHome()), _qtRos, SLOT(moveHome()));

    // We need to register the type
    qRegisterMetaType<rw::math::Q>("rw::math::Q");
    connect(_qtRos, SIGNAL(newState(rw::math::Q)), this, SLOT(newState(rw::math::Q)));

}

RobotPlugin::~RobotPlugin()
{
}


void RobotPlugin::newState(rw::math::Q pos)
{
    // Slot actived each time a new message is received from ros
    _device->setQ(pos, _state);
    getRobWorkStudio()->setState(_state);

}


void RobotPlugin::initialize()
{
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&RobotPlugin::stateChangedListener, this, _1), this);

    // Auto load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(workcellPath);
    getRobWorkStudio()->setWorkCell(wc);

}

void RobotPlugin::open(WorkCell* workcell)
{
    // Default initialization
    _wc = workcell;
    _state = _wc->getDefaultState();
    _device = _wc->findDevice(deviceName);

}


void RobotPlugin::close()
{
     _wc = NULL;
}


void RobotPlugin::btnPressed()
{
    // Chech for btns
    QObject *obj = sender();
    if(obj==_btn0)
    {
        //initialize path planner
        _pathPlanner = new Planner(_wc, &_state, _device, this);
        // connect Planner
        connect(this, &RobotPlugin::signalPlan, _pathPlanner, &Planner::callPlan, Qt::ConnectionType::QueuedConnection);
        bool done = _pathPlanner->initRRT();
        ROS_INFO_STREAM("planner init done" << done);
        //_pathPlanner->start();

        log().info() << "Start\n";
        _qtRos->start();

    }
    else if(obj==_btn1)
    {
        log().info() << "Quit\n";
        emit quitNow();
    }
    else if(obj == _btn2)
    {
        log().info() << "Moving home\n";
        emit moveHome();
    }
}

void RobotPlugin::timer()
{
    if(_movingServo)
    {
        rw::math::Q currentPos = _device->getQ(_state);
        double dist = (currentPos - _pathPlanner->_path.at(_pathIterator)).norm2();

        // start servo movement first
        if(_movingStart)
        {
            ROS_INFO_STREAM("MOVING SERVO!");
            emit signalMoveServo(_pathPlanner->_path.at(_pathIterator));
            _movingStart = false;
        }
        // stop and reset iterator at end
        else if(_pathIterator >= (_pathPlanner->_path.size() - 1))
        {
            if(dist < 0.001)
            {
                ROS_INFO_STREAM("STOPPING SERVO!");
                emit signalStopServo();
                _pathIterator = 0;
                _movingServo = false;
            }
        }
        // move to next Q
        else if(dist < 0.1)
        {
            _pathIterator++;
            emit signalUpdateServo(_pathPlanner->_path.at(_pathIterator));
        }
        // keep moving towards current Q
        else
        {
            emit signalUpdateServo(_pathPlanner->_path.at(_pathIterator));
        }
        ROS_INFO_STREAM("distance: " << dist);
    }

    //_timer->stop();
}

void RobotPlugin::stateChangedListener(const State& state)
{
    _state = state;
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(RobotPlugin);
#endif
