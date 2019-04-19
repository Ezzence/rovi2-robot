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

    // now connect stuff from the ui component
    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn2    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(this->btnPTP, &QPushButton::pressed, [=](){
        _qtRos->testPTP(boxQ1->value(), boxQ2->value(), boxQ3->value(), boxQ4->value(), boxQ5->value(), boxQ6->value());

    });
    connect(this->btnServo, &QPushButton::pressed, [=](){
        _qtRos->testServo(boxQ1->value(), boxQ2->value(), boxQ3->value(), boxQ4->value(), boxQ5->value(), boxQ6->value(), boxTime->value(), boxLookahead->value());

    });
    connect(this->btnPlanner, &QPushButton::pressed, [=](){
        _pathPlanner = new Planner(_wc, &_state, _device);
        bool done = _pathPlanner->initRRT();
        ROS_INFO_STREAM("planner init done" << done);
        rw::math::Q target(6, boxQ1->value(), boxQ2->value(), boxQ3->value(), boxQ4->value(), boxQ5->value(), boxQ6->value());
        done = _pathPlanner->plan(target);
        ROS_INFO_STREAM("Path generation success: " << done);
    });
    connect(this->btnExecute, &QPushButton::pressed, [=](){
        ROS_INFO_STREAM("Executing plan");
        bool done = _qtRos->movePathServo(_pathPlanner->_path, _device, &_state);
        ROS_INFO_STREAM("Plan execution: " << done);
    });

    _qtRos = new QtROS();

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
    _timer->stop();
}

void RobotPlugin::stateChangedListener(const State& state)
{
    _state = state;
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(RobotPlugin);
#endif
