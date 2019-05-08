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

//Q_DECLARE_METATYPE(trajectory::QPath);

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
        _pathPlanner->_iterative = false;
        emit signalPlan(_device->getQ(_state), target, this->boxPlanSelect->currentIndex());
    });
    connect(this->btnInv, &QPushButton::released, [=](){
        ROS_INFO_STREAM("Requesting inverse kinematics");
        Vector3D<double> v = Vector3D<double>(boxX->value(), boxY->value(), boxZ->value());
        Rotation3D<double> rot = RPY<double>(boxRZ->value(), boxRY->value(), boxRX->value()).toRotation3D();
        Transform3D<double> t = Transform3D<double>(v, rot);
        std::vector<Q> s;
        inverseKinematics(_device, _state, t, s);
        if(!s.empty())
        {
            boxQ1->setValue(s.front()[0]);
            boxQ2->setValue(s.front()[1]);
            boxQ3->setValue(s.front()[2]);
            boxQ4->setValue(s.front()[3]);
            boxQ5->setValue(s.front()[4]);
            boxQ6->setValue(s.front()[5]);
        }
        //Transform3D<> tr = _device->baseTend(_state);
        //std::cout<<"Base To End = "<< tr.P()[0] << " " << tr.P()[1] << " " << tr.P()[2] << " " << std::endl;
        //Frame* fTCP = _wc->findFrame("TCP");_device->getEnd();
        //tr = _device->baseTframe(fTCP, _state);
        //std::cout<<"Base To TCP = "<< tr.P()[0] << " " << tr.P()[1] << " " << tr.P()[2] << " " << std::endl;

    });
    connect(this->btnReset, &QPushButton::released, [=](){
        boxQ1->setValue(1.57);
        boxQ2->setValue(-1.57);
        boxQ3->setValue(0);
        boxQ4->setValue(-1.57);
        boxQ5->setValue(-1.57);
        boxQ6->setValue(0);
        emit signalThreadTest();
    });

    // --------- QTROS
    _qtRos = new QtROS();

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
    //qRegisterMetaType<trajectory::QPath>();
    connect(_qtRos, SIGNAL(newState(rw::math::Q)), this, SLOT(newState(rw::math::Q)));

    // ----------- PLANNER
    //initialize path planner
    _pathPlanner = new Planner();
    // connect Planner
    connect(this, &RobotPlugin::signalPlan, _pathPlanner, &Planner::callPlan, Qt::ConnectionType::QueuedConnection);
    connect(_pathPlanner, &Planner::signalPlanChange, this, &RobotPlugin::planChange, Qt::ConnectionType::QueuedConnection);
    connect(this, &RobotPlugin::signalThreadTest, _pathPlanner, &Planner::threadTest, Qt::ConnectionType::QueuedConnection);

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

void RobotPlugin::planChange()
{
    _planChanged = true;    // a new plan is available at _planner._tmpPath
    ROS_INFO_STREAM("plan changing");
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
    _device->setBounds(Device::QBox(Q(6, 1.0, -2.0, -6.26, -6.26, -6.26, -6.26), Q(6, 2.5, 0, 6.26, 6.26, 6.26, 6.26)));

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
        _pathPlanner->_wc = _wc;
        _pathPlanner->_state = &_state;
        _pathPlanner->_device = _device;
        bool done = _pathPlanner->initRRT();
        ROS_INFO_STREAM("planner init done" << done);
        _pathPlanner->start();

        _qtRos->start();
        log().info() << "Start\n " << QApplication::instance()->thread()->currentThreadId() << "\n " << _qtRos->currentThreadId();

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
            if (boxIterative->isChecked() && _pathPlanner->_path.size() > 2)
            {
                rw::math::Q target(6, boxQ1->value(), boxQ2->value(), boxQ3->value(), boxQ4->value(), boxQ5->value(), boxQ6->value());
                _pathPlanner->MAX_TIME = 1500;                                  // 1.5s < 2*1s for servoing
                _pathPlanner->_iterative = true;
                emit signalPlan(_pathPlanner->_path.at(2), target, this->boxPlanSelect->currentIndex());   // assumption: previous planning is already finished by now
                // TODO: set max path cost of next iteration somehow
            }
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
        else if(dist < 0.05)
        {
            QApplication::processEvents();
            if(boxIterative->isChecked() && _pathPlanner->_path.size() > _pathIterator + 2)
            {
                if(_pathIterator != 0 && _pathIterator % 2 == 0)  // reaching the end of a second segment
                {
                    if(_planChanged)
                    {
                        _pathPlanner->_path = _pathPlanner->_tmpPath;
                        _pathIterator = 0;
                        _planChanged = false;
                        ROS_INFO_STREAM("PLAN CHANGED");
                    }
                    rw::math::Q target(6, boxQ1->value(), boxQ2->value(), boxQ3->value(), boxQ4->value(), boxQ5->value(), boxQ6->value());
                    _pathPlanner->MAX_TIME = 1500;                                  // 1.5s < 2*1s for servoing
                    _pathPlanner->_iterative = true;
                    emit signalPlan(_pathPlanner->_path.at(_pathIterator + 2), target, this->boxPlanSelect->currentIndex());   // assumption: previous planning is already finished by now
                    // TODO: set max path cost of next iteration somehow
                }
            }
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

void RobotPlugin::inverseKinematics(rw::common::Ptr<Device> device, const State &state, const Transform3D<> &target, std::vector<Q> &solutions)
{
    invkin::JacobianIKSolver solver(device, state);
    solver.setEnableInterpolation(true);
    solver.setClampToBounds(true);
    Transform3D<double> tcp = solver.getTCP()->fTf(device->getBase(), state);
    std::cout<<"TCP = x: "<< tcp.P()[0] << " y: " << tcp.P()[1] << " z: " << tcp.P()[2] << std::endl;
    solutions = solver.solve(target, state);
    if(solutions.empty()){
        ROS_INFO_STREAM("No IK solution found.");
    }
    else
    {
        for(auto it = solutions.begin(); it != solutions.end(); ++it) {
            if(!_pathPlanner->inCollision(*it.base())){
                std::cout<<"Solution = "<< *it.base()*180.0/M_PI <<std::endl;
            }
            else{
                solutions.erase(it--);
            }
        }
        if(solutions.empty()){
            ROS_INFO_STREAM("No collision free IK solution found.");
        }
    }
}

void RobotPlugin::stateChangedListener(const State& state)
{
    _state = state;
}

#if !RWS_USE_QT5
#include <QtCore/qplugin.h>
Q_EXPORT_PLUGIN(RobotPlugin);
#endif
