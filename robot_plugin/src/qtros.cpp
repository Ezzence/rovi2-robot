#include "qtros.h"
#include <math.h>
#include <caros_common_msgs/Q.h>
#include <caros/common_robwork.h>

#define SUBSCRIBER "/caros_universalrobot/caros_serial_device_service_interface/robot_state"

#define SERVO_TIME 2.f

QtROS::QtROS()
{

  ROS_INFO("Connected to roscore");

  // Subscribe to caros robot state
  _sub = _nh.subscribe(SUBSCRIBER, 1, &QtROS::stateCallback, this);

  _robot = new caros::SerialDeviceSIProxy(_nh, "caros_universalrobot");

  quitfromgui = false;
}

void QtROS::quitNow()
{
  quitfromgui = true;
}

void QtROS::stateCallback(const caros_control_msgs::RobotState & msg)
{
    // Extract configuration from RobotState message
    caros_common_msgs::Q conf = msg.q;

    // Convert from ROS msg to Robwork Q
    rw::math::Q conf_rw = caros::toRw(conf);

    // Emit the configuration
    emit newState(conf_rw);

}

void QtROS::moveHome()
{
    ROS_INFO("Called move home");
    rw::math::Q home = rw::math::Q(6, 0, -M_PI/2.0, 0, -M_PI/2.0, 0, 0);
    _robot->movePtp(home);
}

void QtROS::testPTP(double q1, double q2, double q3, double q4, double q5, double q6)
{
    ROS_INFO("Called robot moving test PTP");
    rw::math::Q testQ = rw::math::Q(6, q1, q2, q3, q4, q5, q6);
    _robot->movePtp(testQ);
}

void QtROS::testServo(rw::math::Q target, float time, float lookahead)
{
    ROS_INFO("Called robot moving test Servo");
    _robot->moveServoQ(target, time, lookahead, 300.f);
    this->wait((static_cast<unsigned long>(time)*1000));              // QThread::wait is in miliseconds
    _robot->moveServoStop();
}

void QtROS::moveServo(rw::math::Q target)
{
    _robot->moveServoQ(target, SERVO_TIME, 0.1f, 300.f);
}

void QtROS::updateServo(rw::math::Q target)
{
    _robot->moveServoUpdate(target);
}

void QtROS::stopServo()
{
    _robot->moveServoStop();
}

bool QtROS::movePathServo(rw::trajectory::QPath &path, rw::models::Device::Ptr device, rw::kinematics::State::Ptr state)
{
    _robot->moveServoQ(path.at(0), 2.f, 0.1f, 300.f);
    rw::math::Q currentPos = device->getQ(*state);
    for(size_t i = 0; i < path.size(); ++i)
    {
        _robot->moveServoQ(path.at(i), 0.5f, 0.1f, 300.f);
        while((currentPos - path.at(i)).norm2() > 0.05)
        {
            ROS_INFO_STREAM("distance: " << (currentPos - path.at(i)).norm2());
            rw::math::Q currentPos = device->getQ(*state);
        }
    }
    return true;
}





void QtROS::run()
{
    while(ros::ok() && !quitfromgui)
    {
        ros::spinOnce();
        QCoreApplication::processEvents();

        // Adjust the sleep to, according to how often you will check ROS for new messages
        ros::Duration(0.01).sleep();
    }
    if (!quitfromgui)
    {
        _robot->closePersistentConnections();
        emit rosQuits();
        ROS_INFO("ROS-Node Terminated\n");
    }
}
