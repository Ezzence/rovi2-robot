#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP


#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition

#include "ui_RobotPlugin.h"
#include "qtros.h"
#include "Planner.h"
#include <ros/ros.h>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/models.hpp>
#include <rw/math/Q.hpp>
#include <rw/models/Device.hpp>


// Initialize plugin using QT5
class RobotPlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
    Q_OBJECT
    Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
    Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
#endif
public:
    RobotPlugin();
    virtual ~RobotPlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void btnPressed();
    void timer();

    void stateChangedListener(const rw::kinematics::State& state);
    void newState(rw::math::Q pos);

signals:
    void quitNow();
    void moveHome();

    void signalPlan(Q target, int planSelect);
    void signalMoveServo(Q target);
    void signalUpdateServo(Q target);
    void signalStopServo();

private:

    QTimer* _timer;
    QtROS *_qtRos;
    Planner* _pathPlanner = nullptr;

    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rw::models::Device::Ptr _device;

    bool _movingServo = false;
    bool _movingStart = false;
    size_t _pathIterator = 0;



};

#endif /*RINGONHOOKPLUGIN_HPP_*/
