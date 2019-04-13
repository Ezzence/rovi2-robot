
#pragma once

#include "qtros.h"
#include <ros/ros.h>
#include <rw/models.hpp>
#include <rw/math/Q.hpp>


class Planner
{
public:
    Planner(rw::models::WorkCell::Ptr wc, rw::kinematics::State::Ptr state, rw::models::Device::Ptr device);
    virtual ~Planner();



private slots:


signals:


private:


    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State::Ptr _state;
    rw::models::Device::Ptr _device;


};

#endif /*RINGONHOOKPLUGIN_HPP_*/
