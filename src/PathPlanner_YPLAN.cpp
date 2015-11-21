// -*- C++ -*-
/*!
 * @file  PathPlanner_YPLAN.cpp
 * @brief Path Planner component using yplanner2d library
 * @date $Date$
 *
 * $Id$
 */

#include "PathPlanner_YPLAN.h"

// Module specification
// <rtc-template block="module_spec">
static const char* pathplanner_yplan_spec[] =
  {
    "implementation_id", "PathPlanner_YPLAN",
    "type_name",         "PathPlanner_YPLAN",
    "description",       "Path Planner component using yplanner2d library",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Navigation",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.RobotRadius", "0.35",
    "conf.default.maxSearchPathLength", "-1",
    "conf.default.pathDistanceTolerance", "0.5",
    "conf.default.goalHeadingTolerance", "0.5",
    "conf.default.goalDistanceTolerance", "0.5",
    "conf.default.pathHeadingTolerance", "0.5",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.RobotRadius", "text",
    "conf.__widget__.maxSearchPathLength", "text",
    "conf.__widget__.pathDistanceTolerance", "text",
    "conf.__widget__.goalHeadingTolerance", "text",
    "conf.__widget__.goalDistanceTolerance", "text",
    "conf.__widget__.pathHeadingTolerance", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
PathPlanner_YPLAN::PathPlanner_YPLAN(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_pathPlannerPort("pathPlanner")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
PathPlanner_YPLAN::~PathPlanner_YPLAN()
{
}



RTC::ReturnCode_t PathPlanner_YPLAN::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  m_pathPlannerPort.registerProvider("PathPlanner", "RTC::PathPlanner", m_pathPlanner);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_pathPlannerPort);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("RobotRadius", m_robotRadius, "0.35");
  bindParameter("maxSearchPathLength", m_maxSearchPathLength, "-1");
  bindParameter("pathDistanceTolerance", m_pathDistanceTolerance, "0.5");
  bindParameter("goalHeadingTolerance", m_goalHeadingTolerance, "0.5");
  bindParameter("goalDistanceTolerance", m_goalDistanceTolerance, "0.5");
  bindParameter("pathHeadingTolerance", m_pathHeadingTolerance, "0.5");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PathPlanner_YPLAN::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_YPLAN::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_YPLAN::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t PathPlanner_YPLAN::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t PathPlanner_YPLAN::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t PathPlanner_YPLAN::onExecute(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_YPLAN::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_YPLAN::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_YPLAN::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_YPLAN::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t PathPlanner_YPLAN::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void PathPlanner_YPLANInit(RTC::Manager* manager)
  {
    coil::Properties profile(pathplanner_yplan_spec);
    manager->registerFactory(profile,
                             RTC::Create<PathPlanner_YPLAN>,
                             RTC::Delete<PathPlanner_YPLAN>);
  }
  
};


