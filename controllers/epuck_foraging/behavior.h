#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include <vector>
//#include "common.h"
//#include "epuck_foraging.h"

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/range.h>


#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_updated_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>

/******************************************************************************/
/******************************************************************************/
class CBehavior;

typedef std::vector<CBehavior*>           TBehaviorVector;
typedef std::vector<CBehavior*>::iterator TBehaviorVectorIterator;


/******************************************************************************/
/******************************************************************************/

//class CEPuckForaging;
using namespace argos;

/******************************************************************************/
/******************************************************************************/

class CBehavior
{
public:
    CBehavior();
    virtual ~CBehavior();

    virtual bool TakeControl() = 0;
    virtual void Suppress();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    struct RobotData
    {
        Real MaxSpeed;
        Real ticks_per_second;
        Real INTERWHEEL_DISTANCE, HALF_INTERWHEEL_DISTANCE;
        Real WHEEL_RADIUS;
        Real seconds_per_tick;
    };

    struct SensoryData
    {
       CRandom::CRNG* m_pcRNG;

       CCI_EPuckProximitySensor::TReadings m_ProximitySensorData;
       CCI_LightUpdatedSensor::TReadings m_LightSensorData;
       CCI_GroundSensor::TReadings m_GroundSensorData;

       void SetSensoryData(CRandom::CRNG* rng, CCI_EPuckProximitySensor::TReadings proximity, CCI_LightUpdatedSensor::TReadings light, CCI_GroundSensor::TReadings ground)
       {
           m_pcRNG = rng;
           m_ProximitySensorData = proximity;
           m_LightSensorData = light;
           m_GroundSensorData = ground;
       }
    };

    static SensoryData m_sSensoryData;
    static RobotData m_sRobotData;
    //virtual void SetAgent(CEPuckForaging *pc_agent);

protected:
    //CEPuckForaging*   m_pcEPuck;
};

/******************************************************************************/
/******************************************************************************/

#endif

/******************************************************************************/
/******************************************************************************/
