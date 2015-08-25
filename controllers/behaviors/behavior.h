#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include <vector>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/vector2.h>


#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_updated_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

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

#define NO_TURN 0
#define SOFT_TURN 1
#define HARD_TURN 2

/******************************************************************************/
/******************************************************************************/

class CBehavior
{
public:
    CBehavior();
    virtual ~CBehavior();

    virtual void SimulationStep() = 0;

    virtual bool TakeControl() = 0;
    virtual void Suppress();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    virtual void PrintBehaviorIdentity();

    virtual void WheelSpeedsFromHeadingVector(CVector2 &m_cHeadingVector, Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    struct RobotData
    {
        Real     MaxSpeed;
        Real     iterations_per_second;
        Real     INTERWHEEL_DISTANCE, HALF_INTERWHEEL_DISTANCE;
        Real     WHEEL_RADIUS;
        Real     seconds_per_iterations;
        CRadians m_cNoTurnOnAngleThreshold;
        CRadians m_cSoftTurnOnAngleThreshold;


        size_t   BEACON_SIGNAL_MARKER;

        size_t   SELF_INFO_PACKET_MARKER;
        size_t   SELF_INFO_PACKET_FOOTER_MARKER;

        size_t   RELAY_FVS_PACKET_MARKER;
        size_t   RELAY_FVS_PACKET_FOOTER_MARKER;

        size_t   VOTER_PACKET_MARKER;
        size_t   VOTER_PACKET_FOOTER_MARKER;

        size_t   DATA_BYTE_BOUND_MARKER;

        size_t   OBSERVATION_MODE_TYPE;
    };

    struct SensoryData
    {
       CRandom::CRNG* m_pcRNG;

       Real m_rTime;

       CCI_EPuckProximitySensor::TReadings m_ProximitySensorData;
       CCI_LightUpdatedSensor::TReadings m_LightSensorData;
       CCI_GroundSensor::TReadings m_GroundSensorData;
       CCI_RangeAndBearingSensor::TReadings  m_RABSensorData;

       Real f_LeftWheelSpeed, f_RightWheelSpeed;

       void SetSensoryData(CRandom::CRNG* rng, Real Time, CCI_EPuckProximitySensor::TReadings proximity, CCI_LightUpdatedSensor::TReadings light, CCI_GroundSensor::TReadings ground,
                           CCI_RangeAndBearingSensor::TReadings  rab)
       {
           m_pcRNG = rng;
           m_rTime = Time;
           m_ProximitySensorData = proximity;
           m_LightSensorData = light;
           m_GroundSensorData = ground;
           m_RABSensorData = rab;
       }

       void SetSensoryData(CRandom::CRNG* rng, Real Time, CCI_EPuckProximitySensor::TReadings proximity, CCI_RangeAndBearingSensor::TReadings  rab)
       {
           m_pcRNG = rng;
           m_rTime = Time;
           m_ProximitySensorData = proximity;
           m_RABSensorData = rab;
       }

       void SetWheelSpeedsFromEncoders(Real LeftWheelSpeed, Real RightWheelSpeed)
       {
           f_LeftWheelSpeed = LeftWheelSpeed; f_RightWheelSpeed = RightWheelSpeed;
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
