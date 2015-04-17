#ifndef PROPRIOCEPTIVE_FEATUREVECTOR_H_
#define PROPRIOCEPTIVE_FEATUREVECTOR_H_


#define MODELSTARTTIME 450.0 // used for your sliding window

/******************************************************************************/
/******************************************************************************/

#include <string>
/******************************************************************************/
/******************************************************************************/
#include <argos3/core/utility/math/vector2.h>

#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_updated_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CProprioceptiveFeatureVector
{
public:

    struct RobotData
    {
        Real     MaxLinearSpeed;
        Real     MaxAngularSpeed;

        Real     MaxLinearAcceleration;
        Real     MaxAngularAcceleration;

        Real     iterations_per_second, seconds_per_iterations;
        Real     INTERWHEEL_DISTANCE, HALF_INTERWHEEL_DISTANCE;
        Real     WHEEL_RADIUS;
    };

    struct SensoryData
    {
       Real m_rTime;
       Real f_LeftWheelSpeed, f_RightWheelSpeed;
       CCI_EPuckProximitySensor::TReadings m_ProximitySensorData;
       CCI_LightUpdatedSensor::TReadings m_LightSensorData;
       CCI_GroundSensor::TReadings m_GroundSensorData;
       CCI_RangeAndBearingSensor::TReadings  m_RABSensorData;

       Real LinearSpeed, AngularSpeed, LinearAcceleration, AngularAcceleration;
       CVector2 pos; CRadians orientation;

       SensoryData()
       {
           m_rTime = 0.0f;
           f_LeftWheelSpeed = 0.0f; f_RightWheelSpeed = 0.0f;
           LinearSpeed = 0.0f; AngularSpeed = 0.0f; LinearAcceleration = 0.0f; AngularAcceleration = 0.0f;
           pos = CVector2(0.0, 0.0);
           orientation.SetValue(0.0f);
       }

       void SetSensoryData(Real time, CCI_EPuckProximitySensor::TReadings proximity, CCI_LightUpdatedSensor::TReadings light, CCI_GroundSensor::TReadings ground,
                           CCI_RangeAndBearingSensor::TReadings  rab, Real LeftWheelSpeed, Real RightWheelSpeed)
       {
           m_rTime = time;
           m_ProximitySensorData = proximity;
           m_LightSensorData = light;
           m_GroundSensorData = ground;
           m_RABSensorData = rab;
           f_LeftWheelSpeed = LeftWheelSpeed; f_RightWheelSpeed = RightWheelSpeed;

           EstimateCurrentSpeedAndAcceleration();
           EstimateCurrentPosition();
       }

       void SetSensoryData(Real time, CCI_EPuckProximitySensor::TReadings proximity, CCI_RangeAndBearingSensor::TReadings  rab, Real LeftWheelSpeed, Real RightWheelSpeed)
       {
           m_rTime = time;
           m_ProximitySensorData = proximity;
           m_RABSensorData = rab;
           f_LeftWheelSpeed = LeftWheelSpeed; f_RightWheelSpeed = RightWheelSpeed;

           EstimateCurrentSpeedAndAcceleration();
           EstimateCurrentPosition();
       }

       void EstimateCurrentSpeedAndAcceleration()
       {
           Real prev_LinearSpeed = LinearSpeed;
           LinearSpeed = (f_LeftWheelSpeed + f_RightWheelSpeed) / 2.0f;
           LinearAcceleration = LinearSpeed - prev_LinearSpeed;

           Real prev_AngularSpeed = AngularSpeed;
           AngularSpeed = (-f_LeftWheelSpeed + f_RightWheelSpeed) / (m_sRobotData.INTERWHEEL_DISTANCE*100.0f);
           AngularAcceleration = AngularSpeed - prev_AngularSpeed;
       }

       void EstimateCurrentPosition()
       {
           CVector2 prev_pos         = pos;
           CRadians prev_orientation = orientation;

           orientation = prev_orientation + CRadians(m_sRobotData.seconds_per_iterations * ((-f_LeftWheelSpeed + f_RightWheelSpeed) / (m_sRobotData.INTERWHEEL_DISTANCE*100.0f)));
           Real rX = prev_pos.GetX() + m_sRobotData.seconds_per_iterations * ((f_LeftWheelSpeed + f_RightWheelSpeed) / 2.0f) * Cos(orientation);
           Real rY = prev_pos.GetY() + m_sRobotData.seconds_per_iterations * ((f_LeftWheelSpeed + f_RightWheelSpeed) / 2.0f) * Sin(orientation);
           pos.Set(rX, rY);
       }
    };


    CProprioceptiveFeatureVector();
    virtual ~CProprioceptiveFeatureVector();

    static unsigned int NUMBER_OF_FEATURES;
    static unsigned int MAX_NUMBER_OF_FEATURES;
    static unsigned int NUMBER_OF_FEATURE_VECTORS;
    static double       FEATURE_RANGE;

    virtual unsigned int GetValue() const;
    virtual unsigned int GetLength() const;

    void PrintFeatureDetails();

    virtual unsigned int SimulationStep();

    //virtual std::string ToString();

    static RobotData m_sRobotData;
    SensoryData m_sSensoryData;

protected:
    virtual void ComputeFeatureValues();

    virtual unsigned CountNeighbors(Real sensor_range);

    unsigned  m_unValue;
    unsigned  m_unLength;

    Real*         m_pfFeatureValues;
    Real*         m_pfAllFeatureValues;

    int*           m_piLastOccuranceEvent;
    int*           m_piLastOccuranceNegEvent;

    int          m_iEventSelectionTimeWindow;


    Real       m_fVelocityThreshold;
    Real       m_fAccelerationThreshold;

    Real       m_tAngularVelocityThreshold;
    Real       m_tAngularAccelerationThreshold;



    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    unsigned int m_unNbrsCurrQueueIndex;

    unsigned int m_unSumTimeStepsNbrsRange0to30;
    unsigned int m_unSumTimeStepsNbrsRange30to60;

    unsigned int* m_punNbrsRange0to30AtTimeStep;
    unsigned int* m_punNbrsRange30to60AtTimeStep;



    // keeping track of distance travelled by bot in last 100 time-steps
    int              m_iDistTravelledTimeWindow;

    unsigned int     m_unCoordCurrQueueIndex;

    Real           m_fSquaredDistTravelled;
    Real           m_fSquaredDistThreshold;

    argos::CVector2  *m_pvecCoordAtTimeStep;


};

/******************************************************************************/
/******************************************************************************/


#endif
