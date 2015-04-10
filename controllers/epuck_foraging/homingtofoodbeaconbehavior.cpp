#include "homingtofoodbeaconbehavior.h"


/******************************************************************************/
/******************************************************************************/

CHomingToFoodBeaconBehavior::CHomingToFoodBeaconBehavior()
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CHomingToFoodBeaconBehavior::TakeControl()
{
    /* Get readings from RAB sensor */
    /* Set heading to beacon with smallest range */

    bool controltaken(false);

    Real closestBeaconRange = 10000.0f;  CRadians closestBeaconBearing; /*Range of 10000.0cm will never be exceeded */
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        //BEACON_ESTABLISHED = 3
        if(m_sSensoryData.m_RABSensorData[i].Data[0] == 3)
        {
            controltaken = true;

            if(m_sSensoryData.m_RABSensorData[i].Range < closestBeaconRange)
            {
                closestBeaconRange   = m_sSensoryData.m_RABSensorData[i].Range;
                closestBeaconBearing = m_sSensoryData.m_RABSensorData[i].HorizontalBearing;
            }
        }
    }

    if(controltaken)
    {
        m_cHomingVector = CVector2(closestBeaconRange, closestBeaconBearing); // range is in cm, but since we are going to normalise the vector it does not matter
        std::cout << " HomingToFoodBeacon Behavior taking control " << std::endl;
    }

    return controltaken;
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CHomingToFoodBeaconBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
     CVector2 m_cHeadingVector = m_sRobotData.MaxSpeed * m_cHomingVector.Normalize() +  m_sRobotData.MaxSpeed * CVector2(1.0f, 0.0f);

     WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);
}

/******************************************************************************/
/******************************************************************************/
