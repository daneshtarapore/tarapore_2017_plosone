#include "antiphototaxisbehavior.h"


/******************************************************************************/
/******************************************************************************/

CAntiPhototaxisBehavior::CAntiPhototaxisBehavior()
{
}

/******************************************************************************/
/******************************************************************************/

bool CAntiPhototaxisBehavior::TakeControl()
{
    /* Get readings from light sensors */
    /* Sum them together */
    m_cLightVector.Set(0.0f, 0.0f);
    for(size_t i = 0; i <  m_sSensoryData.m_LightSensorData.size(); ++i)
    {
        m_cLightVector += CVector2(m_sSensoryData.m_LightSensorData[i].Value, m_sSensoryData.m_LightSensorData[i].Angle);
    }
    m_cLightVector /= m_sSensoryData.m_LightSensorData.size();

    // light detected and in nest
    if(m_cLightVector.Length() > 0.0f &&
       m_sSensoryData.m_GroundSensorData[0] > 0.25f && m_sSensoryData.m_GroundSensorData[0] < 0.75f &&
       m_sSensoryData.m_GroundSensorData[1] > 0.25f && m_sSensoryData.m_GroundSensorData[1] < 0.75f &&
       m_sSensoryData.m_GroundSensorData[2] > 0.25f && m_sSensoryData.m_GroundSensorData[2] < 0.75f)
    {
        //std::cout << " AntiPhototaxis behavior taking control " << std::endl;

        return true;
    }

    return false;
}

/******************************************************************************/
/******************************************************************************/

void CAntiPhototaxisBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    CVector2 m_cHeadingVector;
    m_cHeadingVector = -0.25f * m_sRobotData.MaxSpeed * CVector2(1.0f, m_cLightVector.Angle()) + 1.0f * m_sRobotData.MaxSpeed * CVector2(1.0f, 0.0f); // -0.25 as a too high magnitude anti-phototaxis vector confuses the robot very near the light source; All the sensors are highly active and no directional information can be gained.

    WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);
}

/******************************************************************************/
/******************************************************************************/
