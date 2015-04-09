#include "phototaxisbehavior.h"


/******************************************************************************/
/******************************************************************************/

CPhototaxisBehavior::CPhototaxisBehavior()
{
}

/******************************************************************************/
/******************************************************************************/

bool CPhototaxisBehavior::TakeControl()
{
    /* Get readings from light sensors */
    /* Sum them together */
    m_cLightVector.Set(0.0f, 0.0f);
    for(size_t i = 0; i <  m_sSensoryData.m_LightSensorData.size(); ++i)
    {
        m_cLightVector += CVector2(m_sSensoryData.m_LightSensorData[i].Value, m_sSensoryData.m_LightSensorData[i].Angle);
    }
    m_cLightVector /= m_sSensoryData.m_LightSensorData.size();


    /* If no light detected */
    if(m_cLightVector.Length() <= 0.0f)
        return false;
    else
    {
        std::cout << " Phototaxis behavior taking control " << std::endl;
        return true;
    }

}

/******************************************************************************/
/******************************************************************************/

void CPhototaxisBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    CVector2 m_cHeadingVector;
    m_cHeadingVector = m_sRobotData.MaxSpeed * CVector2(1.0f, m_cLightVector.Angle()) + m_sRobotData.MaxSpeed * CVector2(1.0f, 0.0f);

    WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);
}

/******************************************************************************/
/******************************************************************************/
