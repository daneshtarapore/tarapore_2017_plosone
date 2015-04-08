#include "phototaxisbehavior.h"


/******************************************************************************/
/******************************************************************************/

CPhototaxisBehavior::CPhototaxisBehavior(Real LightSensorThreshold, CRange<CDegrees> GoStraightAngleRangeDegrees) :
    m_fLightSensorThreshold(LightSensorThreshold),
    m_cGoStraightAngleRangeDegrees(GoStraightAngleRangeDegrees)
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
    if(m_cLightVector.Length() <= m_fLightSensorThreshold)
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

    m_cHeadingVector = CVector2(1.0,m_cLightVector.Angle());


    if (m_cGoStraightAngleRangeDegrees.WithinMinBoundIncludedMaxBoundIncluded(ToDegrees(m_cHeadingVector.Angle().SignedNormalize())))
    {
        std::cout << " P Straight m_cHeadingVector.Angle().SignedNormalize() " << (ToDegrees(m_cHeadingVector.Angle().SignedNormalize())) << std::endl;
        fLeftWheelSpeed  = m_sRobotData.MaxSpeed;
        fRightWheelSpeed = m_sRobotData.MaxSpeed;
        return;
    }


    Real fSpeed;
    CRadians angle = m_cHeadingVector.Angle();

    //     std::cout << " Light homing behavior: m_cHeadingVector.Angle() " << m_cHeadingVector.Angle() << std::endl;
    angle = angle.SignedNormalize();
    //     std::cout << " Light homing behavior: angle.SignedNormalize() " << angle.SignedNormalize() << std::endl;

    //     std::cout << " Light homing behavior: angle.GetAbsoluteValue() " << angle.GetAbsoluteValue() << std::endl;


    fSpeed = angle.GetAbsoluteValue() * m_sRobotData.HALF_INTERWHEEL_DISTANCE / m_sRobotData.seconds_per_tick;
    fSpeed = fSpeed * 100.0; // converting to cm/s - as used in SetLinearVelocity
    fSpeed = Min<Real>(fSpeed, m_sRobotData.MaxSpeed);

    if(angle.GetValue() > 0.0f) // light source in the right, turn right
    {
        std::cout << "light source in the right: turn right " << angle.GetValue() << std::endl;
        fLeftWheelSpeed  = -fSpeed;
        fRightWheelSpeed = +fSpeed;
        std::cout << "P: fLeftWheelSpeed " << fLeftWheelSpeed <<  " fRightWheelSpeed " << fRightWheelSpeed  << std::endl;
    }
    else // light source in the left, turn left
    {
        std::cout << "light source in the left: turn left " << angle.GetValue() << std::endl;
        fLeftWheelSpeed  = +fSpeed;
        fRightWheelSpeed = -fSpeed;
        std::cout << "P: fLeftWheelSpeed " << fLeftWheelSpeed <<  " fRightWheelSpeed " << fRightWheelSpeed  << std::endl;
    }
}

/******************************************************************************/
/******************************************************************************/
