#include "dispersebehavior.h"


/******************************************************************************/
/******************************************************************************/

CDisperseBehavior::CDisperseBehavior(Real ProximitySensorThreshold, CRange<CDegrees> GoStraightAngleRangeDegrees) :
    m_fProximitySensorThreshold(ProximitySensorThreshold),
    m_cGoStraightAngleRangeDegrees(GoStraightAngleRangeDegrees)
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CDisperseBehavior::TakeControl() 
{

    /* Get readings from proximity sensor */
    /* Sum them together */
    m_cDiffusionVector.Set(0.0f, 0.0f);
    for(size_t i = 0; i <  m_sSensoryData.m_ProximitySensorData.size(); ++i)
    {
        m_cDiffusionVector += CVector2(m_sSensoryData.m_ProximitySensorData[i].Value, m_sSensoryData.m_ProximitySensorData[i].Angle);
    }
    m_cDiffusionVector /= m_sSensoryData.m_ProximitySensorData.size();

    /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
    if(m_cGoStraightAngleRangeDegrees.WithinMinBoundIncludedMaxBoundIncluded(ToDegrees(m_cDiffusionVector.Angle())) &&
       m_cDiffusionVector.Length() < m_fProximitySensorThreshold)
        return false;
    else
    {
        std::cout << " Disperse behavior taking control " << std::endl;
        return true;
    }


    //return false;//            m_pcAgent->CountAgents(m_fSensoryRadius, ROBOT) > 0;
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CDisperseBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
     CVector2 m_cHeadingVector = -m_cDiffusionVector;

     if(m_cHeadingVector.Angle().GetAbsoluteValue() <= ToRadians(CDegrees(35.0f)).GetValue()) // obstacle behind robot // move straight ahead.
     {
         fLeftWheelSpeed  = m_sRobotData.MaxSpeed;
         fRightWheelSpeed = m_sRobotData.MaxSpeed;
         return;
     }

     Real fSpeed;
     CRadians angle = m_cHeadingVector.Angle();

     /*std::cout << " Disperse behavior: m_cHeadingVector.Angle() " << m_cHeadingVector.Angle() << std::endl;
     angle = angle.SignedNormalize();
     std::cout << " Disperse behavior: angle.SignedNormalize() " << angle.SignedNormalize() << std::endl;

     std::cout << " Disperse behavior: angle.GetAbsoluteValue() " << angle.GetAbsoluteValue() << std::endl;*/


     fSpeed = angle.GetAbsoluteValue() * m_sRobotData.HALF_INTERWHEEL_DISTANCE / m_sRobotData.seconds_per_tick;
     fSpeed = fSpeed * 100.0; // converting to cm/s - as used in SetLinearVelocity
     fSpeed = Min<Real>(fSpeed, m_sRobotData.MaxSpeed);


     if(angle.GetValue() > 0.0f) // obstacle in the right, turn left
     {
         std::cout << "Disperse: turn left " << angle.GetValue() << std::endl;
         fLeftWheelSpeed  = -fSpeed;
         fRightWheelSpeed = fSpeed;
         std::cout << "Disperse: fLeftWheelSpeed " << fLeftWheelSpeed <<  " fRightWheelSpeed " << fRightWheelSpeed  << std::endl;
     }
     else // obstacle in the left, turn right
     {
         std::cout << "Disperse: turn right " << angle.GetValue() << std::endl;
         fLeftWheelSpeed  = fSpeed;
         fRightWheelSpeed = -fSpeed;
         std::cout << "Disperse: fLeftWheelSpeed " << fLeftWheelSpeed <<  " fRightWheelSpeed " << fRightWheelSpeed  << std::endl;
     }


    /*{
        CRadians angle = m_cHeadingVector.Angle();

        // the diffusion-vector's units are in the same as IR reflected intensity. We need to convert that to meters to use this approach.
        Real fSpeed1   = 0.5f * (2.0f * m_cHeadingVector.Length() - m_sRobotData.INTERWHEEL_DISTANCE * m_cHeadingVector.Angle().SignedNormalize().GetValue()) /
                m_sRobotData.seconds_per_tick;
        fSpeed1 = fSpeed1 * 100.0; // converting to cm/s - as used in SetLinearVelocity

        Real fSpeed2  = 0.5f * (2.0f * m_cHeadingVector.Length() + m_sRobotData.INTERWHEEL_DISTANCE * m_cHeadingVector.Angle().SignedNormalize().GetValue()) /
                m_sRobotData.seconds_per_tick;
        fSpeed2 = fSpeed2 * 100.0; // converting to cm/s - as used in SetLinearVelocity


        std::cout << " m_cHeadingVector.Length() " << m_cHeadingVector.Length() << std::endl;

        if(fSpeed1 > 0.0f)
            fSpeed1 = Min<Real>(fSpeed1, m_sRobotData.MaxSpeed);
        else
            fSpeed1 = Max<Real>(fSpeed1, -m_sRobotData.MaxSpeed);

        if(fSpeed2 > 0.0f)
            fSpeed2 = Min<Real>(fSpeed2, m_sRobotData.MaxSpeed);
        else
            fSpeed2 = Max<Real>(fSpeed2, -m_sRobotData.MaxSpeed);


        fLeftWheelSpeed = fSpeed1;
        fRightWheelSpeed = fSpeed2;
     }*/
}

/******************************************************************************/
/******************************************************************************/
