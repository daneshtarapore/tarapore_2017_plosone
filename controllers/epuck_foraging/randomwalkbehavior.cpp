#include "randomwalkbehavior.h"
//#include "random.h"


/******************************************************************************/
/******************************************************************************/

CRandomWalkBehavior::CRandomWalkBehavior(double f_change_direction_probability) :
    m_fChangeDirectionProbability(f_change_direction_probability) 
{    
}

/******************************************************************************/
/******************************************************************************/
    
bool CRandomWalkBehavior::TakeControl() 
{
    return true;
}

/******************************************************************************/
/******************************************************************************/

void CRandomWalkBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    if (m_fChangeDirectionProbability >= m_sSensoryData.m_pcRNG->Uniform(CRange<Real>(0.0, 1.0)))
    {
        Real fSpeed;

        CRadians angle = m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(CRadians::ZERO, CRadians::TWO_PI));

        fSpeed = CRadians::GetAbsoluteValue(angle.SignedNormalize()) * m_sRobotData.HALF_INTERWHEEL_DISTANCE / m_sRobotData.ticks_per_second;
        fSpeed = fSpeed * 100.0; // converting to cm/s - as used in SetLinearVelocity
        fSpeed = Min<Real>(fSpeed, m_sRobotData.MaxSpeed);


        if(angle > 0.0) //turn right
        {
            fLeftWheelSpeed  = fSpeed;
            fRightWheelSpeed = -fSpeed;
        }
        else
        {
            fLeftWheelSpeed  = -fSpeed;
            fRightWheelSpeed =  fSpeed;
        }

    }
    else
    {
        fLeftWheelSpeed = m_sRobotData.MaxSpeed;
        fRightWheelSpeed = m_sRobotData.MaxSpeed;
    }


}

/******************************************************************************/
/******************************************************************************/
