#include "dispersebehavior.h"


/******************************************************************************/
/******************************************************************************/

CDisperseBehavior::CDisperseBehavior(double f_sensory_radius) :
    m_fSensoryRadius(f_sensory_radius) 
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CDisperseBehavior::TakeControl() 
{
    return false;//            m_pcAgent->CountAgents(m_fSensoryRadius, ROBOT) > 0;
}

/******************************************************************************/
/******************************************************************************/

void CDisperseBehavior::SimulationStep() 
{
    //m_tCenterOfMass = m_pcAgent->GetCenterOfMassOfSurroundingAgents(m_fSensoryRadius, ROBOT);
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CDisperseBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    double CHANGE_CENTER_OF_MASS_PROB = 0.05;
    double m_fCoMXOffset, m_fCoMYOffset;

    /*if (Random::nextDouble() < CHANGE_CENTER_OF_MASS_PROB)
    {
        m_fCoMXOffset = Random::nextNormGaussian();
        m_fCoMYOffset = Random::nextNormGaussian();
    }*/

    // CoM wrt the agent
    //m_tCenterOfMass.x = m_tCenterOfMass.x - m_pcAgent->GetPosition()->x;
    //m_tCenterOfMass.y = m_tCenterOfMass.y - m_pcAgent->GetPosition()->y;

    // The opposite direction from the CoM wrt the agent + random offset to avoid scenarios of a cluster of 3 stationary agents in a straight line.
    //m_tCenterOfMass.x = -m_tCenterOfMass.x + m_pcAgent->GetPosition()->x + m_fCoMXOffset;
    //m_tCenterOfMass.y = -m_tCenterOfMass.y + m_pcAgent->GetPosition()->y + m_fCoMYOffset;

    //m_pcAgent->MoveTowards(m_tCenterOfMass, m_pcAgent->GetMaximumSpeed());
}

/******************************************************************************/
/******************************************************************************/
