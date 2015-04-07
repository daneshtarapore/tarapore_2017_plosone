#ifndef DISPERSEBEHAVIOR_H_
#define DISPERSEBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CDisperseBehavior : public CBehavior 
{
public:
    CDisperseBehavior(double f_sensory_radius);

    virtual void SimulationStep();    
    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

protected:
    double     m_fSensoryRadius;
    CVector2  m_tCenterOfMass;

};


/******************************************************************************/
/******************************************************************************/

#endif 
