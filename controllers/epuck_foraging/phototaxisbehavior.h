#ifndef HOMINGTOLIGHTBEHAVIOR_H_
#define HOMINGTOLIGHTBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CPhototaxisBehavior : public CBehavior
{
public:
    CPhototaxisBehavior(Real m_fLightSensorThreshold, CRange<CDegrees>m_cGoStraightAngleRangeDegrees);

    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

protected:
    double           m_fLightSensorThreshold;
    CRange<CDegrees> m_cGoStraightAngleRangeDegrees;

    CVector2         m_cLightVector;


};


/******************************************************************************/
/******************************************************************************/

#endif 
