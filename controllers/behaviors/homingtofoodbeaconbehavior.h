#ifndef HOMINGTOFOODBEACONBEHAVIOR_H_
#define HOMINGTOFOODBEACONBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CHomingToFoodBeaconBehavior : public CBehavior
{
public:
    CHomingToFoodBeaconBehavior(int m_iBeaconData);
    CHomingToFoodBeaconBehavior();

    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

protected:
    CVector2       m_cHomingVector;
    int            m_iBeaconData;
};


/******************************************************************************/
/******************************************************************************/

#endif 
