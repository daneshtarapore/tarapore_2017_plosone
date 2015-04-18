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
    CHomingToFoodBeaconBehavior(UInt8 BeaconData);

    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

protected:
    CVector2       m_cHomingVector;
    UInt8          m_iBeaconData;
};


/******************************************************************************/
/******************************************************************************/

#endif 
