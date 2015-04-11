#ifndef AGGREGATEBEHAVIOR_H_
#define AGGREGATEBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CAggregateBehavior : public CBehavior
{
public:
    CAggregateBehavior(Real m_fRangeAndBearing_RangeThreshold);

    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

protected:
    Real           m_fRangeAndBearing_RangeThreshold;
    CVector2       m_cAggregationVector;

};


/******************************************************************************/
/******************************************************************************/

#endif 
