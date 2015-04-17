#include <list>

/******************************************************************************/
/******************************************************************************/

#include "featurevectorsinrobotagent.h"

/******************************************************************************/
/******************************************************************************/

void UpdaterDistribution(t_listFVsSensed &listFVsSensed, unsigned int fv, double increment)
{
    t_listFVsSensed::iterator it;

    // check if fv is in listFVsSensed
    // if so, update the value it holds by increment
    // if not insert it (while keeping list sorted based on fv) and initialize its value by increment
    for (it = listFVsSensed.begin(); it != listFVsSensed.end(); ++it)
    {
        if((*it).uFV == fv)
        {
            // if fv is already present
            (*it).fRobots += increment;
            return;
        }

        if((*it).uFV > fv)
        {   // we assume the list is kept sorted.
            // if fv is absent
            listFVsSensed.insert(it, StructFVsSensed(fv, increment));
            return;
        }
    }
    // when the list is empty or item is to be inserted in the end
    listFVsSensed.push_back(StructFVsSensed(fv, increment));
}

/******************************************************************************/
/******************************************************************************/

void UpdateFVDetails(t_listDetailedInfoFVsSensed &listDetailedInformationFVsSensed,
                     unsigned int fv, unsigned robotId, double timesensed)
{
    t_listDetailedInfoFVsSensed::iterator itd;

    // check if fv is in listFVsSensed
    // if so, update the value it holds by increment
    // if not insert it (while keeping list sorted based on fv) and initialize its value by increment
    for (itd = listDetailedInformationFVsSensed.begin(); itd != listDetailedInformationFVsSensed.end(); ++itd)
    {
        if((*itd).uFV == fv)
        {
            // if fv is already present - from another robot
            listDetailedInformationFVsSensed.insert(itd, DetailedInformationFVsSensed(robotId, timesensed, fv));
            return;
        }

        if((*itd).uFV > fv)
        {   // we assume the list is kept sorted.
            // if fv is absent
            listDetailedInformationFVsSensed.insert(itd, DetailedInformationFVsSensed(robotId, timesensed, fv));
            return;
        }
    }
    // when the list is empty or item is to be inserted in the end
    listDetailedInformationFVsSensed.push_back(DetailedInformationFVsSensed(robotId, timesensed, fv));
}

/******************************************************************************/
/******************************************************************************/

void UpdateFeatureVectorDistribution(t_listFVsSensed &listFVsSensed, t_listDetailedInfoFVsSensed &listDetailedInformationFVsSensed,
                                     unsigned int fv, unsigned robotId, double timesensed)
{

    double increment = 1.0; // increment by one robot
    UpdaterDistribution(listFVsSensed, fv, increment);
    UpdateFVDetails(listDetailedInformationFVsSensed, fv, robotId, timesensed);
}

/******************************************************************************/
/******************************************************************************/
