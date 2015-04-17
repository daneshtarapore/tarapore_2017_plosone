#ifndef FEATUREVECTORSINROBOTAGENT_H_
#define FEATUREVECTORSINROBOTAGENT_H_

#include <list>

struct StructFVsSensed
{
    unsigned int uFV;
    double fRobots;
    unsigned int uMostWantedState; // 0:  Dont know - no T-cells to make decision or E approx. equal to R (should not occur as T-cells are seeded for APCs where affinity=1)
                                   // 1:  Attack
                                   // 2:  Tolerate
                                   // 3:  FV not in sensed list
                                   // 4:  Suspicious to be abnormal

    // proportion of the past time-steps when the FV would have been deemed as abnormal
    //double fSuspicious; //we are now going to use a history of previously sensed feature vectors

    StructFVsSensed(unsigned int fv, double density)
    {
        uFV     = fv;
        fRobots = density;
    }

    StructFVsSensed(unsigned int fv, double density, unsigned int state)
    {
        uFV     = fv;
        fRobots = density;
        uMostWantedState = state;
    }
};

struct DetailedInformationFVsSensed
{
    unsigned int uFV;
    unsigned int uRobotId;
    double       fTimeSensed;

    DetailedInformationFVsSensed(unsigned int robotid, double timesensed, unsigned int fv)
    {
        uFV         = fv;
        uRobotId    = robotid;
        fTimeSensed = timesensed;
    }
};


typedef std::list<StructFVsSensed>              t_listFVsSensed;
typedef std::list<DetailedInformationFVsSensed> t_listDetailedInfoFVsSensed;

/******************************************************************************/
/******************************************************************************/

void UpdaterDistribution(t_listFVsSensed &listFVsSensed, unsigned int fv, double increment);

void UpdateFVDetails(t_listDetailedInfoFVsSensed &listDetailedInformationFVsSensed,
                     unsigned int fv, unsigned robotId, double timesensed);

void UpdateFeatureVectorDistribution(t_listFVsSensed &listFVsSensed, t_listDetailedInfoFVsSensed &listDetailedInformationFVsSensed,
                                     unsigned int fv, unsigned robotId, double timesensed);

/******************************************************************************/
/******************************************************************************/

#endif
