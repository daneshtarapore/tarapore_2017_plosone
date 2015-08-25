#ifndef FEATUREVECTORSINROBOTAGENT_H_
#define FEATUREVECTORSINROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/

#include <list>
#include <algorithm>

/******************************************************************************/
/******************************************************************************/

#include <argos3/core/utility/math/vector2.h>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;


/*
 * Stores the distribution of observed Feature vectors - both recent and past (determined by forget probability)
 */
struct StructFVsSensed
{
    unsigned int uFV;
    double fRobots;
    unsigned int uMostWantedState; // 0:  Dont know - no T-cells to make decision or E approx. equal to R (should not occur as T-cells are seeded for APCs where affinity=1)
                                   // 1:  Attack
                                   // 2:  Tolerate

    // proportion of the past time-steps when the FV would have been deemed as abnormal
    //double fSuspicious; //we are now going to use a history of previously sensed feature vectors

    StructFVsSensed(unsigned int fv, double density)
    {
        uFV     = fv;
        fRobots = density;
        uMostWantedState = 999;
    }

    StructFVsSensed(unsigned int fv, double density, unsigned int state)
    {
        uFV     = fv;
        fRobots = density;
        uMostWantedState = state;
    }
};

/*
 * Stores the map of observed FVs to robot ids (many to one function map)
 */
struct DetailedInformationFVsSensed
{
    unsigned int uFV;
    unsigned int uRobotId;
    double       fTimeSensed;

    double       f_TimesAttacked, f_TimesTolerated; //! we count the number of times the fv uFV is attacked / tolerated according to the CRM. During the last X time-steps, we send out the vote depending on what is in the majority???

    DetailedInformationFVsSensed(unsigned int robotid, double timesensed, unsigned int fv)
    {
        uFV         = fv;
        uRobotId    = robotid;
        fTimeSensed = timesensed;
    }
};


struct ConsensusInformationRobots
{
    unsigned uRobotId;
    unsigned consensus_state; //attack or tolerate

    ConsensusInformationRobots(unsigned int id, unsigned state)
    {
        uRobotId        = id;
        consensus_state = state;
    }
};

struct VoteInformationRobots
{
    unsigned uRobotId;

    std::list <unsigned> uVoterIds;
    unsigned attackvote_count;
    unsigned toleratevote_count;

    VoteInformationRobots(unsigned Id, unsigned voterId, unsigned attack_tolerate_vote)
    {
        uRobotId        = Id;

        uVoterIds.clear();
        uVoterIds.push_back(voterId);


        attackvote_count = 0; toleratevote_count = 0;
        if (attack_tolerate_vote == 1)
            attackvote_count++;
        else
            toleratevote_count++;
    }
};



typedef std::list<StructFVsSensed>              t_listFVsSensed;
typedef std::list<DetailedInformationFVsSensed> t_listMapFVsToRobotIds;
typedef std::list<ConsensusInformationRobots>   t_listConsensusInfoOnRobotIds;
typedef std::list<VoteInformationRobots>        t_listVoteInformationRobots;

/******************************************************************************/
/******************************************************************************/

/*
 *
 *
 */
void UpdaterFvDistribution(t_listFVsSensed &listFVsSensed, t_listMapFVsToRobotIds &listDetailedInformationFVsSensed,
                           CRandom::CRNG *m_pcRNG, Real m_fProbForget);


/*
 *
 *
 */
void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listDetailedInformationFVsSensed,
                     unsigned int fv, unsigned robotId, double timesensed);


/*
 *
 *
 */
void UpdateVoterRegistry(t_listVoteInformationRobots   &listVoteInformationRobots,
                         t_listMapFVsToRobotIds   &listMapFVsToRobotIds,
                         t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds,
                         unsigned voter_id, unsigned fv,
                         unsigned attack_tolerate_vote);

/*
 * Removes old maps from FVs to Robot Ids
 * f_IdToFV_MaintenanceTime: how long we maintain a particular mapping from fv to robot id
   when do we delete old map entries ? if an entry is older than f_FvToId_MaintenanceTime, we delete it.

 */
void TrimFvToRobotIdMap(t_listMapFVsToRobotIds &listDetailedInformationFVsSensed, Real f_CurrentRobotTime, Real f_FvToId_MaintenanceTime);

/******************************************************************************/
/******************************************************************************/

#endif
