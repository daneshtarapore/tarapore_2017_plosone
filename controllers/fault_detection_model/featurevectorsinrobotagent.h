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

#define ConsensusOnMapOfIDtoFV

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
 * Stores the map of observed FVs to robot ids (one to many function map)
 */
struct DetailedInformationFVsSensed
{
    unsigned int uRobotId;

    unsigned int uFV;
    double       fTimeSensed;

    //double       f_TimesAttacked, f_TimesTolerated; //! we count the number of times the fv uFV is attacked / tolerated according to the CRM. During the last X time-steps, we send out the vote depending on what is in the majority???

    DetailedInformationFVsSensed(unsigned int robotid, double timesensed, unsigned int fv)
    {
        uFV         = fv;
        uRobotId    = robotid;
        fTimeSensed = timesensed;
    }


    /*Additional data structure for establishing consensus on id-fv map entries*/
    std::vector<unsigned> vec_ObserverRobotIds;
    std::vector<unsigned> vec_ObservedRobotFVs;
    std::vector<Real>     vec_TimeObserved;

    DetailedInformationFVsSensed(unsigned int ObserverRobotId, unsigned int ObservedRobotId, double timesensed, unsigned int fv)
    {
        vec_ObserverRobotIds.clear(); vec_ObservedRobotFVs.clear(); vec_TimeObserved.clear();

        uRobotId = ObservedRobotId;
        vec_ObserverRobotIds.push_back(ObserverRobotId);
        vec_TimeObserved.push_back(timesensed);
        vec_ObservedRobotFVs.push_back(fv);
    }

    void AddNewInformationFVsSensed(unsigned int ObserverRobotId, double timesensed, unsigned int fv)
    {
        vec_ObserverRobotIds.push_back(ObserverRobotId);
        vec_TimeObserved.push_back(timesensed);
        vec_ObservedRobotFVs.push_back(fv);
    }

    void SelectBestFVFromAllObservedFVs(unsigned u_NumFeatures, CRandom::CRNG* m_pcRNG_FVs)
    {
        std::vector <int> vec_countbestfeatures(u_NumFeatures, 0);

        for (size_t fv_index = 0; fv_index < vec_ObservedRobotFVs.size(); ++fv_index)
        {
            for(size_t feature_index = 0; feature_index < u_NumFeatures; ++feature_index)
            {
                vec_countbestfeatures[feature_index] += ((vec_ObservedRobotFVs[fv_index] >> feature_index) & 0x1) ? 1 : -1;
            }
        }

        uFV = 0u;
        for(size_t feature_index = 0; feature_index < u_NumFeatures; ++feature_index)
        {
            if(vec_countbestfeatures[feature_index] > 0)
                uFV += (1 << feature_index);
            else if(vec_countbestfeatures[feature_index] == 0)
            {
                if (m_pcRNG_FVs->Uniform(CRange<Real>(0.0f, 1.0f)) > 0.5f)
                    uFV += (1 << feature_index);
            }
        }
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
void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listDetailedInformationFVsSensed,
                     unsigned int ObserverRobotId, unsigned int fv, unsigned ObservedRobotId, double timesensed);



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


/*
 *
 */
void PrintFvToRobotIdMap(unsigned u_MapOfRobotId, t_listMapFVsToRobotIds &listDetailedInformationFVsSensed, unsigned u_ObservedRobotId=99999u);


/*
 *
 *
 */
void PrintVoterRegistry(unsigned u_VoterRegistryOfRobotId, t_listVoteInformationRobots &listVoteInformationRobots, unsigned u_VotedOnRobotId=99999u);


/*
 *
 */
void PrintConsensusRegistry(unsigned u_ConsensusRegistryOfRobotId, t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds, unsigned u_ConsensusOnRobotId=99999u);


/*
 * Select the best feature value for each feature from all the observered feature-vectors for each map entry.
 */
void SelectBestFVFromAllObservedFVs(t_listMapFVsToRobotIds &listDetailedInformationFVsSensed, unsigned u_NumFeatures, CRandom::CRNG* m_pcRNG_FVs);

/******************************************************************************/
/******************************************************************************/

#endif
