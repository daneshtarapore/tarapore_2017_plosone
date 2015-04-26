#include <list>

/******************************************************************************/
/******************************************************************************/

#include "featurevectorsinrobotagent.h"

/******************************************************************************/
/******************************************************************************/


void UpdaterFvDistribution(t_listFVsSensed &listFVsSensed, t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                           CRandom::CRNG* m_pcRNG, Real m_fProbForget)
{

    t_listFVsSensed::iterator it_dist, it_history;

    // forget old FVs in distribution with probability m_fProbForgetFV
    it_dist = listFVsSensed.begin();
    while(it_dist != listFVsSensed.end())
    {
        if(m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f)) <= m_fProbForget)
        {
            it_dist = listFVsSensed.erase(it_dist);
            continue;
        }
        ++it_dist;
    }

    t_listFVsSensed tmp_list = t_listFVsSensed(listFVsSensed.begin(), listFVsSensed.end());

    listFVsSensed.clear();
    // updated listFVsSensed distribution with the most recent number of robots for different FVs.
    for(t_listMapFVsToRobotIds::iterator it_map = listMapFVsToRobotIds.begin(); it_map != listMapFVsToRobotIds.end(); ++it_map)
    {
        double increment = 1.0f;

        bool b_EntryInserted(false);

        // check if fv is in listFVsSensed
        // if so, update the value it holds by increment
        // if not insert it (while keeping list sorted based on fv) and initialize its value by increment
        for (it_dist = listFVsSensed.begin(); it_dist != listFVsSensed.end(); ++it_dist)
        {
            if(it_dist->uFV == it_map->uFV)
            {
                // if fv is already present
                it_dist->fRobots += increment;
                b_EntryInserted = true;
                break;
            }

            if(it_dist->uFV > it_map->uFV)
            {   // we assume the list is kept sorted.
                // if fv is absent
                listFVsSensed.insert(it_dist, StructFVsSensed(it_map->uFV, increment));
                b_EntryInserted = true;
                break;
            }
        }

        if(b_EntryInserted)
            continue;

        // when the list is empty or item is to be inserted in the end
        listFVsSensed.push_back(StructFVsSensed(it_map->uFV, increment));
    }

    // integrate into the current list listFVsSensed the history from tmp_list
    // if FV is the same in the current list, and in the history, - the history for that FV is ignored.
    // else the FV is integrated into the current list
    it_dist = listFVsSensed.begin(); it_history = tmp_list.begin();
    while(it_history != tmp_list.end() && it_dist != listFVsSensed.end())
    {
        if(it_history->uFV == it_dist->uFV)
        {
            ++it_history; ++it_dist;
            continue;
        }

        if(it_history->uFV < it_dist->uFV)
        {
                listFVsSensed.insert(it_dist, StructFVsSensed(it_history->uFV, it_history->fRobots, it_history->uMostWantedState));
                ++it_history;
        }
        else
             ++it_dist;
   }

    while(it_history != tmp_list.end())
    {
            listFVsSensed.push_back(StructFVsSensed(it_history->uFV, it_history->fRobots, it_history->uMostWantedState));
            ++it_history;
    }
}

/******************************************************************************/
/******************************************************************************/

void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                     unsigned int fv, unsigned robotId, double timesensed)
{
    t_listMapFVsToRobotIds::iterator itd;

    // check if robotId is in listMapFVsToRobotIds
    // if so, than update its fv if timesensed is more recent.
    // if not, than add its fv to the list

    bool robot_present(false);
    for (itd = listMapFVsToRobotIds.begin(); itd != listMapFVsToRobotIds.end(); ++itd)
    {
        if((*itd).uRobotId == robotId)
        {
            robot_present = true;

            // if the robot id is already present, add if the new information is more recent
            if(timesensed > (*itd).fTimeSensed)
            {
                (*itd).fTimeSensed = timesensed;
                (*itd).uRobotId = robotId;
                (*itd).uFV = fv;
            }
            return;
        }
    }

    if(!robot_present) // if the list is empty or the robot with given id is not present in the list
        listMapFVsToRobotIds.push_back(DetailedInformationFVsSensed(robotId, timesensed, fv));
}

/******************************************************************************/
/******************************************************************************/

bool sortfventries (const DetailedInformationFVsSensed& i, const DetailedInformationFVsSensed& j) { return (i.fTimeSensed < j.fTimeSensed); }

void TrimFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds, Real f_CurrentRobotTime, Real f_FvToId_MaintenanceTime)
{
    // when do we delete entries?
    // if an entry is older than f_FvToId_MaintenanceTime, we delete it.

    /*listMapFVsToRobotIds.sort(sortfventries);
    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(listMapFVsToRobotIds.size() > 10)
        itd = listMapFVsToRobotIds.erase(itd);*/

    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end()) // REMEMBER A FOR LOOP HERE WILL CAUSE RUNTIME ERRORS IF THE LAST T-CELL CLONAL POPULATION IS DELETED.
    {
         if (itd->fTimeSensed < (f_CurrentRobotTime - f_FvToId_MaintenanceTime))
         {
             itd = listMapFVsToRobotIds.erase(itd);
             continue;
         }

         ++itd;
    }
}

/******************************************************************************/
/******************************************************************************/

void UpdateVoterRegistry(t_listVoteInformationRobots   &listVoteInformationRobots,
                         t_listMapFVsToRobotIds   &listMapFVsToRobotIds,
                         t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds,
                         unsigned voter_id,
                         unsigned fv,
                         unsigned attack_tolerate_vote)
{

    /* Read the voter id:
     * Followed by <fv, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
     * If a vote is received,
     *                      1. map the fv to the robot id (if none existed - ignore vote???)
     *                      2. if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search.
     *                      3. if voter id has voted on mapped robot id before, ignore vote. But remember, the voted fv may map to other robot ids so continue search.
    */

    bool b_IdForFvFound(false);
    for(t_listMapFVsToRobotIds::iterator it_map = listMapFVsToRobotIds.begin(); it_map != listMapFVsToRobotIds.end(); ++it_map)
    {
        if(it_map->uFV == fv)
        {
            b_IdForFvFound = (true);

            /* map the fv to the robot id  (one-many function) */
            unsigned mappedRobotId = it_map->uRobotId;

            /* if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search. */
            bool b_ConsensusReachedOnId(false);
            for(t_listConsensusInfoOnRobotIds::iterator it_cons  = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
            {
                if(it_cons->uRobotId == mappedRobotId)
                {
                    b_ConsensusReachedOnId = (true);
                    break;
                }
            }

            if(b_ConsensusReachedOnId) /* continue search for other robot ids. the fv-id function is one-many*/
                continue;

            /* consensus has not yet been reached for mappedRobotId. lets vote*/
            bool b_MappedRobotVotedOnBefore(false);
            for(t_listVoteInformationRobots::iterator it_vote = listVoteInformationRobots.begin(); it_vote != listVoteInformationRobots.end(); ++it_vote)
            {
                /* robot is on the list of robots being voted on */
                if(it_vote->uRobotId == mappedRobotId)
                {
                    b_MappedRobotVotedOnBefore = (true);

                    /* if voter id has voted on mapped robot id before, ignore vote */
                    bool b_VoterIdHasVotedBefore(false);
                    for(std::list<unsigned>::iterator it_voterids = it_vote->uVoterIds.begin(); it_voterids != it_vote->uVoterIds.end(); ++it_voterids)
                    {
                        if((*it_voterids) == voter_id) /* yes voterid has voted before on mappedrobotid - ignore this vote */
                        {
                            b_VoterIdHasVotedBefore = (true);
                            break;
                        }
                    }

                    if(!b_VoterIdHasVotedBefore) /* no voterid has not voted before on mappedrobotid - ignore this vote */
                    {
                        it_vote->uVoterIds.push_back(voter_id);

                        if (attack_tolerate_vote == 1)
                            it_vote->attackvote_count++;
                        else
                            it_vote->toleratevote_count++;
                    }

                    break; /* robots on the list of robots being voted on, are unique */
                }

            }

            /* if mappedRobotId robot has not been voted on before, lets vote */
            if(!b_MappedRobotVotedOnBefore)
                listVoteInformationRobots.push_back(VoteInformationRobots(mappedRobotId, voter_id, attack_tolerate_vote));

        }
    }

    if(!b_IdForFvFound)
    {
        /* no robot id was found in the map, for the voted on fv;
         *                              options: 1. ignore vote???
         *                                       2. cast vote on ids with fvs close to the voted on fv?
          */
    }

}



/******************************************************************************/
/******************************************************************************/

//void UpdateFeatureVectorDistribution(t_listMapFVsToRobotIds &listMapFVsToRobotIds,
//                                     unsigned int fv, unsigned robotId, double timesensed)
//{
//    UpdateFVDetails(listMapFVsToRobotIds, fv, robotId, timesensed);
//}

/******************************************************************************/
/******************************************************************************/


//void UpdateFeatureVectorDistribution(t_listFVsSensed &listFVsSensed, t_listMapFVsToRobotIds &listMapFVsToRobotIds,
//                                     unsigned int fv, unsigned robotId, double timesensed)
//{

//    double increment = 1.0; // increment by one robot
//    UpdaterDistribution(listFVsSensed, fv, increment);
//    UpdateFVDetails(listMapFVsToRobotIds, fv, robotId, timesensed);
//}
