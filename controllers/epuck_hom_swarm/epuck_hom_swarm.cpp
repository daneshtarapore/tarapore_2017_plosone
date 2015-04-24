/* Include the controller definition */
#include "epuck_hom_swarm.h"

/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

UInt8 CEPuckHomSwarm::BEACON_SIGNAL = 200;

#define END_BUFFER 240
#define OBSERVED_FVS_PACKET 100
#define VOTER_PACKET 120
#define ATTACK_VOTE 130
#define TOLERATE_VOTE 140
#define ATTACK_CONSENSUS 150
#define TOLERATE_CONSENSUS 160

/****************************************/
/****************************************/

/*
 * Probability to forget FV in distribution
 */
#define PROBABILITY_FORGET_FV 0.002f

/*
 * Consensus threshold on FVs.
 */
#define CONSENSUS_THRESHOLD 3u /* odd number so that we don't have a tie in attackers and tolerators - but this is just a threshold. number of voters may be more than threshold */

/*
 * The results of the CRM are valid for atmost 10s in the absence of any FVs to run the CRM
 */
#define CRM_RESULTS_VALIDFOR_SECONDS 10.0f

/*
 * The vote counts and consensus are valid for atmost 10s before being refreshed
 */
#define VOTCON_RESULTS_VALIDFOR_SECONDS 10.0f

/****************************************/
/****************************************/

CBehavior::SensoryData CBehavior::m_sSensoryData;
CBehavior::RobotData CBehavior::m_sRobotData;

CProprioceptiveFeatureVector::RobotData CProprioceptiveFeatureVector::m_sRobotData;

/****************************************/
/****************************************/

CEPuckHomSwarm::ExperimentToRun::ExperimentToRun() :
    SBehavior(SWARM_AGGREGATION),
    FBehavior(FAULT_NONE),
    id_FaultyRobotInSwarm("-1") {}


void CEPuckHomSwarm::ExperimentToRun::Init(TConfigurationNode& t_node)
{
    std::string swarmbehav, errorbehav;

    try
    {
        GetNodeAttribute(t_node, "swarm_behavior", swarmbehav);
        GetNodeAttribute(t_node, "fault_behavior", errorbehav);
        GetNodeAttribute(t_node, "id_faulty_robot", id_FaultyRobotInSwarm);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);

    if (swarmbehav.compare("SWARM_AGGREGATION") == 0)
        SBehavior = SWARM_AGGREGATION;
    else if (swarmbehav.compare("SWARM_DISPERSION") == 0)
        SBehavior = SWARM_DISPERSION;
    else if (swarmbehav.compare("SWARM_HOMING") == 0)
        SBehavior = SWARM_HOMING;
    else
    {
        std::cerr << "invalid swarm behavior";
        exit(-1);
    }

    if (errorbehav.compare("FAULT_NONE") == 0)
        FBehavior = FAULT_NONE;
    else if  (errorbehav.compare("FAULT_STRAIGHTLINE") == 0)
        FBehavior = FAULT_STRAIGHTLINE;
    else if  (errorbehav.compare("FAULT_RANDOMWALK") == 0)
        FBehavior = FAULT_RANDOMWALK;
    else if  (errorbehav.compare("FAULT_CIRCLE") == 0)
        FBehavior = FAULT_CIRCLE;
    else if  (errorbehav.compare("FAULT_STOP") == 0)
        FBehavior = FAULT_STOP;
    else
    {
        std::cerr << "invalid fault behavior";
        exit(-1);
    }
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::SWheelTurningParams::Init(TConfigurationNode& t_node)
{
    try
    {
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
}

/****************************************/
/****************************************/

CEPuckHomSwarm::CEPuckHomSwarm() :
    m_pcWheels(NULL),
    m_pcLEDs(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcProximity(NULL),
    m_pcRNG(NULL),
    b_damagedrobot(false)
{
    m_fInternalRobotTimer=0.0f;
    listFVsSensed.clear();
    listMapFVsToRobotIds.clear();
    listMapFVsToRobotIds_relay.clear();
    listConsensusInfoOnRobotIds.clear();
    listVoteInformationRobots.clear();

    b_CRM_Run = false;
    m_fCRM_RUN_TIMESTAMP = 0.0f;
    m_uRobotFV = 9999; // for debugging urposes
}

/****************************************/
/****************************************/

CEPuckHomSwarm::~CEPuckHomSwarm()
{
    // delete all behaviors

    // delete list of feature-vectors

    // delete crm model's data containers
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::Init(TConfigurationNode& t_node)
{
    try
    {
        /*
       * Initialize sensors/actuators
       */
        m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
        m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
        m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
        m_pcProximity = GetSensor  <CCI_EPuckProximitySensor        >("epuck_proximity"    );

        /*
       * Parse XML parameters
       */
        /* Experiment to run */
        m_sExpRun.Init(GetNode(t_node, "experiment_run"));
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing the e-puck hom_swarm controller for robot \"" << GetId() << "\"", ex);

    /*
    * Initialize other stuff
    */
    /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
    m_pcRNG = CRandom::CreateRNG("argos");

    Reset();

    CBehavior::m_sRobotData.MaxSpeed = m_sWheelTurningParams.MaxSpeed;
    CBehavior::m_sRobotData.iterations_per_second  = 10.0f; /*10 ticks per second so dt=0.01s. i.e., the controlcycle is run 10 times per second*/
    CBehavior::m_sRobotData.seconds_per_iterations = 1.0f / CBehavior::m_sRobotData.iterations_per_second;
    CBehavior::m_sRobotData.HALF_INTERWHEEL_DISTANCE = 0.053f * 0.5f;
    CBehavior::m_sRobotData.INTERWHEEL_DISTANCE  = 0.053f;
    CBehavior::m_sRobotData.WHEEL_RADIUS = 0.0205f;

    CBehavior::m_sRobotData.m_cNoTurnOnAngleThreshold   = ToRadians(CDegrees(10.0f)); //10.0 - straight to food spot; 35.0 spiral to food spot
    CBehavior::m_sRobotData.m_cSoftTurnOnAngleThreshold = ToRadians(CDegrees(70.0f));

    if(this->GetId().compare("ep"+m_sExpRun.id_FaultyRobotInSwarm) == 0)
        b_damagedrobot = true;




    CProprioceptiveFeatureVector::m_sRobotData.MaxLinearSpeed           = m_sWheelTurningParams.MaxSpeed; //cm/s
    CProprioceptiveFeatureVector::m_sRobotData.MaxLinearAcceleration    = m_sWheelTurningParams.MaxSpeed; //cm/s^2


    CProprioceptiveFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = 0.053f * 0.5f; // m
    CProprioceptiveFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = 0.053f; // m
    CProprioceptiveFeatureVector::m_sRobotData.MaxAngularSpeed          = (m_sWheelTurningParams.MaxSpeed + m_sWheelTurningParams.MaxSpeed) /
            (CBehavior::m_sRobotData.INTERWHEEL_DISTANCE * 100.0f); //rad/s
    CProprioceptiveFeatureVector::m_sRobotData.MaxAngularSpeed          = CProprioceptiveFeatureVector::m_sRobotData.MaxAngularSpeed; //rad/s^2;

    CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second    = 10.0f; /*10 ticks per second so dt=0.01s. i.e., the controlcycle is run 10 times per second*/
    CProprioceptiveFeatureVector::m_sRobotData.seconds_per_iterations   = 1.0f / CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second;
    CProprioceptiveFeatureVector::m_sRobotData.WHEEL_RADIUS             = 0.0205f; //m


    // robotid set to 0 for now
    crminAgent = new CRMinRobotAgentOptimised(RobotIdStrToInt(), CProprioceptiveFeatureVector::NUMBER_OF_FEATURES);
}

/****************************************/
/****************************************/

unsigned CEPuckHomSwarm::SumFVDist(t_listFVsSensed& FVsSensed)
{
    unsigned robotcount = 0;
    for (t_listFVsSensed::iterator it = FVsSensed.begin(); it != FVsSensed.end(); ++it)
        robotcount += it->fRobots;

    return robotcount;
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::ControlStep()
{
    m_fInternalRobotTimer+=1.0f;

    if(b_damagedrobot && (m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_CIRCLE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_STOP))
        RunGeneralFaults();

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_DISPERSION ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING)
        RunHomogeneousSwarmExperiment();


    CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_pcProximity->GetReadings(), m_pcRABS->GetReadings());

    Real leftSpeed = 0.0, rightSpeed = 0.0;
    bool bControlTaken = false;
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        if (!bControlTaken)
        {
            bControlTaken = (*i)->TakeControl();
            if (bControlTaken)
            {
                (*i)->Action(leftSpeed, rightSpeed);
            }
        } else
            (*i)->Suppress();
    }
    m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed); // in cm/s


    /****************************************/
    /****************************************/
    /* Estimate feature-vectors - proprioceptively */
    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(m_fInternalRobotTimer, m_pcProximity->GetReadings(), m_pcRABS->GetReadings(), leftSpeed, rightSpeed);
    m_cProprioceptiveFeatureVector.SimulationStep();
    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue();

    /* Communicate your id and proprioceptively computed FV to whoever is in range, using the RAB sensor*/
    /* Also relay the id and fvs of neighbours, received by you in the previous control cycle */
    if ((unsigned)m_fInternalRobotTimer%2u == 0)
        SendFVsToNeighbours();

    /* Listen for robot ids + feature vectors from neighbours and then assimilate them */
    Sense(PROBABILITY_FORGET_FV);

    if(((unsigned)m_fInternalRobotTimer % (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) == 0u)
    {
        listConsensusInfoOnRobotIds.clear();
        listVoteInformationRobots.clear();
    }

    /* Listen for voting packets and consensus packets from neighbours*/
    ReceiveVotesAndConsensus();
    EstablishConsensus();


    Real TimeSinceCRM = (m_fInternalRobotTimer - m_fCRM_RUN_TIMESTAMP) * CProprioceptiveFeatureVector::m_sRobotData.seconds_per_iterations; // in seconds
    if (b_CRM_Run && (TimeSinceCRM > CRM_RESULTS_VALIDFOR_SECONDS)) /* the results of the CRM are no longer valid */
        b_CRM_Run = false;

    if((m_fInternalRobotTimer > 450.0) && (listFVsSensed.size() > 0)) // the robot has atleast had one FV entry in its distribution. if not the CRM will crash.
    {
        //std::cout << "RobotTimer " << m_fInternalRobotTimer << " Id. " << RobotIdStrToInt(GetId()) << std::endl;
        crminAgent->SimulationStepUpdatePosition(m_fInternalRobotTimer, &listFVsSensed);
        b_CRM_Run = true;
        m_fCRM_RUN_TIMESTAMP = m_fInternalRobotTimer;
    }

    if(b_CRM_Run) // a failsafe to make sure you don't use outdated CRM results
    {
        // the CRM results on FVs in listFVsSensed is not outdated
        for(t_listFVsSensed::iterator it_fv = listFVsSensed.begin(); it_fv != listFVsSensed.end(); ++it_fv)
            UpdateVoterRegistry(listVoteInformationRobots,
                                listMapFVsToRobotIds,
                                listConsensusInfoOnRobotIds,
                                RobotIdStrToInt(), it_fv->uFV, it_fv->uMostWantedState);
    }

    if ((unsigned)m_fInternalRobotTimer%2u == 1)
        SendCRMResultsAndConsensusToNeighbours(b_CRM_Run); // only send CRM results if they are valid

}

/****************************************/
/****************************************/

void CEPuckHomSwarm::SendFVsToNeighbours()
{
    /*Communicate your id and FV, and relay the id and fvs of neighbours, received by you in the previous control cycle*/
    WriteToCommunicationChannel(RobotIdStrToInt(), m_cProprioceptiveFeatureVector.GetValue(), listMapFVsToRobotIds_relay);
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::SendCRMResultsAndConsensusToNeighbours(bool b_CRM_Results_Valid)
{
    /* Commmunicate your CRM results to your neighbours
     * Also broadcast consensus information - add any new consensus info to your local list and send it out again */
    WriteToCommunicationChannel(RobotIdStrToInt(), listMapFVsToRobotIds,
                                listFVsSensed, listConsensusInfoOnRobotIds, b_CRM_Results_Valid);
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::Sense(Real m_fProbForget)
{
    if(m_fProbForget < 1.0f)
    {
        std:cerr << "History of FVs still to be implemented";
        exit(-1);
    }

    const CCI_RangeAndBearingSensor::TReadings& tmp = m_pcRABS->GetReadings();

    /* Listen for feature vectors from neighbours */
    /* Read the id and proprioceptively computed FV of your neighbours. Read from communication channel and stored in listMapFVsToRobotIds */
    /* Mark these read FVs as to be relayed. Stored separately in listMapFVsToRobotIds_relay */
    /* Also read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */
    bool read_status = ReadFromCommunicationChannel_IdFv(tmp); /* returns true if successfully read id and fvs from at least one neighbour*/


    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBehavior::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS); /*remove entries older than 10s */

    //listFVsSensed.clear();
    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG, m_fProbForget); // update listFVsSensed

    /*if(m_fInternalRobotTimer == 2616 && (this->GetId().compare("ep4") == 0))
        std::cout << "listFVsSensed.size() " << listMapFVsToRobotIds.size() << std::endl;*/
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::ReceiveVotesAndConsensus()
{
    /* Listen to votes and consensus from neighbours */
    /* Read the voter id:
     * Followed by <fv, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
     * If a vote is received,
     *                      1. map the fv to the robot id (if none existed - ignore vote???)
     *                      2. if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search.
     *                      3. if voter id has voted on mapped robot id before, ignore vote. But remember, the voted fv may map to other robot ids so continue search.
     *
     * If a consensus is received,
     *                      1. update listConsensusInfoOnRobotIds to include id and ATTACK_CONSENSUS / TOLERATE_CONSENSUS
     *                      2. if listConsensusInfoOnRobotIds already has id with a different CONSENSUS from the received message - problem with discrepancy in consensus???? Could except consensus from lowest voter id as true???
     *
    */

    const CCI_RangeAndBearingSensor::TReadings& tmp = m_pcRABS->GetReadings();
    bool read_status = ReadFromCommunicationChannel_VotCon(tmp); /* returns true if successfully read votes or consensus from at least one neighbour*/
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::EstablishConsensus()
{
    /* For each robot id in listVoteInformationRobots that is not in listConsensusInfoOnRobotIds
     * If #votes-registered > CONSENSUS_THRESHOLD, or if we are close to the expiry time of current VOTCON_RESULTS_VALIDFOR_SECONDS window (1s remaining)
     * Establish temporary consensus on robot id by adding it to listConsensusInfoOnRobotIds
     */

    for (t_listVoteInformationRobots::iterator it_vot = listVoteInformationRobots.begin(); it_vot != listVoteInformationRobots.end(); ++it_vot)
    {
        unsigned VotedOnRobotId = it_vot->uRobotId;
        bool b_ConsensusReachedOnId(false);

        for (t_listConsensusInfoOnRobotIds::iterator it_cons = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
        {
            if (it_cons->uRobotId == VotedOnRobotId)
            {
                b_ConsensusReachedOnId = true;
                break; /* the robot id in consensus list are unique */
            }

        }

        if (b_ConsensusReachedOnId)
            continue; /* consensus already reached for VotedOnRobotId, lets go to the next robot in the listVoteInformationRobots list */
        else
        {
            bool b_OneSecondToVotConReset = (((unsigned)m_fInternalRobotTimer %
                                              (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) >= 9u);

            /* If #votes-registered > CONSENSUS_THRESHOLD, or if we are close to the expiry time of current VOTCON_RESULTS_VALIDFOR_SECONDS window (1s remaining) after which the consensus and vote vectors will be cleared */
            if ((it_vot->uVoterIds.size() > CONSENSUS_THRESHOLD) || b_OneSecondToVotConReset) /* at least one vote will be registered. establish consensus on that */
            {
                listConsensusInfoOnRobotIds.push_back(ConsensusInformationRobots(it_vot->uRobotId,
                                                                                 (it_vot->attackvote_count > it_vot->toleratevote_count)?1u:2u)); /* if equal votes, we tolerate robot*/

            }
        }
    }
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::RunGeneralFaults()
{
    m_pcLEDs->SetAllColors(CColor::RED);

    m_vecBehaviors.clear();
    if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE)
    {
        CRandomWalkBehavior* pcStraightLineBehavior = new CRandomWalkBehavior(0.0f);
        m_vecBehaviors.push_back(pcStraightLineBehavior);
    }
    else if (m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if (m_sExpRun.FBehavior == ExperimentToRun::FAULT_CIRCLE)
    {
        CCircleBehavior* pcCircleBehavior = new CCircleBehavior();
        m_vecBehaviors.push_back(pcCircleBehavior);

    }
    else //m_sExpRun.FBehavior == ExperimentToRun::FAULT_STOP
    {}
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::RunHomogeneousSwarmExperiment()
{
    m_vecBehaviors.clear();

    if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(60.0f); //range threshold in cm
        m_vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_DISPERSION)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING)
    {
        if(this->GetId().compare("ep0") == 0)
        {
            // ep0 is the beacon robot
            /* Sends out data '200' with RABS that you are a beacon. Neighbouring robots will use this data to home in on your position */
            // 200 is way above the maximum 6-bit FV of range [0,63] and the possible ids of robots (max 20 in swarm). Also within 1 byte [0-255] of RAB sensor data [4 or 10 1 byte array depending on epuck or footbot]
            m_pcRABA->SetData(0, BEACON_SIGNAL);
            m_pcLEDs->SetAllColors(CColor::YELLOW);
        }
        else
        {
            CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
            m_vecBehaviors.push_back(pcDisperseBehavior);

            Real MAX_BEACON_SIGNAL_RANGE = 1.0f; //1m
            CHomingToFoodBeaconBehavior* pcHomingToFoodBeaconBehavior = new CHomingToFoodBeaconBehavior(BEACON_SIGNAL, MAX_BEACON_SIGNAL_RANGE);
            m_vecBehaviors.push_back(pcHomingToFoodBeaconBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
            m_vecBehaviors.push_back(pcRandomWalkBehavior);
        }

        // Homing disabled as the beacon signal data will interfere with the FV data
        //exit(-1);
    }
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::Reset()
{
    /* Set LED color */
    m_pcLEDs->SetAllColors(CColor::WHITE);
    m_pcRABA->ClearData();
}

/****************************************/
/****************************************/

unsigned CEPuckHomSwarm::RobotIdStrToInt()
{
    std::string id = GetId();

    if(id.compare("ep0")==0)
        return 0;

    else if(id.compare("ep1")==0)
        return 1;

    else if(id.compare("ep2")==0)
        return 2;

    else if(id.compare("ep3")==0)
        return 3;

    else if(id.compare("ep4")==0)
        return 4;

    else if(id.compare("ep5")==0)
        return 5;

    else if(id.compare("ep6")==0)
        return 6;

    else if(id.compare("ep7")==0)
        return 7;

    else if(id.compare("ep8")==0)
        return 8;

    else if(id.compare("ep9")==0)
        return 9;

    else if(id.compare("ep10")==0)
        return 10;

    else if(id.compare("ep11")==0)
        return 11;

    else if(id.compare("ep12")==0)
        return 12;

    else if(id.compare("ep13")==0)
        return 13;

    else if(id.compare("ep14")==0)
        return 14;

    else if(id.compare("ep15")==0)
        return 15;

    else if(id.compare("ep16")==0)
        return 16;

    else if(id.compare("ep17")==0)
        return 17;

    else if(id.compare("ep18")==0)
        return 18;

    else if(id.compare("ep19")==0)
        return 19;

    else
        LOGERR << "We can't be here, there's a bug!" << std::endl;
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::WriteToCommunicationChannel(unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay)
{
    size_t databyte_index;

    if (m_pcRABA->GetData(0) == BEACON_SIGNAL)
        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 onwards
        databyte_index = 1;
    else
        databyte_index = 0;


    m_pcRABA->SetData(databyte_index++, OBSERVED_FVS_PACKET);
    m_pcRABA->SetData(databyte_index++, SelfId);
    m_pcRABA->SetData(databyte_index++, SelfFV);

    bool buffer_full(false);
    for(t_listMapFVsToRobotIds::iterator itd = IdToFVsMap_torelay.begin(); itd != IdToFVsMap_torelay.end(); ++itd)
    {
        m_pcRABA->SetData(databyte_index++, itd->uRobotId);
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            buffer_full = true;
            m_pcRABA->SetData(databyte_index, END_BUFFER);
            break;
        }

        m_pcRABA->SetData(databyte_index++, itd->uFV);
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            buffer_full = true;
            m_pcRABA->SetData(databyte_index, END_BUFFER);
            break;
        }

    }
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::WriteToCommunicationChannel(unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds,
                                                 t_listFVsSensed& CRMResultsOnFVDist, t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid)
{
    size_t databyte_index;

    if (m_pcRABA->GetData(0) == BEACON_SIGNAL)
        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 onwards
        databyte_index = 1;
    else
        databyte_index = 0;

    m_pcRABA->SetData(databyte_index++, VOTER_PACKET);
    m_pcRABA->SetData(databyte_index++, VoterId);

    bool buffer_full(false);
    /*
     * Write the consensus list to the channel comprising < .... <robot id, its consensus state> ... >
     */
    for (t_listConsensusInfoOnRobotIds::iterator it_cons = ConsensusLst.begin(); it_cons != ConsensusLst.end(); ++it_cons)
    {

        m_pcRABA->SetData(databyte_index++, it_cons->uRobotId);
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            buffer_full = true;
            m_pcRABA->SetData(databyte_index, END_BUFFER);
            break;
        }

        m_pcRABA->SetData(databyte_index++, (it_cons->consensus_state==1)?ATTACK_CONSENSUS:TOLERATE_CONSENSUS);
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            buffer_full = true;
            m_pcRABA->SetData(databyte_index, END_BUFFER);
            break;
        }
    }

    if(buffer_full)
    {
        std::cerr << " No longer able to write the vote packet. complain by exiting";
        exit(-1);
    }

    if(!b_CRM_Results_Valid) /* the crm results on the FVs in CRMResultsOnFVDist is no longer valid */
        return;

    /*
     * Write the results of CRM to the channel comprising < .... <fv, attack/tolerate state> ... >
     * We dont write the results of all the FVs in the listFVsSensed as they may be some very old FVs no longer present in the swarm.
     * Update: Your FV-ID map may be old too. Other robots may have a better map. Don't curtail information from them.
     */
    for (t_listFVsSensed::iterator it_fvdist = CRMResultsOnFVDist.begin(); it_fvdist != CRMResultsOnFVDist.end(); ++it_fvdist)
    {
//        for(t_listMapFVsToRobotIds::iterator it_map = MapFVsToRobotIds.begin(); it_map != MapFVsToRobotIds.end(); ++it_map)
//        {
//            if(it_fvdist->uFV == it_map->uFV)
//            {
                m_pcRABA->SetData(databyte_index++, it_fvdist->uFV);
                if(databyte_index == m_pcRABA->GetSize()-1)
                {
                    buffer_full = true;
                    m_pcRABA->SetData(databyte_index, END_BUFFER);
                    break;
                }

                m_pcRABA->SetData(databyte_index++, (it_fvdist->uMostWantedState==1)?ATTACK_VOTE:TOLERATE_VOTE);
                if(databyte_index == m_pcRABA->GetSize()-1)
                {
                    buffer_full = true;
                    m_pcRABA->SetData(databyte_index, END_BUFFER);
                    break;
                }

//                break; // only one vote is cast per fv
//            }
//        }

        if(buffer_full)
        {
            std::cerr << " No longer able to write the rest of the vote packets. complain by exiting";
            exit(-1);
        }
    }
}

/****************************************/
/****************************************/

bool func_SortPacketsOnRange (const CCI_RangeAndBearingSensor::SPacket i, const CCI_RangeAndBearingSensor::SPacket j) { return (i.Range < j.Range); }

/****************************************/

bool  CEPuckHomSwarm::ReadFromCommunicationChannel_IdFv(const CCI_RangeAndBearingSensor::TReadings& tPackets)
{
    /* Read the id and proprioceptively computed FV of your neighbours. Read from communication channel and stored in listMapFVsToRobotIds */
    /* Mark these read FVs as to be relayed. Stored separately in listMapFVsToRobotIds_relay */
    /* Also read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */

    bool read_successful(false); // successfully read id and fvs from at least one neighbour

    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations

     // Adding the most recent observations into listMapFVsToRobotIds and marked for relay in listMapFVsToRobotIds_relay
    listMapFVsToRobotIds_relay.clear();
    for(size_t i = 0; i < tPackets.size(); ++i)
    {
        size_t byte_index = 0; unsigned robotId, fv;

        if(tPackets[i].Data[0] == BEACON_SIGNAL) // data from a beacon  - get the next two bytes
            byte_index = 1;
        else
            byte_index = 0;


        if(tPackets[i].Data[byte_index++] != OBSERVED_FVS_PACKET) // this neighbour is not sending me observed FVs
            continue;

        robotId = tPackets[i].Data[byte_index++];
        fv      = tPackets[i].Data[byte_index++];
        read_successful = true;

        listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv));
        UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer);


        for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
        {
            if(tPackets[i].Data[byteindex1] == END_BUFFER)
                break;

            robotId = tPackets[i].Data[byteindex1];

            if(tPackets[i].Data[byteindex1+1] == END_BUFFER)
                break;

            fv      = tPackets[i].Data[byteindex1+1];

            UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer-1); // old information
        }

    }

    return read_successful;
}

/****************************************/
/****************************************/

bool  CEPuckHomSwarm::ReadFromCommunicationChannel_VotCon(const CCI_RangeAndBearingSensor::TReadings& tPackets)
{

    /* Listen to votes and consensus from neighbours */
    /* Read the voter id:
     * Followed by <fv, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
     * If a vote is received,
     *                      1. map the fv to the robot id (if none existed - ignore vote???)
     *                      2. if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search.
     *                      3. if voter id has voted on mapped robot id before, ignore vote. But remember, the voted fv may map to other robot ids so continue search.
     *
     * If a consensus is received,
     *                      1. update listConsensusInfoOnRobotIds to include id and ATTACK_CONSENSUS / TOLERATE_CONSENSUS
     *                      2. if listConsensusInfoOnRobotIds already has id with a different CONSENSUS from the received message - problem with discrepancy in consensus???? Could except consensus from lowest voter id as true???
     *
    */

    bool read_successful(false); // successfully read votes / consensus from at least one neighbour


    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations


    for(size_t i = 0; i < tPackets.size(); ++i)
    {
        size_t byte_index = 0;
        unsigned votertId, fv, attack_tolerate_vote, ConsensusOnRobotId, ConsensusState; unsigned tmp1, tmp2;

        if(tPackets[i].Data[0] == BEACON_SIGNAL) // data from a beacon  - get the next two bytes
            byte_index = 1;
        else
            byte_index = 0;


        if(tPackets[i].Data[byte_index++] != VOTER_PACKET) // this neighbour is not sending me observed FVs
            continue;

        votertId = tPackets[i].Data[byte_index++];

        for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
        {
            if(tPackets[i].Data[byteindex1] == END_BUFFER)
                break;

            tmp1 = tPackets[i].Data[byteindex1];

            if(tPackets[i].Data[byteindex1+1] == END_BUFFER)
                break;

            tmp2 = tPackets[i].Data[byteindex1+1];


            if(tmp2 == ATTACK_VOTE || tmp2 == TOLERATE_VOTE)
            {
                fv                   = tmp1;
                attack_tolerate_vote = (tmp2==ATTACK_VOTE)?1u:2u;

                UpdateVoterRegistry(listVoteInformationRobots,
                                    listMapFVsToRobotIds,
                                    listConsensusInfoOnRobotIds,
                                    votertId, fv, attack_tolerate_vote);
            }
            else
            {
                assert(tmp2 == ATTACK_CONSENSUS || ConsensusState == TOLERATE_CONSENSUS);
                ConsensusOnRobotId = tmp1;
                ConsensusState     = (tmp2==ATTACK_CONSENSUS)?1u:2u;

                bool b_ConsensusAlreadyEstablishedOnRobot(false);
                for (t_listConsensusInfoOnRobotIds::iterator it_cons = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
                {
                    if(it_cons->uRobotId == ConsensusOnRobotId)
                    {
                        b_ConsensusAlreadyEstablishedOnRobot = true;
                        if (ConsensusState != it_cons->consensus_state)
                        {
                            // Difference in consensus state. there is a disparity in our consensus ????
                            // We can correct state, assuming the lowest id (between my id and voter id) is correct?
                        }
                        break; // robot ids are unique in the consensus list
                    }
                }

                if(!b_ConsensusAlreadyEstablishedOnRobot)
                    listConsensusInfoOnRobotIds.push_back(ConsensusInformationRobots(ConsensusOnRobotId, ConsensusState));
            }

            read_successful = true;
        }

    }

    return read_successful;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckHomSwarm, "epuck_homswarm_controller")
