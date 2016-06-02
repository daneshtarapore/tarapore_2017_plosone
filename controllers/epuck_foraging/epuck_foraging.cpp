/* Include the controller definition */
#include "epuck_foraging.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

//#define DATA_BYTE_BOUND 240.0f
//UInt8 CEPuckForaging::BEACON_SIGNAL = 241;
//UInt8 CEPuckForaging::NEST_BEACON_SIGNAL = 242;


//#define SELF_INFO_PACKET 243 /* used to encompass info of self, be that the proprioceptively computed FVs, the bearings at which neighbours are observed, or proprioceptively computed angular acceleration.*/
//#define SELF_INFO_PACKET_FOOTER 244

//#define RELAY_FVS_PACKET 245
//#define RELAY_FVS_PACKET_FOOTER 246

//#define VOTER_PACKET 247
//#define ATTACK_VOTE 248
//#define TOLERATE_VOTE 249
//#define ATTACK_CONSENSUS 250
//#define TOLERATE_CONSENSUS 251
//#define VOTER_PACKET_FOOTER 252


//#define PROPRIOCEPT_MODE 0
//#define OBSERVATION_MODE 1
//#define COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE 2
//#define BAYESIANINFERENCE_MODE 3
//#define FV_MODE BAYESIANINFERENCE_MODE

///****************************************/
///****************************************/

///*
// * Probability to forget FV in distribution
// */
//#define PROBABILITY_FORGET_FV 1.0f //0.001f // We don't need a large history because FVs are being relayed every time-step. That combined with the BI of FVs (small observation window) means that robots will have sufficient FVs of neighbours to make a decision. Also it is difficult to assume that robot behavior has not changed in the last 100s.Will have a CRM_RESULTS_VALIDFOR_SECONDS history instead.

///*
// * Consensus threshold on FVs.
// */
//#define CONSENSUS_THRESHOLD 5u /* odd number so that we don't have a tie in attackers and tolerators - but this is just a threshold. number of voters may be more than threshold */

///*
// * The results of the CRM are valid for atmost 10s in the absence of any FVs to run the CRM
// */
//#define CRM_RESULTS_VALIDFOR_SECONDS 10.0f // in seconds // assume the robot behaviors have not changed in the last 10s

///*
// * The vote counts and consensus are valid for atmost 10s before being refreshed
// */
//#define VOTCON_RESULTS_VALIDFOR_SECONDS 10.0f // in seconds //1.0f

/****************************************/
/****************************************/

CBehavior::SensoryData CBehavior::m_sSensoryData;
CBehavior::RobotData CBehavior::m_sRobotData;

CProprioceptiveFeatureVector::RobotData CProprioceptiveFeatureVector::m_sRobotData;

CObservedFeatureVector::RobotData CObservedFeatureVector::m_sRobotData;

CBayesianInferenceFeatureVector::RobotData CBayesianInferenceFeatureVector::m_sRobotData;

/****************************************/
/****************************************/

CEPuckForaging::ExperimentToRun::ExperimentToRun() :
    SBehavior(SWARM_FORAGING),
    FBehavior(FAULT_NONE),
    id_FaultyRobotInSwarm("-1") {}


void CEPuckForaging::ExperimentToRun::Init(TConfigurationNode& t_node)
{
    std::string errorbehav;

    try
    {
        GetNodeAttribute(t_node, "swarm_behavior", swarmbehav);
        GetNodeAttribute(t_node, "fault_behavior", errorbehav);
        GetNodeAttribute(t_node, "id_faulty_robot", id_FaultyRobotInSwarm);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);

    if (swarmbehav.compare("SWARM_FORAGING") == 0)
        SBehavior = SWARM_FORAGING;
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


    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMIN") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMIN;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMAX") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMAX;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETRANDOM") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETRANDOM;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETOFFSET") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETOFFSET;


    else if  (errorbehav.compare("FAULT_RABSENSOR_SETOFFSET") == 0)
        FBehavior = FAULT_RABSENSOR_SETOFFSET;
    else if  (errorbehav.compare("FAULT_RABSENSOR_MISSINGRECEIVERS") == 0)
        FBehavior = FAULT_RABSENSOR_MISSINGRECEIVERS;


    else if  (errorbehav.compare("FAULT_ACTUATOR_LWHEEL_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_LWHEEL_SETZERO;
    else if  (errorbehav.compare("FAULT_ACTUATOR_RWHEEL_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_RWHEEL_SETZERO;
    else if  (errorbehav.compare("FAULT_ACTUATOR_BWHEELS_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_BWHEELS_SETZERO;

    else if  (errorbehav.compare("FAULT_SOFTWARE") == 0)
        FBehavior = FAULT_SOFTWARE;

    else if  (errorbehav.compare("FAULT_POWER_FAILURE") == 0)
        FBehavior = FAULT_POWER_FAILURE;

    else
    {
        std::cerr << "invalid fault behavior";
        assert(-1);
    }
}

/****************************************/
/****************************************/

CEPuckForaging::SFoodData::SFoodData() :
    HasFoodItem(false),
    TotalFoodItems(0)
{
}

void CEPuckForaging::SFoodData::Reset()
{
    HasFoodItem = false;
    TotalFoodItems = 0;
}

/****************************************/
/****************************************/

void CEPuckForaging::SWheelTurningParams::Init(TConfigurationNode& t_node)
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

CEPuckForaging::SStateData::SStateData() :
    ProbRange(0.0f, 1.0f)
{}

void CEPuckForaging::SStateData::Init(TConfigurationNode& t_node)
{
    try
    {
        GetNodeAttribute(t_node, "initial_rest_to_explore_prob", InitialRestToExploreProb);
        GetNodeAttribute(t_node, "minimum_resting_time", MinimumRestingTime);
        GetNodeAttribute(t_node, "minimum_unsuccessful_explore_time", MinimumUnsuccessfulExploreTime);
        GetNodeAttribute(t_node, "minimum_search_for_place_in_nest_time", MinimumSearchForPlaceInNestTime);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
}

void CEPuckForaging::SStateData::Reset()
{
    State = STATE_RESTING;
    InNest = true;
    OnFood = false;
    TimeExploringUnsuccessfully = 0;
    /* Initially the robot is resting, and by setting RestingTime to
      MinimumRestingTime we force the robots to make a decision at the
      experiment start. If instead we set RestingTime to zero, we would
      have to wait till RestingTime reaches MinimumRestingTime before
      something happens, which is just a waste of time. */
    TimeRested = MinimumRestingTime;
    TimeSearchingForPlaceInNest = 0;
}

/****************************************/
/****************************************/

CEPuckForaging::CEPuckForaging() :
    m_fInternalRobotTimer(0.0f),
    m_pcWheels(NULL),
    m_pcLEDs(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcProximity(NULL),
    m_pcLight(NULL),
    m_pcGround(NULL),
    m_pcRNG(CRandom::CreateRNG("argos")),
    m_pcRNG_FVs(CRandom::CreateRNG("argos")),
    b_damagedrobot(false),
    u_num_consequtivecollisions(0)
{
#ifndef DESYNC_ROB_CLOCK
    m_fRobotTimerAtStart = 0.0f;
#else
    // desync clocks by +/-5 s - Gaussian dist
    m_fRobotTimerAtStart = m_pcRNG->Gaussian(25.0f, 50.0f); // mean 50 ticsk, std dev. 10 ticks (50 ticks = 5 sec)
    if(m_fRobotTimerAtStart < 0.0f)
        m_fRobotTimerAtStart = 0.0f;
    else if(m_fRobotTimerAtStart > 100.0f)
        m_fRobotTimerAtStart = 100.0f;
    else
        m_fRobotTimerAtStart = (unsigned) m_fRobotTimerAtStart;
#endif

    m_fInternalRobotTimer = m_fRobotTimerAtStart;

    listFVsSensed.clear();
    listMapFVsToRobotIds.clear();
    listMapFVsToRobotIds_relay.clear();
    listConsensusInfoOnRobotIds.clear();
    listVoteInformationRobots.clear();

    b_CRM_Run = false;
    m_fCRM_RUN_TIMESTAMP = 0.0f;
    m_uRobotFV = 9999; // for debugging urposes


    TIME_STATE_RESTING = 0u; TIME_STATE_EXPLORING = 0u; TIME_STATE_BEACON = 0u;
    TIME_STATE_RESTING_AT_FOOD = 0u;  TIME_STATE_RETURN_TO_NEST = 0u;

    TIME_STATE_RESTING1 = 0u; TIME_STATE_EXPLORING1 = 0u; TIME_STATE_BEACON1 = 0u;
    TIME_STATE_RESTING_AT_FOOD1 = 0u;  TIME_STATE_RETURN_TO_NEST1 = 0u;

    TIME_STATE_RESTING2 = 0u; TIME_STATE_EXPLORING2 = 0u; TIME_STATE_BEACON2 = 0u;
    TIME_STATE_RESTING_AT_FOOD2 = 0u;  TIME_STATE_RETURN_TO_NEST2 = 0u;
}

/****************************************/
/****************************************/

void CEPuckForaging::Init(TConfigurationNode& t_node)
{
    try
    {
        /*
       * Initialize sensors/actuators
       */
        m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcWheelsEncoder = GetSensor  <CCI_DifferentialSteeringSensor  >("differential_steering");
        m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
        m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
        m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
        m_pcProximity = GetSensor  <CCI_EPuckProximitySensor        >("epuck_proximity"    );
        m_pcLight     = GetSensor  <CCI_LightUpdatedSensor          >("light_updated"        );
        m_pcGround    = GetSensor  <CCI_GroundSensor                >("ground" );
        /*
       * Parse XML parameters
       */
        /* Experiment to run */
        m_sExpRun.Init(GetNode(t_node, "experiment_run"));
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
        /* Controller state */
        m_sStateData.Init(GetNode(t_node, "state"));
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing the e-puck foraging controller for robot \"" << GetId() << "\"", ex);

    /*
    * Initialize other stuff
    */
    /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
    // Now initialised at CEPuckForaging() constructor
    /*m_pcRNG = CRandom::CreateRNG("argos");
    m_pcRNG_FVs = CRandom::CreateRNG("argos");*/
    Reset();

    m_sRobotDetails.SetKinematicDetails(m_sWheelTurningParams.MaxSpeed, m_sWheelTurningParams.MaxSpeed);

    CopyRobotDetails(m_sRobotDetails);

    if(this->GetId().compare("ep"+m_sExpRun.id_FaultyRobotInSwarm) == 0)
        b_damagedrobot = true;

    // robotid set to 0 for now
    crminAgent = new CRMinRobotAgentOptimised(RobotIdStrToInt(), CProprioceptiveFeatureVector::NUMBER_OF_FEATURES);
}


/****************************************/
/****************************************/

void CEPuckForaging::CopyRobotDetails(RobotDetails& robdetails)
{
    CBehavior::m_sRobotData.MaxSpeed                    = robdetails.MaxLinearSpeed * robdetails.iterations_per_second; // max speed in cm/s to control behavior
    CBehavior::m_sRobotData.iterations_per_second       = robdetails.iterations_per_second;
    CBehavior::m_sRobotData.seconds_per_iterations      = 1.0f / robdetails.iterations_per_second;
    CBehavior::m_sRobotData.HALF_INTERWHEEL_DISTANCE    = robdetails.HALF_INTERWHEEL_DISTANCE;
    CBehavior::m_sRobotData.INTERWHEEL_DISTANCE         = robdetails.INTERWHEEL_DISTANCE;
    CBehavior::m_sRobotData.WHEEL_RADIUS                = robdetails.WHEEL_RADIUS;

    CBehavior::m_sRobotData.m_cNoTurnOnAngleThreshold   = robdetails.m_cNoTurnOnAngleThreshold;
    CBehavior::m_sRobotData.m_cSoftTurnOnAngleThreshold = robdetails.m_cSoftTurnOnAngleThreshold;

    CBehavior::m_sRobotData.BEACON_SIGNAL_MARKER           = BEACON_SIGNAL;
    CBehavior::m_sRobotData.NEST_BEACON_SIGNAL_MARKER      = NEST_BEACON_SIGNAL;
    CBehavior::m_sRobotData.SELF_INFO_PACKET_MARKER        = SELF_INFO_PACKET;
    CBehavior::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER = SELF_INFO_PACKET_FOOTER;
    CBehavior::m_sRobotData.RELAY_FVS_PACKET_MARKER        = RELAY_FVS_PACKET;
    CBehavior::m_sRobotData.RELAY_FVS_PACKET_FOOTER_MARKER = RELAY_FVS_PACKET_FOOTER;
    CBehavior::m_sRobotData.VOTER_PACKET_MARKER            = VOTER_PACKET;
    CBehavior::m_sRobotData.VOTER_PACKET_FOOTER_MARKER     = VOTER_PACKET_FOOTER;
    CBehavior::m_sRobotData.DATA_BYTE_BOUND_MARKER         = DATA_BYTE_BOUND;
    CBehavior::m_sRobotData.OBSERVATION_MODE_TYPE          = FV_MODE;




    CProprioceptiveFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CProprioceptiveFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CProprioceptiveFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CProprioceptiveFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CProprioceptiveFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;



    CObservedFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CObservedFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CObservedFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CObservedFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CObservedFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CObservedFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CObservedFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CObservedFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CObservedFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;



    CObservedFeatureVector::m_sRobotData.BEACON_SIGNAL_MARKER           = BEACON_SIGNAL;
    CObservedFeatureVector::m_sRobotData.NEST_BEACON_SIGNAL_MARKER      = NEST_BEACON_SIGNAL;
    CObservedFeatureVector::m_sRobotData.SELF_INFO_PACKET_MARKER        = SELF_INFO_PACKET;
    CObservedFeatureVector::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER = SELF_INFO_PACKET_FOOTER;
    CObservedFeatureVector::m_sRobotData.RELAY_FVS_PACKET_MARKER        = RELAY_FVS_PACKET;
    CObservedFeatureVector::m_sRobotData.RELAY_FVS_PACKET_FOOTER_MARKER = RELAY_FVS_PACKET_FOOTER;
    CObservedFeatureVector::m_sRobotData.VOTER_PACKET_MARKER            = VOTER_PACKET;
    CObservedFeatureVector::m_sRobotData.VOTER_PACKET_FOOTER_MARKER     = VOTER_PACKET_FOOTER;
    CObservedFeatureVector::m_sRobotData.DATA_BYTE_BOUND_MARKER         = DATA_BYTE_BOUND;
    CObservedFeatureVector::m_sRobotData.OBSERVATION_MODE_TYPE          = FV_MODE;




    CBayesianInferenceFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CBayesianInferenceFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CBayesianInferenceFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CBayesianInferenceFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CBayesianInferenceFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;

    CBayesianInferenceFeatureVector::m_sRobotData.SetLengthOdometryTimeWindows();

    CBayesianInferenceFeatureVector::m_sRobotData.BEACON_SIGNAL_MARKER           = BEACON_SIGNAL;
    CBayesianInferenceFeatureVector::m_sRobotData.NEST_BEACON_SIGNAL_MARKER           = NEST_BEACON_SIGNAL;
    CBayesianInferenceFeatureVector::m_sRobotData.SELF_INFO_PACKET_MARKER        = SELF_INFO_PACKET;
    CBayesianInferenceFeatureVector::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER = SELF_INFO_PACKET_FOOTER;
    CBayesianInferenceFeatureVector::m_sRobotData.RELAY_FVS_PACKET_MARKER        = RELAY_FVS_PACKET;
    CBayesianInferenceFeatureVector::m_sRobotData.RELAY_FVS_PACKET_FOOTER_MARKER = RELAY_FVS_PACKET_FOOTER;
    CBayesianInferenceFeatureVector::m_sRobotData.VOTER_PACKET_MARKER            = VOTER_PACKET;
    CBayesianInferenceFeatureVector::m_sRobotData.VOTER_PACKET_FOOTER_MARKER     = VOTER_PACKET_FOOTER;
    CBayesianInferenceFeatureVector::m_sRobotData.DATA_BYTE_BOUND_MARKER         = DATA_BYTE_BOUND;
    CBayesianInferenceFeatureVector::m_sRobotData.OBSERVATION_MODE_TYPE          = FV_MODE;
}

/****************************************/
/****************************************/

unsigned CEPuckForaging::SumFVDist(t_listFVsSensed& FVsSensed)
{
    unsigned robotcount = 0;
    for (t_listFVsSensed::iterator it = FVsSensed.begin(); it != FVsSensed.end(); ++it)
        robotcount += it->fRobots;

    return robotcount;
}

/****************************************/
/****************************************/

void CEPuckForaging::ControlStep()
{
    m_pcRABA->ClearData(); // clear the channel at the start of each control cycle

    m_fInternalRobotTimer += 1.0f;

    if(m_fInternalRobotTimer == 10000.0f)
    {
        TIME_STATE_RESTING1         = TIME_STATE_RESTING;
        TIME_STATE_EXPLORING1       = TIME_STATE_EXPLORING;
        TIME_STATE_BEACON1          = TIME_STATE_BEACON;
        TIME_STATE_RESTING_AT_FOOD1 = TIME_STATE_RESTING_AT_FOOD;
        TIME_STATE_RETURN_TO_NEST1  = TIME_STATE_RETURN_TO_NEST;

        TIME_STATE_RESTING = 0u; TIME_STATE_EXPLORING = 0u; TIME_STATE_BEACON = 0u;
        TIME_STATE_RESTING_AT_FOOD = 0u;  TIME_STATE_RETURN_TO_NEST = 0u;
    }
    else if(m_fInternalRobotTimer == 20000.0f)
    {
        TIME_STATE_RESTING2         = TIME_STATE_RESTING;
        TIME_STATE_EXPLORING2       = TIME_STATE_EXPLORING;
        TIME_STATE_BEACON2          = TIME_STATE_BEACON;
        TIME_STATE_RESTING_AT_FOOD2 = TIME_STATE_RESTING_AT_FOOD;
        TIME_STATE_RETURN_TO_NEST2  = TIME_STATE_RETURN_TO_NEST;

        TIME_STATE_RESTING = 0u; TIME_STATE_EXPLORING = 0u; TIME_STATE_BEACON = 0u;
        TIME_STATE_RESTING_AT_FOOD = 0u;  TIME_STATE_RETURN_TO_NEST = 0u;
    }

    bool b_RunningGeneralFaults(false);
    if(b_damagedrobot && (m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_CIRCLE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_STOP))
    {
        b_RunningGeneralFaults = true;
        RunGeneralFaults();
    }


    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_FORAGING)
        RunForagingExperiment();


    if(!b_damagedrobot || b_RunningGeneralFaults || m_sExpRun.FBehavior == ExperimentToRun::FAULT_NONE)
        CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
    else
    {
        //m_pcLEDs->SetAllColors(CColor::RED);

        if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETMIN)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETMAX)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETRANDOM)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETOFFSET)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_RABSENSOR_SETOFFSET)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_RABSENSOR_MISSINGRECEIVERS)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_LWHEEL_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_RWHEEL_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_BWHEELS_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_SOFTWARE)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));

        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_POWER_FAILURE)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
    }

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

    /*If robot is contantly colliding against a wall, half the speed at which the wheels rotate - to make the robot movement closer to reality. We use the IR sensors to detect this scenario and we can do this even when there is a sensor fault as in reality the speed would reduce on its own when the robot is stuck to a wall*/
    /*Using the noiseless variant of the IR sensors for this detection*/

    if(m_pcProximity->GetReadings_Noiseless()[0].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[1].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[2].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[3].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[4].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[5].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[6].Value > 0.4f ||
       m_pcProximity->GetReadings_Noiseless()[7].Value > 0.4f)
        u_num_consequtivecollisions++;
    else
        u_num_consequtivecollisions = 0u;


    // if the robot is colliding with the wall other robot for more than 1s, we reduce its speed by half
    /* this will be harder to detect when we add noise on the IR sensors. Be wary of that. So using the noiseless variant of the IR sensors for this detection*/
    if((Real)u_num_consequtivecollisions > (m_sRobotDetails.iterations_per_second * 1.0f))
    {
        leftSpeed = leftSpeed/2.0f;
        rightSpeed = rightSpeed/2.0f;
    }

    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_LWHEEL_SETZERO)
        leftSpeed  = 0.0f;

    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_RWHEEL_SETZERO)
        rightSpeed = 0.0f;

    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_BWHEELS_SETZERO)
    {
        leftSpeed = 0.0f;
        rightSpeed = 0.0f;
    }

    m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed); // in cm/s

    //std::cout << "LS:  " << leftSpeed << " RS:  " << rightSpeed << std::endl;

    CCI_RangeAndBearingSensor::TReadings rabsensor_readings = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);

    m_uRobotId = RobotIdStrToInt();
    SenseCommunicateDetect(RobotIdStrToInt(), m_pcRABA, m_pcWheelsEncoder,
                           m_fInternalRobotTimer, rabsensor_readings,
                           listMapFVsToRobotIds, listMapFVsToRobotIds_relay, listFVsSensed, listVoteInformationRobots, listConsensusInfoOnRobotIds,
                           m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
                           b_CRM_Run, m_fCRM_RUN_TIMESTAMP, crminAgent, m_pcRNG_FVs, m_uRobotFV, m_sExpRun.swarmbehav, beaconrobots_ids);




//    /****************************************/
//#if FV_MODE == PROPRIOCEPT_MODE

//    /* Estimate feature-vectors - proprioceptively */

//    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                                 m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);
//    m_cProprioceptiveFeatureVector.SimulationStep();

//    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue(); // to debug
//    m_uRobotId = RobotIdStrToInt();

//    /* Communicate your id and proprioceptively computed FV to whoever is in range, using the RAB sensor*/
//    /* Also relay the id and fvs of neighbours, received by you in the previous control cycle */
//    //if ((unsigned)m_fInternalRobotTimer%2u == 0)

//    //printf(" SendFVsToNeighbours() \n\n\n");
//    SendFVsToNeighbours();

//    /* Listen for robot ids + feature vectors from neighbours and then assimilate them  */
//    //printf(" Sense(PROBABILITY_FORGET_FV); \n\n\n");
//    Sense(PROBABILITY_FORGET_FV);
//#endif
//    /****************************************/



//    /****************************************/
//#if FV_MODE == OBSERVATION_MODE || FV_MODE == COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE

//    /*if(RobotIdStrToInt() == 0)
//    {
//        std::cerr << "L " << RobotIdStrToInt() << "enc "  << m_pcWheelsEncoder->GetReading().VelocityLeftWheel  << " controller " << leftSpeed   << std::endl;
//        std::cerr << "R " << RobotIdStrToInt() << "enc " << m_pcWheelsEncoder->GetReading().VelocityRightWheel << " controller " <<  rightSpeed  << std::endl;
//    }*/



//    /* Estimating FVs proprioceptively - to be used for the simplifying fault detection and to compute angular acceleration for the COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE */
//    /*m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                                 leftSpeed, rightSpeed);*/
//    /*encoders give you the speed at the previous tick not current tick */
//    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                                 m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);


//    m_cProprioceptiveFeatureVector.SimulationStep();
//    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue();

//    /* Estimate feature-vectors - via observation */
//    m_uRobotId = RobotIdStrToInt();


//    /*m_cObservationFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                              leftSpeed, rightSpeed);*/
//    /*encoders give you the speed at the previous tick not current tick */
//    m_cObservationFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                              m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);
//    m_cObservationFeatureVector.SimulationStep();

//    Sense(PROBABILITY_FORGET_FV);

//    //if ( ((unsigned)m_fInternalRobotTimer%2u == 0) || (m_fInternalRobotTimer <= MODELSTARTTIME))
//    /*
//     * Send the robot id and the bearing at which it observes its different neighbours. Also relay the observed FVs
//     */

//    SendIdSelfBearingAndObsFVsToNeighbours(GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), listMapFVsToRobotIds_relay);
//#endif
//    /****************************************/

//    /****************************************/
//#if FV_MODE == BAYESIANINFERENCE_MODE

//    /* Estimating FVs proprioceptively - to be used for computing angular acceleration*/
//    /*encoders give you the speed at the previous tick not current tick */
//    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                                 m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);


//    m_cProprioceptiveFeatureVector.SimulationStep();
//    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue();

//    /* Estimate feature-vectors - via observation */
//    m_uRobotId = RobotIdStrToInt();


//    /*encoders give you the speed at the previous tick not current tick */
//    m_cBayesianInferredFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
//                                                              m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);
//    m_cBayesianInferredFeatureVector.SimulationStep();

//    Sense(PROBABILITY_FORGET_FV);

//    SendIdSelfBearingAndObsFVsToNeighbours(GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), listMapFVsToRobotIds_relay);
//#endif

//    /****************************************/

//    /*if(m_fInternalRobotTimer >= 2500.0f && m_fInternalRobotTimer <= 3000u && RobotIdStrToInt() == 0)
//    {
//        for(t_listMapFVsToRobotIds::iterator it_fv = listMapFVsToRobotIds.begin(); it_fv != listMapFVsToRobotIds.end(); ++it_fv)
//        {
//            std::cout << "Map: id " << it_fv->uRobotId  << " to fv " << it_fv->uFV  << " time sensed " << it_fv->fTimeSensed << std::endl << std::endl;
//        }
//    }*/



//    if(((unsigned)m_fInternalRobotTimer % (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) == 0u)
//        // to avoid consensus already in the medium to establish itself in the next step. when the robot clocks are not in sync, this period would have to be longer than just 2 iterations
//    {
//        listConsensusInfoOnRobotIds.clear();
//        listVoteInformationRobots.clear();
//    }
//    else if(m_fInternalRobotTimer > MODELSTARTTIME)
//        /* else because you don't want to receive consensus already in the medium from before the buffer was cleared*/
//    {
//        /* Listen for voting packets and consensus packets from neighbours*/
//        //printf(" ReceiveVotesAndConsensus(); \n\n\n");
//        ReceiveVotesAndConsensus();
//        //printf(" Finished ReceiveVotesAndConsensus(); \n\n\n");
//        EstablishConsensus();
//    }


//    Real TimeSinceCRM = (m_fInternalRobotTimer - m_fCRM_RUN_TIMESTAMP) * CProprioceptiveFeatureVector::m_sRobotData.seconds_per_iterations; // in seconds
//    if (b_CRM_Run && (TimeSinceCRM > CRM_RESULTS_VALIDFOR_SECONDS)) /* the results of the CRM are no longer valid */
//        b_CRM_Run = false;

//    if((m_fInternalRobotTimer > MODELSTARTTIME) && (listFVsSensed.size() > 0))
//        // the robot has atleast had one FV entry in its distribution. if not the CRM will crash.
//    {
//        crminAgent->SimulationStepUpdatePosition(m_fInternalRobotTimer, &listFVsSensed);
//        b_CRM_Run = true;
//        m_fCRM_RUN_TIMESTAMP = m_fInternalRobotTimer;
//    }

//    if(b_CRM_Run) // a failsafe to make sure you don't use outdated CRM results
//    {
//        // the CRM results on FVs in listFVsSensed is not outdated
//        for(t_listFVsSensed::iterator it_fv = listFVsSensed.begin(); it_fv != listFVsSensed.end(); ++it_fv)
//        {
//            UpdateVoterRegistry(listVoteInformationRobots,
//                                listMapFVsToRobotIds,
//                                listConsensusInfoOnRobotIds,
//                                RobotIdStrToInt(), it_fv->uFV, it_fv->uMostWantedState);
//        }
//    }

//    if(((unsigned)m_fInternalRobotTimer % (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) != 0u) /* dont send CRM results if buffer is cleared*/
//    {
//        if ((m_fInternalRobotTimer > MODELSTARTTIME)) // && (unsigned)m_fInternalRobotTimer%2u == 1)
//        {
//            //printf(" SendCRMResultsAndConsensusToNeighbours(b_CRM_Run); \n\n\n");
//            SendCRMResultsAndConsensusToNeighbours(b_CRM_Run); // only send CRM results if they are valid
//            //printf(" Finished SendCRMResultsAndConsensusToNeighbours(b_CRM_Run); \n\n\n");
//        }
//    }
//    /*else
//        SendIdSelfBearingAndObsFVsToNeighbours(GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior)); */ /* we need to send the robot id on the range and bearing sensors all the time - as gaps in data reception are not programmed for */
}

/****************************************/
/****************************************/

//void CEPuckForaging::SendIdSelfBearingAndObsFVsToNeighbours(const CCI_RangeAndBearingSensor::TReadings& tPackets, t_listMapFVsToRobotIds &IdToFVsMap_torelay)
//{
//    /*Communicate your id to neighbours, so they know who they are observing*/
//    /*Also communicate the bearing at which you observed the neighbours */
//    /*Also communicate the FVs you have observed */
//    WriteToCommunicationChannel(RobotIdStrToInt(), tPackets, IdToFVsMap_torelay);
//}

/****************************************/
/****************************************/

//void CEPuckForaging::SendFVsToNeighbours()
//{
//    /*Communicate your id and FV, and relay the id and fvs of neighbours, received by you in the previous control cycle*/
//    WriteToCommunicationChannel(RobotIdStrToInt(), m_cProprioceptiveFeatureVector.GetValue(), listMapFVsToRobotIds_relay);
//}

/****************************************/
/****************************************/

//void CEPuckForaging::SendCRMResultsAndConsensusToNeighbours(bool b_CRM_Results_Valid)
//{
//    /* Commmunicate your CRM results to your neighbours
//     * Also broadcast consensus information - add any new consensus info to your local list and send it out again */
//    WriteToCommunicationChannel(RobotIdStrToInt(), listMapFVsToRobotIds,
//                                listFVsSensed, listConsensusInfoOnRobotIds, b_CRM_Results_Valid);

//    //printf(" finished WriteToCommunicationChannel \n\n");
//}

/****************************************/
/****************************************/

//void CEPuckForaging::Sense(Real m_fProbForget)
//{
//#if FV_MODE == PROPRIOCEPT_MODE

//#ifdef ConsensusOnMapOfIDtoFV
//    exit(-1);
//#endif

//    const CCI_RangeAndBearingSensor::TReadings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);

//    /* Listen for feature vectors from neighbours */
//    /* Read the id and proprioceptively computed FV of your neighbours. Read from communication channel and stored in listMapFVsToRobotIds */
//    /* Mark these read FVs as to be relayed. Stored separately in listMapFVsToRobotIds_relay */
//    /* Also read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */
//    bool read_status = ReadFromCommunicationChannel_IdFv(tmp); /* returns true if successfully read id and fvs from at least one neighbour*/


//    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBehavior::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS); /*remove entries older than 10s */

//    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed
//#endif

//#if FV_MODE == OBSERVATION_MODE || FV_MODE == COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE

//#ifdef ConsensusOnMapOfIDtoFV
//    exit(-1);
//#endif

//    listMapFVsToRobotIds_relay.clear();
//    for (size_t i = 0; i < m_cObservationFeatureVector.ObservedRobotIDs.size(); ++i)
//    {
//        unsigned robotId = m_cObservationFeatureVector.ObservedRobotIDs[i];
//        unsigned fv      = m_cObservationFeatureVector.ObservedRobotFVs[i];

//        listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv));

//        //if(robotId == 15)
//        //  std::cerr << "Observer: " << m_uRobotId << " ObservedId " << robotId << " ObservedFV " << fv << std::endl;

//        UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer);
//    }

//    const CCI_RangeAndBearingSensor::TReadings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);
//    bool read_status = ReadFromCommunicationChannel_RelayedFv(tmp); /* returns true if successfully read id and fvs from at least one neighbour*/

//    //remove entries older than 10s
//    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBehavior::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS);
//    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed
//#endif

///*#if FV_MODE == BAYESIANINFERENCE_MODE
//    listMapFVsToRobotIds_relay.clear();
//    for (size_t i = 0; i < m_cBayesianInferredFeatureVector.ObservedRobotIDs.size(); ++i)
//    {
//        unsigned robotId = m_cBayesianInferredFeatureVector.ObservedRobotIDs[i];
//        unsigned fv      = m_cBayesianInferredFeatureVector.ObservedRobotFVs[i];
//        unsigned num_observations = m_cBayesianInferredFeatureVector.ObservedRobotFVs_Min_Number_Featureobservations[i];


//        listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv));

//        //if(robotId == 15)
//        //  std::cerr << "Observer: " << m_uRobotId << " ObservedId " << robotId << " ObservedFV " << fv << std::endl;

//        UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer);
//    }

//    const CCI_RangeAndBearingSensor::TReadings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);
//    bool read_status = ReadFromCommunicationChannel_RelayedFv(tmp); // returns true if successfully read id and fvs from at least one neighbour

//    //remove entries older than 10s
//    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBehavior::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS);
//    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed
//#endif*/

//#if FV_MODE == BAYESIANINFERENCE_MODE

//    listMapFVsToRobotIds_relay.clear();
//    for (size_t i = 0; i < m_cBayesianInferredFeatureVector.ObservedRobotIDs.size(); ++i)
//    {
//        unsigned robotId = m_cBayesianInferredFeatureVector.ObservedRobotIDs[i];
//        unsigned fv      = m_cBayesianInferredFeatureVector.ObservedRobotFVs[i];
//        unsigned num_observations = m_cBayesianInferredFeatureVector.ObservedRobotFVs_Min_Number_Featureobservations[i];

////#ifndef ConsensusOnMapOfIDtoFV
//        listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv));
////#endif

//        //if(robotId == 15)
//        //  std::cerr << "Observer: " << m_uRobotId << " ObservedId " << robotId << " ObservedFV " << fv << std::endl;

//#ifndef ConsensusOnMapOfIDtoFV
//        UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer);
//#else
//        UpdateFvToRobotIdMap(listMapFVsToRobotIds, RobotIdStrToInt(), fv, robotId, m_fInternalRobotTimer);
//#endif
//    }

//    const CCI_RangeAndBearingSensor::TReadings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);
//    bool read_status = ReadFromCommunicationChannel_RelayedFv(tmp); /* returns true if successfully read id and fvs from at least one neighbour*/

//    //remove entries older than 10s
//    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBehavior::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS);

//#ifdef ConsensusOnMapOfIDtoFV
//    SelectBestFVFromAllObservedFVs(listMapFVsToRobotIds, CProprioceptiveFeatureVector::NUMBER_OF_FEATURES, m_pcRNG_FVs);
//#endif

///*#ifdef ConsensusOnMapOfIDtoFV
//    listMapFVsToRobotIds_relay.clear();
//    for(t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin(); itd != listMapFVsToRobotIds.end(); ++itd)
//        listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(itd->uRobotId, -1.0f, itd->uFV));
//#endif*/

//    /*if((RobotIdStrToInt() == 2 || RobotIdStrToInt() == 13) && (m_fInternalRobotTimer == 501.0f || m_fInternalRobotTimer == 502.0f))
//        PrintFvToRobotIdMap(RobotIdStrToInt(), listMapFVsToRobotIds, 15);*/

//    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed

//#endif
//}

/****************************************/
/****************************************/

//void CEPuckForaging::ReceiveVotesAndConsensus()
//{
//    /* Listen to votes and consensus from neighbours */
//    /* Read the voter id:
//     * Followed by <fv, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
//     * If a vote is received,
//     *                      1. map the fv to the robot id (if none existed - ignore vote???)
//     *                      2. if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search.
//     *                      3. if voter id has voted on mapped robot id before, ignore vote. But remember, the voted fv may map to other robot ids so continue search.
//     *
//     * If a consensus is received,
//     *                      1. update listConsensusInfoOnRobotIds to include id and ATTACK_CONSENSUS / TOLERATE_CONSENSUS
//     *                      2. if listConsensusInfoOnRobotIds already has id with a different CONSENSUS from the received message - problem with discrepancy in consensus???? Could except consensus from lowest voter id as true???
//     *
//    */

//    const CCI_RangeAndBearingSensor::TReadings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);
//    bool read_status = ReadFromCommunicationChannel_VotCon(tmp); /* returns true if successfully read votes or consensus from at least one neighbour*/
//}

/****************************************/
/****************************************/

//void CEPuckForaging::EstablishConsensus()
//{
//    /* For each robot id in listVoteInformationRobots that is not in listConsensusInfoOnRobotIds
//     * If #votes-registered > CONSENSUS_THRESHOLD, or if we are close to the expiry time of current VOTCON_RESULTS_VALIDFOR_SECONDS window (1s remaining) (not anymore)
//     * Establish temporary consensus on robot id by adding it to listConsensusInfoOnRobotIds
//     */

//    for (t_listVoteInformationRobots::iterator it_vot = listVoteInformationRobots.begin(); it_vot != listVoteInformationRobots.end(); ++it_vot)
//    {
//        unsigned VotedOnRobotId = it_vot->uRobotId;
//        bool b_ConsensusReachedOnId(false);

//        for (t_listConsensusInfoOnRobotIds::iterator it_cons = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
//        {
//            if (it_cons->uRobotId == VotedOnRobotId)
//            {
//                b_ConsensusReachedOnId = true;
//                break; /* the robot id in consensus list are unique */
//            }

//        }

//        if (b_ConsensusReachedOnId)
//            continue; /* consensus already reached for VotedOnRobotId, lets go to the next robot in the listVoteInformationRobots list */
//        else
//        {
//            /*bool b_OneSecondToVotConReset = (((unsigned)m_fInternalRobotTimer %
//                                              (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) >= 9u);*/
//            bool b_OneSecondToVotConReset(false);


//            /* If #votes-registered > CONSENSUS_THRESHOLD, or if we are close to the expiry time of current VOTCON_RESULTS_VALIDFOR_SECONDS window (1s remaining) after which the consensus and vote vectors will be cleared (not anymore) */
//            if ((it_vot->uVoterIds.size() >= CONSENSUS_THRESHOLD) || b_OneSecondToVotConReset) /* at least one vote will be registered. establish consensus on that */
//            {
//                listConsensusInfoOnRobotIds.push_back(ConsensusInformationRobots(it_vot->uRobotId,
//                                                                                 (it_vot->attackvote_count > it_vot->toleratevote_count)?1u:2u)); /* if equal votes, we tolerate robot*/
//            }
//        }
//    }
//}

/****************************************/
/****************************************/

void CEPuckForaging::RunGeneralFaults()
{
    //m_pcLEDs->SetAllColors(CColor::RED);

    m_vecBehaviors.clear();
    if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE)
    {
         CRandomWalkBehavior* pcStraightLineBehavior = new CRandomWalkBehavior(0.0f);
         m_vecBehaviors.push_back(pcStraightLineBehavior);
    }
    else if (m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f);
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

void CEPuckForaging::Reset()
{
    /* Reset robot state */
    m_sStateData.Reset();
    /* Reset food data */
    m_sFoodData.Reset();
    /* Set LED color */
    //m_pcLEDs->SetAllColors(CColor::RED);
    /* Clear up the last exploration result */
    m_eLastExplorationResult = LAST_EXPLORATION_NONE;
    m_pcRABA->ClearData();
    //m_pcRABA->SetData(0, LAST_EXPLORATION_NONE);
}

/****************************************/
/****************************************/

void CEPuckForaging::UpdateState()
{
    /* Reset state flags */
    m_sStateData.InNest = false;
    /* Read stuff from the ground sensor */
    const std::vector<Real> tGroundReads = m_pcGround->GetReadings();
    /*
    * You can say whether you are in the nest by checking the ground sensor
    * placed close to the wheel motors. It returns a value between 0 and 1.
    * It is 1 when the robot is on a white area, it is 0 when the robot
    * is on a black area and it is around 0.5 when the robot is on a gray
    * area.
    * The e-puck has 3 sensors in a straight line in the front of the robot.
    */
    if(tGroundReads[0] > 0.25f &&
            tGroundReads[0] < 0.75f &&
            tGroundReads[1] > 0.25f &&
            tGroundReads[1] < 0.75f &&
            tGroundReads[2] > 0.25f &&
            tGroundReads[2] < 0.75f)
        m_sStateData.InNest = true;


    m_sStateData.OnFood = false;
    if(tGroundReads[0] <= 0.1f &&
            tGroundReads[1] <= 0.1f &&
            tGroundReads[2] <= 0.1f)
        m_sStateData.OnFood = true;

    //std::cout << " tGroundReads[0] " << tGroundReads[0] << " tGroundReads[1] " << tGroundReads[1] << " tGroundReads[2] " << tGroundReads[2] << std::endl << std::endl;
}

/****************************************/
/****************************************/

void CEPuckForaging::RunForagingExperiment()
{
    switch(m_sStateData.State) /* Deciding state transitions */
    {
    case SStateData::STATE_RESTING:
    {
        //std::cout << "SStateData::STATE_RESTING " << std::endl;
        RestAtNest();
        TIME_STATE_RESTING++;
        break;
    }
    case SStateData::STATE_EXPLORING:
    {
        //std::cout << "SStateData::STATE_EXPLORING " << std::endl;
        Explore(); // have we transitioned from explore?
        TIME_STATE_EXPLORING++;
        break;
    }
    case SStateData::STATE_BEACON:
    {
        //std::cout << "SStateData::STATE_BEACON " << std::endl;
        BecomeABeacon();
        TIME_STATE_BEACON++;
        break;
    }
    case SStateData::STATE_RESTING_AT_FOOD:
    {
        //std::cout << "SStateData::STATE_RESTING_AT_FOOD " << std::endl;
        RestAtFood();
        TIME_STATE_RESTING_AT_FOOD++;
        break;
    }
    case SStateData::STATE_RETURN_TO_NEST:
    {
        //std::cout << "SStateData::STATE_RETURN_TO_NEST " << std::endl;
        ReturnToNest();
        TIME_STATE_RETURN_TO_NEST++;
        break;
    }
    default:
    {
        LOGERR << "We can't be here, there's a bug!" << std::endl;
    }
    }

    //m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;

    if(m_sStateData.State == SStateData::STATE_RESTING)
        m_vecBehaviors.clear(); // nothing to execute

    else if(m_sStateData.State == SStateData::STATE_RESTING_AT_FOOD)
        m_vecBehaviors.clear(); // nothing to execute

    else if(m_sStateData.State == SStateData::STATE_BEACON)
        m_vecBehaviors.clear(); // nothing to execute

    else if(m_sStateData.State == SStateData::STATE_EXPLORING)
    {
        m_vecBehaviors.clear();

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        Real MAX_BEACON_SIGNAL_RANGE = 1.0f; //1m
        CHomingToFoodBeaconBehavior* pcHomingToFoodBeaconBehavior = new CHomingToFoodBeaconBehavior(BEACON_SIGNAL, MAX_BEACON_SIGNAL_RANGE);
        m_vecBehaviors.push_back(pcHomingToFoodBeaconBehavior);

        /*CAntiPhototaxisBehavior* pcAntiPhototaxisBehavior = new CAntiPhototaxisBehavior();
        m_vecBehaviors.push_back(pcAntiPhototaxisBehavior);*/

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sStateData.State == SStateData::STATE_RETURN_TO_NEST) // Perform exploration
    {
        m_vecBehaviors.clear();

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        //! Oscillation of take control between CPhototaxisBehavior and Disperse can cause robots to remain stuck at food source in RETURN_TO_NEST STATE.
        //! But this happends very rarely
        //CPhototaxisBehavior* pCPhototaxisBehavior = new CPhototaxisBehavior();
        //m_vecBehaviors.push_back(pCPhototaxisBehavior);


        /*Real MAX_BEACON_SIGNAL_RANGE = 1.0f; //1m
        CHomingToFoodBeaconBehavior* pcHomingToNestBeaconBehavior = new CHomingToFoodBeaconBehavior(NEST_BEACON_SIGNAL, MAX_BEACON_SIGNAL_RANGE);
        m_vecBehaviors.push_back(pcHomingToNestBeaconBehavior);*/


        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }
}


/****************************************/
/****************************************/

void CEPuckForaging::RestAtNest()
{
    /* If we have stayed here enough, probabilistically switch to
    * 'exploring' */
    if(m_sStateData.TimeRested > m_sStateData.MinimumRestingTime &&
            m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.InitialRestToExploreProb)
    {
        //m_pcLEDs->SetAllColors(CColor::GREEN);
        m_sStateData.State = SStateData::STATE_EXPLORING;
        m_sStateData.TimeRested = 0;
    }
    else
    {
        /* Send out data with RABS that you are a nest beacon. Neighbouring robots will use this data to home in on your position */
        //m_pcRABA->SetData(0, NEST_BEACON_SIGNAL);

        ++m_sStateData.TimeRested;
    }
}

/****************************************/
/****************************************/

void CEPuckForaging::RestAtFood()
{
    /* If we have stayed here enough, probabilistically switch to
    * 'ReturnToNest' */
    if(m_sStateData.TimeRested > m_sStateData.MinimumRestingTime &&
            m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.InitialRestToExploreProb) //InitialRestToExploreProb used here as well.
    {
        //m_pcLEDs->SetAllColors(CColor::BLUE);
        m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
        m_sStateData.TimeRested = 0;
    }
    else
    {
        ++m_sStateData.TimeRested;
    }
}

/****************************************/
/****************************************/

void CEPuckForaging::BecomeABeacon()
{
    UpdateState();

    if(m_sStateData.OnFood)
    {
        /* Send out data with RABS that you are a beacon. Neighbouring robots will use this data to home in on your position */
        m_pcRABA->SetData(0, BEACON_SIGNAL);
    }
    else
        m_sStateData.State = SStateData::STATE_EXPLORING;
}

/****************************************/
/****************************************/

void CEPuckForaging::Explore()
{
    /* We switch to 'become a beacon' in one situation:
    * 1. if we are on a food item and there is no beacon nearby
    */


    /* We switch to 'return to nest' in two situations:
    * 1. if we have a food item
    * 2. if we have not found a food item for some time;
    *    in this case, the switch is probabilistic
    */

    bool bBecomeBeacon(false);
    UpdateState();
    if(m_sStateData.OnFood)
    {
        bBecomeBeacon = true;

        // Check if you are to be a beacon
        const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
        for(size_t i = 0; i < tPackets.size(); ++i)
        {
            /*if(tPackets[i].Data[0] == BEACON_SIGNAL)
                std::cout << "Packet index " << i << " packet range to beacon " << tPackets[i].Range << "cm" << std::endl;*/
            /* tPackets[i].Range is in cm and bearing is normalized [-pi pi] */
            if(tPackets[i].Data[0] == BEACON_SIGNAL && tPackets[i].Range < 25.0f) // Each food spot has radius of 0.2m // a slightly higher threshold is chosen to be safe
            {
                // Note: If a foraging robot blocks the IR rays from the RAB actuator of a beacon robot, more than one beacons may be formed at the foraging site
                bBecomeBeacon = false;
                break;
            }
        }
        if(bBecomeBeacon == false)
            m_sFoodData.HasFoodItem = true;
    }


    bool bReturnToNest(false);
    bool bRestAtFood(false);

    /*
    * Test the first condition: have we found a food item?
    * NOTE: the food data is updated by the loop functions, so
    * here we just need to read it
    * UPDATE: Not any more. Now it is check in the robot's control loop
    */
    if(m_sFoodData.HasFoodItem)
    {
        m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;
        /* Switch to 'return to nest' */
        bRestAtFood = true;
    }
    else if(bBecomeBeacon)
    {
        //m_eLastExplorationResult = BEACON_ESTABLISHED;
    }

    /* Test the second condition: we switch to 'return to
    * nest' if we have been wandering for a lot of time and found nothing */
    else if(m_sStateData.TimeExploringUnsuccessfully > m_sStateData.MinimumUnsuccessfulExploreTime)
    {
        /* Store the result of the expedition */
        m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
        /* Switch to 'return to nest' */
        bReturnToNest = true;
    }


    if(bReturnToNest) /* So, do we return to the nest - ie, the last exploration has been unsuccessful */
    {
        /* Yes, we do! */
        m_sStateData.TimeExploringUnsuccessfully = 0;
        m_sStateData.TimeSearchingForPlaceInNest = 0;
        //m_pcLEDs->SetAllColors(CColor::BLUE);
        m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
    }
    else if(bBecomeBeacon) /*Do we become a beacon */
    {
        /* Yes, we do! */
        m_sStateData.TimeExploringUnsuccessfully = 0;
        m_sStateData.TimeSearchingForPlaceInNest = 0;
        //m_pcLEDs->SetAllColors(CColor::YELLOW);
        m_sStateData.State = SStateData::STATE_BEACON;
    }
    else if(bRestAtFood) /* So, do we rest at the food source  */
    {
        /* Yes, we do! */
        m_sStateData.TimeExploringUnsuccessfully = 0;
        m_sStateData.TimeSearchingForPlaceInNest = 0;
        //m_pcLEDs->SetAllColors(CColor::BLACK);
        m_sStateData.State = SStateData::STATE_RESTING_AT_FOOD;
    }
    else
    {
        /* No, perform the actual exploration */
        ++m_sStateData.TimeExploringUnsuccessfully;
    }
}

/****************************************/
/****************************************/

void CEPuckForaging::ReturnToNest()
{
    /* As soon as you get to the nest, switch to 'resting' */
    UpdateState();
    /* Are we in the nest? */
    if(m_sStateData.InNest)
    {
        if(m_sFoodData.HasFoodItem)
        {
            /* Drop the food item */
            m_sFoodData.HasFoodItem = false;
            ++m_sFoodData.TotalFoodItems;
        }

        /* Have we looked for a place long enough? */
        if(m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime)
        {
            // m_pcLEDs->SetAllColors(CColor::YELLOW); if the robot became a nest beacon

            /* switch to state 'resting' */
            //m_pcLEDs->SetAllColors(CColor::RED);
            m_sStateData.State = SStateData::STATE_RESTING;
            m_sStateData.TimeSearchingForPlaceInNest = 0;
            m_eLastExplorationResult = LAST_EXPLORATION_NONE;
            return;
        }
        else
        {
            /* No, keep looking */
            ++m_sStateData.TimeSearchingForPlaceInNest;
        }
    }
    else
    {
        /* Still outside the nest */
        m_sStateData.TimeSearchingForPlaceInNest = 0;
    }
    //    /* Keep going */
    //    bool bCollision;
    //    SetWheelSpeedsFromVector(
    //                m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
    //                m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
}

/****************************************/
/****************************************/

unsigned CEPuckForaging::RobotIdStrToInt()
{
    std::string id = GetId();
    id.erase(0, 2); // remove the first two characters

    std::string::size_type sz;   // alias of size_t
    unsigned u_id = std::stoi(id, &sz);
    return u_id;

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

//void CEPuckForaging::WriteToCommunicationChannel(unsigned SelfId, const CCI_RangeAndBearingSensor::TReadings& tPackets, t_listMapFVsToRobotIds &IdToFVsMap_torelay)
//{
//    size_t databyte_index;

//    if ((m_pcRABA->GetData(0) == BEACON_SIGNAL) || (m_pcRABA->GetData(0) == NEST_BEACON_SIGNAL))
//        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 onwards
//        databyte_index = 1;
//    else
//        databyte_index = 0;


//    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET);
//    m_pcRABA->SetData(databyte_index++, SelfId);

//#if FV_MODE == COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE || FV_MODE == BAYESIANINFERENCE_MODE

//    // angular acceleration [-1, +1] to [0, DATA_BYTE_BOUND]
//    unsigned un_angularacceleration = (unsigned)(((m_cProprioceptiveFeatureVector.m_sSensoryData.GetNormalisedAngularAcceleration() + 1.0) / 2.0) * DATA_BYTE_BOUND);
//    m_pcRABA->SetData(databyte_index++, un_angularacceleration);

//#elif FV_MODE == OBSERVATION_MODE
//    for(size_t i = 0; i < tPackets.size(); ++i)
//    {
//        size_t byte_index = 0; unsigned robotId, un_bearing; CRadians bearing;

//        if((tPackets[i].Data[0] == BEACON_SIGNAL) || (tPackets[i].Data[0] == NEST_BEACON_SIGNAL)) // data from a beacon  - get the next two bytes
//            byte_index = 1;
//        else
//            byte_index = 0;

//        byte_index++; // the message header type. this is always followed by the robot id.

//        robotId = tPackets[i].Data[byte_index];
//        bearing = tPackets[i].HorizontalBearing;
//        un_bearing = (unsigned)(ToDegrees(bearing).UnsignedNormalize().GetValue() * DATA_BYTE_BOUND / 360.0f);

//        /*if (SelfId == 15 && robotId == 17)
//        {
//            std::cerr << ToDegrees(bearing).UnsignedNormalize() << " " << ToDegrees(bearing).UnsignedNormalize().GetValue() << std::endl;
//            std::cerr << ToDegrees(bearing).UnsignedNormalize().GetValue() * (DATA_BYTE_BOUND / 360.0f) << " " << (un_bearing * 360.0f / DATA_BYTE_BOUND) << std::endl;
//        }*/

//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, const CCI_RangeAndBearingSensor::TReadings& tPackets) ";
//            exit(-1);
//        }
//        m_pcRABA->SetData(databyte_index++, robotId);

//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, const CCI_RangeAndBearingSensor::TReadings& tPackets) ";
//            exit(-1);
//        }
//        m_pcRABA->SetData(databyte_index++, un_bearing);
//    }
//#endif

//    /*if(databyte_index != m_pcRABA->GetSize()-1) // END_BUFFER has not yet been placed
//      m_pcRABA->SetData(databyte_index, END_BUFFER);*/

//    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET_FOOTER);


//    m_pcRABA->SetData(databyte_index++, RELAY_FVS_PACKET);
//    for (t_listMapFVsToRobotIds::iterator it = IdToFVsMap_torelay.begin(); it != IdToFVsMap_torelay.end(); ++it)
//    {
//        m_pcRABA->SetData(databyte_index++, it->uRobotId);
//        m_pcRABA->SetData(databyte_index++, it->uFV);
//    }
//    m_pcRABA->SetData(databyte_index, RELAY_FVS_PACKET_FOOTER);
//}

/****************************************/
/****************************************/

//void CEPuckForaging::WriteToCommunicationChannel(unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay)
//{
//    size_t databyte_index;

//    if ((m_pcRABA->GetData(0) == BEACON_SIGNAL) || (m_pcRABA->GetData(0) == NEST_BEACON_SIGNAL))
//        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 onwards
//        databyte_index = 1;
//    else
//        databyte_index = 0;


//    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET);
//    m_pcRABA->SetData(databyte_index++, SelfId);
//    m_pcRABA->SetData(databyte_index++, SelfFV);
//    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET_FOOTER);


//    m_pcRABA->SetData(databyte_index++, RELAY_FVS_PACKET);
//    bool buffer_full(false);
//    for(t_listMapFVsToRobotIds::iterator itd = IdToFVsMap_torelay.begin(); itd != IdToFVsMap_torelay.end(); ++itd)
//    {
//        m_pcRABA->SetData(databyte_index++, itd->uRobotId);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            /*buffer_full = true;
//            m_pcRABA->SetData(databyte_index, END_BUFFER);
//            break;*/
//            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay) " << std::endl;
//            exit(-1);
//        }

//        m_pcRABA->SetData(databyte_index++, itd->uFV);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            /*buffer_full = true;
//            m_pcRABA->SetData(databyte_index, END_BUFFER);
//            break;*/
//            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay) " << std::endl;
//            exit(-1);
//        }
//    }

//    /*if(databyte_index != m_pcRABA->GetSize()-1) // END_BUFFER has not yet been placed
//      m_pcRABA->SetData(databyte_index, END_BUFFER);*/

//    m_pcRABA->SetData(databyte_index, RELAY_FVS_PACKET_FOOTER);
//}

/****************************************/
/****************************************/

//void CEPuckForaging::WriteToCommunicationChannel(unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds,
//                                                 t_listFVsSensed& CRMResultsOnFVDist, t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid)
//{
//    //printf("WriteToCommunicationChannel \n\n");

//    size_t databyte_index;

//    // we now put all the different message types in the same packet - to be sent at the same cycle
//    /*if ((m_pcRABA->GetData(0) == BEACON_SIGNAL) || (m_pcRABA->GetData(0) == NEST_BEACON_SIGNAL))
//        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 onwards
//        databyte_index = 1;
//    else
//    databyte_index = 0;*/

//    bool end_buffer_found(false);
//    for (size_t tmp_index = 0; tmp_index < m_pcRABA->GetSize(); ++tmp_index)
//    {
//        if (m_pcRABA->GetData(tmp_index) == RELAY_FVS_PACKET_FOOTER)
//        {
//            end_buffer_found = true;
//            databyte_index = tmp_index + 1;
//            break;
//        }
//    }

//    if (end_buffer_found == false)
//    {
//        std::cerr << " RELAY_FVS_PACKET_FOOTER not found  " << " WriteToCommunicationChannel(unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds, t_listFVsSensed& CRMResultsOnFVDist, t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid) " << std::endl;
//        exit(-1);
//    }


//    if(databyte_index == (m_pcRABA->GetSize()-1))
//    {
//        std::cerr << " buffer full. no place to write voter packet header type " << std::endl;
//        exit(-1);
//    }
//    m_pcRABA->SetData(databyte_index++, VOTER_PACKET);
//    if(databyte_index == (m_pcRABA->GetSize()-1))
//    {
//        std::cerr << " buffer full. no place to write voter id " << std::endl;
//        exit(-1);
//    }
//    m_pcRABA->SetData(databyte_index++, VoterId);


//    if(CRMResultsOnFVDist.size() == 0 && ConsensusLst.size() == 0) // nothing to be written
//    {
//        if(databyte_index == (m_pcRABA->GetSize()))
//        {
//            std::cerr << " buffer full. no place to write end buffer " << std::endl;
//            exit(-1);
//        }
//        m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//        return;
//    }

//    bool buffer_full(false);
//    /*
//     * Write the consensus list to the channel comprising < .... <robot id, its consensus state> ... >
//     */
//    for (t_listConsensusInfoOnRobotIds::iterator it_cons = ConsensusLst.begin(); it_cons != ConsensusLst.end(); ++it_cons)
//    {
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }

//        m_pcRABA->SetData(databyte_index++, it_cons->uRobotId);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }

//        m_pcRABA->SetData(databyte_index++, (it_cons->consensus_state==1)?ATTACK_CONSENSUS:TOLERATE_CONSENSUS);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }
//    }

//    if(buffer_full)
//    {
//        std::cerr << " Written consensus. But buffer full now. No longer able to write the vote packet. complain by exiting" << std::endl;
//        std::cerr << " CRMResultsOnFVDist.size() " << CRMResultsOnFVDist.size() << " ConsensusLst.size() " << ConsensusLst.size() << std::endl;
//        exit(-1);
//    }

//    if(!b_CRM_Results_Valid) /* the crm results on the FVs in CRMResultsOnFVDist is no longer valid */
//    {
//        if(databyte_index == (m_pcRABA->GetSize()))
//        {
//            std::cerr << " buffer full. no place to write end buffer " << std::endl;
//            exit(-1);
//        }
//        m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//        return;
//    }

//    /*
//     * Write the results of CRM to the channel comprising < .... <fv, attack/tolerate state> ... >
//     * We dont write the results of all the FVs in the listFVsSensed as they may be some very old FVs no longer present in the swarm.
//     * Update: Your FV-ID map may be old too. Other robots may have a better map. Don't curtail information from them.
//     */
//    buffer_full = false;
//    for (t_listFVsSensed::iterator it_fvdist = CRMResultsOnFVDist.begin(); it_fvdist != CRMResultsOnFVDist.end(); ++it_fvdist)
//    {
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }

//        m_pcRABA->SetData(databyte_index++, it_fvdist->uFV);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }

//        m_pcRABA->SetData(databyte_index++, (it_fvdist->uMostWantedState==1)?ATTACK_VOTE:TOLERATE_VOTE);
//        if(databyte_index == m_pcRABA->GetSize()-1)
//        {
//            buffer_full = true;
//            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//            break;
//        }
//    }

//    if(buffer_full)
//    {
//        std::cerr << " No longer able to write the vote packet. complain by exiting" << std::endl;
//        std::cerr << " CRMResultsOnFVDist.size() " << CRMResultsOnFVDist.size() << " ConsensusLst.size() " << ConsensusLst.size() << std::endl;
//        exit(-1);
//    }

//    if(databyte_index != m_pcRABA->GetSize()-1) // END_BUFFER has not yet been placed
//        m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
//}

/****************************************/
/****************************************/

//bool func_SortPacketsOnRange (const CCI_RangeAndBearingSensor::SPacket i, const CCI_RangeAndBearingSensor::SPacket j) { return (i.Range < j.Range); }

/****************************************/

//bool  CEPuckForaging::ReadFromCommunicationChannel_IdFv(const CCI_RangeAndBearingSensor::TReadings& tPackets)
//{
//    /* Read the id and proprioceptively computed FV of your neighbours. Read from communication channel and stored in listMapFVsToRobotIds */
//    /* Mark these read FVs as to be relayed. Stored separately in listMapFVsToRobotIds_relay */
//    /* Also read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */

//    bool read_successful(false); // successfully read id and fvs from at least one neighbour

//    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations

//    // Adding the most recent observations into listMapFVsToRobotIds and marked for relay in listMapFVsToRobotIds_relay
//    listMapFVsToRobotIds_relay.clear();
//    for(size_t i = 0; i < tPackets.size(); ++i)
//    {
//        size_t byte_index = 0; unsigned robotId, fv;

//        if((tPackets[i].Data[0] == BEACON_SIGNAL) || (tPackets[i].Data[0] == NEST_BEACON_SIGNAL)) // data from a beacon  - get the next two bytes
//            byte_index = 1;
//        else
//            byte_index = 0;


//        if(tPackets[i].Data[byte_index++] == SELF_INFO_PACKET) // this neighbour is not sending me its FVs
//        {
//            robotId = tPackets[i].Data[byte_index++];
//            fv      = tPackets[i].Data[byte_index++];
//            read_successful = true;
//            byte_index++; // SELF_INFO_PACKET_FOOTER

//            listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv));
//            UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer);
//        }


//        if(tPackets[i].Data[byte_index++] == RELAY_FVS_PACKET) // this neighbour is not sending me its FVs
//            for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
//            {
//                if(tPackets[i].Data[byteindex1] == RELAY_FVS_PACKET_FOOTER)
//                    break;

//                robotId = tPackets[i].Data[byteindex1];

//                if(tPackets[i].Data[byteindex1+1] == RELAY_FVS_PACKET_FOOTER)
//                    break;

//                fv      = tPackets[i].Data[byteindex1+1];

//                UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer-1); // old information
//            }

//    }

//    return read_successful;
//}

/****************************************/
/****************************************/

//bool  CEPuckForaging::ReadFromCommunicationChannel_RelayedFv(const CCI_RangeAndBearingSensor::TReadings& tPackets)
//{
//    /* Only read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */

//    bool read_successful(false); // successfully read id and fvs from at least one neighbour

//    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations


//    for(size_t i = 0; i < tPackets.size(); ++i)
//    {
//        size_t byte_index = 0;  unsigned robotId, fv, observerId = 999u;

//        bool SELF_INFO_PACKET_FOUND(false);
//        for(byte_index = 0; byte_index < tPackets[i].Data.Size(); ++byte_index)
//        {
//            if(tPackets[i].Data[byte_index] == SELF_INFO_PACKET)
//            {
//                SELF_INFO_PACKET_FOUND = true;
//                byte_index++;
//                break;
//            }
//        }

//        if(SELF_INFO_PACKET_FOUND == true)
//            observerId = tPackets[i].Data[byte_index];


//        bool RELAY_FVS_PACKET_FOUND(false);
//        for(byte_index = 0; byte_index < tPackets[i].Data.Size(); ++byte_index)
//        {
//            if(tPackets[i].Data[byte_index] == RELAY_FVS_PACKET)
//            {
//                RELAY_FVS_PACKET_FOUND = true;
//                byte_index++;
//                break;
//            }
//        }

//        if(RELAY_FVS_PACKET_FOUND == false)
//            continue;


//        for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
//        {
//            if (tPackets[i].Data[byteindex1] == RELAY_FVS_PACKET_FOOTER)
//                break;

//            robotId = tPackets[i].Data[byteindex1];

//            if (tPackets[i].Data[byteindex1+1] == RELAY_FVS_PACKET_FOOTER)
//                break;

//            fv      = tPackets[i].Data[byteindex1+1];


//            /*if(m_uRobotId == robotId)
//            {
//                std::cerr << "Robot " << observerId << " observing " << robotId << " fv " << fv << std::endl;
//            }*/

//            if(robotId == 14)
//            {
//                    //std::cerr << "Robot " << observerId << " observing " << robotId << " fv " << fv << std::endl;
//            }
//            else
//            {
//                    //std::cout << "Robot " << observerId << " observing " << robotId << " fv " << fv << std::endl;
//            }


//            read_successful = true;

//            //UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer-1); // old information
//#ifndef ConsensusOnMapOfIDtoFV
//            UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer-1); // old information
//#else
//            if(observerId == 999u)
//            {
//                printf("\n observerId was not in packet");
//                exit(-1);
//            }
//            UpdateFvToRobotIdMap(listMapFVsToRobotIds, observerId, fv, robotId, m_fInternalRobotTimer-1);
//#endif
//        }
//    }

//    return read_successful;
//}

/****************************************/
/****************************************/

//bool  CEPuckForaging::ReadFromCommunicationChannel_VotCon(const CCI_RangeAndBearingSensor::TReadings& tPackets)
//{

//    /* Listen to votes and consensus from neighbours */
//    /* Read the voter id:
//     * Followed by <fv, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
//     * If a vote is received,
//     *                      1. map the fv to the robot id (if none existed - ignore vote???)
//     *                      2. if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search.
//     *                      3. if voter id has voted on mapped robot id before, ignore vote. But remember, the voted fv may map to other robot ids so continue search.
//     *
//     * If a consensus is received,
//     *                      1. update listConsensusInfoOnRobotIds to include id and ATTACK_CONSENSUS / TOLERATE_CONSENSUS
//     *                      2. if listConsensusInfoOnRobotIds already has id with a different CONSENSUS from the received message - problem with discrepancy in consensus???? Could except consensus from lowest voter id as true???
//     *
//    */

//    bool read_successful(false); // successfully read votes / consensus from at least one neighbour


//    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations


//    for(size_t i = 0; i < tPackets.size(); ++i)
//    {
//        size_t byte_index = 0;
//        unsigned votertId, fv, attack_tolerate_vote, ConsensusOnRobotId, ConsensusState; unsigned tmp1, tmp2;

//        /*if((tPackets[i].Data[0] == BEACON_SIGNAL) || (tPackets[i].Data[0] == NEST_BEACON_SIGNAL)) // data from a beacon  - get the next two bytes
//            byte_index = 1;
//        else
//        byte_index = 0;*/

//        bool voter_packet_found(false);
//        for (size_t tmp_index = 0; tmp_index < tPackets[i].Data.Size(); ++tmp_index)
//        {
//            if (tPackets[i].Data[tmp_index] == VOTER_PACKET)
//            {
//                voter_packet_found = true;
//                byte_index = tmp_index + 1;
//                break;
//            }
//        }

//        if(voter_packet_found == false) // this neighbour is not sending me any votes or consensus
//            continue;


//        votertId = tPackets[i].Data[byte_index++];

//        for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
//        {
//            //printf("\npacket index %d; byteindex1=%d \n",i,byteindex1);

//            if(tPackets[i].Data[byteindex1] == VOTER_PACKET_FOOTER)
//                break;

//            tmp1 = tPackets[i].Data[byteindex1];

//            if(tPackets[i].Data[byteindex1+1] == VOTER_PACKET_FOOTER)
//                break;

//            tmp2 = tPackets[i].Data[byteindex1+1];


//            if(tmp2 == ATTACK_VOTE || tmp2 == TOLERATE_VOTE)
//            {
//                fv                   = tmp1;
//                attack_tolerate_vote = (tmp2==ATTACK_VOTE)?1u:2u;

//                UpdateVoterRegistry(listVoteInformationRobots,
//                                    listMapFVsToRobotIds,
//                                    listConsensusInfoOnRobotIds,
//                                    votertId, fv, attack_tolerate_vote);
//            }
//            else
//            {
//                assert(tmp2 == ATTACK_CONSENSUS || tmp2 == TOLERATE_CONSENSUS);
//                ConsensusOnRobotId = tmp1;
//                ConsensusState     = (tmp2==ATTACK_CONSENSUS)?1u:2u;

//                bool b_ConsensusAlreadyEstablishedOnRobot(false);
//                for (t_listConsensusInfoOnRobotIds::iterator it_cons = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
//                {
//                    if(it_cons->uRobotId == ConsensusOnRobotId)
//                    {
//                        b_ConsensusAlreadyEstablishedOnRobot = true;
//                        if (ConsensusState != it_cons->consensus_state)
//                        {
//                            // Difference in consensus state. there is a disparity in our consensus ????
//                            // We can correct state, assuming the lowest id (between my id and voter id) is correct?
//                        }
//                        break; // robot ids are unique in the consensus list
//                    }
//                }

//                if(!b_ConsensusAlreadyEstablishedOnRobot)
//                    listConsensusInfoOnRobotIds.push_back(ConsensusInformationRobots(ConsensusOnRobotId, ConsensusState));
//            }

//            read_successful = true;
//        }

//    }

//    return read_successful;
//}

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
REGISTER_CONTROLLER(CEPuckForaging, "epuck_foraging_controller")
