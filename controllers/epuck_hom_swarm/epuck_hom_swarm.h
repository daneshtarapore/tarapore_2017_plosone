/*
 * AUTHOR: Danesh Tarapore <daneshtarapore@gmail.com>
 *
 * An example foraging controller for the foot-bot.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/foraging.argos
 */

//! TODO
//! 1. Introduce noise in the sensors and actuators of the e-puck (see parameters from Lorenzo)


#ifndef EPUCK_HOMSWARM_H
#define EPUCK_HOMSWARM_H

/*
 * Include some necessary headers.
 */

#include <iostream>
#include <vector>
#include <algorithm>    // std::sort

/****************************************/
/****************************************/
/* ARGoS headers */

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>

/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

/* Definition of the differential steering wheel encoder */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>

/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/* Definition of the e-puck proximity sensor */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/****************************************/
/****************************************/
/* Definitions for behaviors used to control robot */

#include "behavior.h"
#include "aggregatebehavior.h"
#include "dispersebehavior.h"
#include "randomwalkbehavior.h"
#include "phototaxisbehavior.h"
#include "antiphototaxisbehavior.h"
#include "homingtofoodbeaconbehavior.h"
#include "circlebehavior.h"

/****************************************/
/****************************************/
/* Definition of functions to estimate feature-vectors - proprioceptively, or by observation */

#include "propriofeaturevector.h"
#include "observedfeaturevector.h"

/****************************************/
/****************************************/
/* Definition of functions to assimilate the different feature-vectors and perform abnormality detection */

#include "featurevectorsinrobotagent.h"
#include "crminrobotagent_optimised.h"

/****************************************/
/****************************************/

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CEPuckHomSwarm : public CCI_Controller
{

public:

    struct ExperimentToRun
    {
        /* The type of experiment to run */
        enum SwarmBehavior
        {
            SWARM_AGGREGATION = 0,
            SWARM_DISPERSION,
            SWARM_HOMING
        } SBehavior;


        /* The possible faults on robot */
        enum FaultBehavior
        {
            FAULT_NONE = 0,

            /*faults whose effects cause one of the following four general failures */
            FAULT_STRAIGHTLINE,
            FAULT_RANDOMWALK,
            FAULT_CIRCLE,
            FAULT_STOP
        } FBehavior;

        std::string id_FaultyRobotInSwarm;

        ExperimentToRun();
        void Init(TConfigurationNode& t_node);
    };

    /*
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_foraging_controller><parameters><wheel_turning>
    * section.
    */
    struct SWheelTurningParams
    {
        Real MaxSpeed;
        void Init(TConfigurationNode& t_tree);
    };


    struct RobotDetails
    {
        Real iterations_per_second; /* controlcycles run per second*/
        Real seconds_per_iterations;
        Real HALF_INTERWHEEL_DISTANCE;  // in m
        Real INTERWHEEL_DISTANCE;  // in m
        Real WHEEL_RADIUS;  // in m

        CRadians m_cNoTurnOnAngleThreshold; CRadians m_cSoftTurnOnAngleThreshold;

        Real MaxLinearSpeed; //cm/ (controlcycle)
        Real MaxLinearAcceleration; //cm/controlcycle/controlcycle

        Real MaxAngularSpeed; //rad/controlcycle
        Real MaxAngularAcceleration; //rad/controlcycle/controlcycle;

        RobotDetails()
        {
            iterations_per_second  = 10.0f; /*10 ticks per second so dt=0.01s. i.e., the controlcycle is run 10 times per second*/
            seconds_per_iterations = 1.0f / CBehavior::m_sRobotData.iterations_per_second;
            HALF_INTERWHEEL_DISTANCE = 0.053f * 0.5f;  // m
            INTERWHEEL_DISTANCE  = 0.053f;  // m
            WHEEL_RADIUS = 0.0205f;  // m

            m_cNoTurnOnAngleThreshold   = ToRadians(CDegrees(10.0f)); //10.0 - straight to food spot; 35.0 spiral to food spot
            m_cSoftTurnOnAngleThreshold = ToRadians(CDegrees(70.0f));
        }

        void SetKinematicDetails(Real f_MaxLeftWheelSpeed, Real f_MaxRightWheelSpeed) // arguments are speeds in cm/s
        {
            // the max linear speed is 1 cm/controlcycle.
            // so, MaxLinearSpeed = 1
            MaxLinearSpeed        = ((f_MaxLeftWheelSpeed + f_MaxRightWheelSpeed) / 2.0f) * seconds_per_iterations;  //in cm/ (controlcycle)

            // as the max speed is 1 cm/controlcycle (resultant speed is always positive as the robot does not traverse backwards), the max acceleration is +/-1 cm/controlcycle/controlcycle
            // so, MaxLinearAcceleration = |+/-1 cm/controlcycle/controlcycle| = 1cm/controlcycle/controlcycle
            MaxLinearAcceleration = MaxLinearSpeed;

            // the max angular speed is +/-21.6 degrees/controlcycle,
            // so MaxAngularSpeed = |+/-21.621 degrees/controlcycle| = |21.621 degrees/controlcycle|
            MaxAngularSpeed       = ((f_MaxLeftWheelSpeed + f_MaxRightWheelSpeed) /
                                     (INTERWHEEL_DISTANCE * 100.0f)) * seconds_per_iterations; //rad/controlcycle

            // as the max angular speed is +/-21.6 degrees/controlcycle, the max acceleration is +/-43.2421 radians/controlcycle/controlcycle
            // so MaxAngularAcceleration = |+/-43.2421 degrees/controlcycle/controlcycle| = 43.2421 degrees/controlcycle/controlcycle
            MaxAngularAcceleration   = 2.0f * MaxAngularSpeed; //rad/controlcycle/controlcycle;
        }
    };

    RobotDetails m_sRobotDetails;

public:

    /* Class constructor. */
    CEPuckHomSwarm();
    /* Class destructor. */
    virtual ~CEPuckHomSwarm();

    /*
     *
     *
    */
    virtual void CopyRobotDetails(RobotDetails& sRobotDetails);

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_foraging_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

    /*
     *
     *
     */
    unsigned SumFVDist(t_listFVsSensed& FVsSensed);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

    /*
     *
     *
     */
    virtual void SendIdSelfBearingAndObsFVsToNeighbours(const CCI_RangeAndBearingSensor::TReadings& tPackets, t_listMapFVsToRobotIds& IdToFVsMap_torelay);

    /*
     *
     *
    */
    virtual void SendFVsToNeighbours();


    /*
     *
     *
     */
    virtual void WriteToCommunicationChannel(unsigned SelfId, const CCI_RangeAndBearingSensor::TReadings &tPackets, t_listMapFVsToRobotIds &IdToFVsMap_torelay);

    /*
     *
     *
     */
    virtual void WriteToCommunicationChannel(unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay);

    /*
     *
     *
     */
    virtual void WriteToCommunicationChannel(unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds,
                                             t_listFVsSensed& CRMResultsOnFVDist, t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid);

    /*
     *
     */
    virtual bool ReadFromCommunicationChannel_IdFv(const CCI_RangeAndBearingSensor::TReadings& tPackets);


    /*
     *
     */
    virtual bool ReadFromCommunicationChannel_RelayedFv(const CCI_RangeAndBearingSensor::TReadings& tPackets);


    /*
     *
     */
    virtual bool ReadFromCommunicationChannel_VotCon(const CCI_RangeAndBearingSensor::TReadings& tPackets);

    /*
    * This function is called once every time step.
    * It listens for feature-vectors at the current time-step and then assimilates them into the robot's internal feature-vector distribution.
    */
    virtual void Sense(Real m_fProbForget);

    /*
     *
     */
    virtual void ReceiveVotesAndConsensus();

    /*
     *
     */
    virtual void EstablishConsensus();


    /*
     *
     *
     */
    virtual void SendCRMResultsAndConsensusToNeighbours(bool b_CRM_Results_Valid);

    /*
     * Execute general faults
    */
    virtual void RunGeneralFaults();

    /*
     * Run one of the swarm experiments (Aggregation, Dispersion, Homing)
    */
    virtual void RunHomogeneousSwarmExperiment();

    /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    */
    virtual void Reset();

    /*
     *  This function returns the interger cast of the string robot id
    */
    virtual unsigned RobotIdStrToInt();

    /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

    virtual t_listConsensusInfoOnRobotIds& GetListConsensusInfoOnRobotIds()
    {
        return listConsensusInfoOnRobotIds;
    }

    /*
     * For debugging purposes
     */
    virtual unsigned GetRobotFeatureVector()
    {
        return m_uRobotFV;
    }

    /*
    * Returns the experiment type
    */
    inline ExperimentToRun& GetExperimentType()
    {
        return m_sExpRun;
    }

    virtual CObservedFeatureVector& GetObservedFeatureVectors()
    {
        return m_cObservationFeatureVector;
    }


    t_listFVsSensed&             GetListFVsSensed()         {return listFVsSensed;}
    t_listMapFVsToRobotIds&      GetMapFVsSensed()          {return listMapFVsToRobotIds;}

    Real m_fInternalRobotTimer;
    Real m_fCRM_RUN_TIMESTAMP;
    bool b_CRM_Run;

    static UInt8 BEACON_SIGNAL;

private:



private:

    TBehaviorVector             m_vecBehaviors;
    bool                        b_damagedrobot;     // true if robot is damaged

    CProprioceptiveFeatureVector  m_cProprioceptiveFeatureVector;
    CObservedFeatureVector        m_cObservationFeatureVector;

    t_listFVsSensed               listFVsSensed;
    t_listMapFVsToRobotIds        listMapFVsToRobotIds; // ids and fvs of observed neighbours, including ids and fvs the neighbours have relayed to you
    t_listMapFVsToRobotIds        listMapFVsToRobotIds_relay; // ids and fvs of observed neighbours - for you to relay to your neighbours.

    t_listConsensusInfoOnRobotIds listConsensusInfoOnRobotIds; // consensus established on the following robots

    t_listVoteInformationRobots   listVoteInformationRobots;   // list of registered votes for each observed robot id, and the corresponding voter ids

    CRMinRobotAgentOptimised*     crminAgent;

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;

    /* Pointer to the differential steering encoder */
    CCI_DifferentialSteeringSensor* m_pcWheelsEncoder;
    /* Pointer to the LEDs actuator */
    CCI_LEDsActuator* m_pcLEDs;
    /* Pointer to the range and bearing actuator */
    CCI_RangeAndBearingActuator*  m_pcRABA;
    /* Pointer to the range and bearing sensor */
    CCI_RangeAndBearingSensor* m_pcRABS;
    /* Pointer to the foot-bot proximity sensor */
    CCI_EPuckProximitySensor* m_pcProximity;

    /* The random number generator */
    CRandom::CRNG* m_pcRNG;
    CRandom::CRNG* m_pcRNG_FVs; // we have a separate RNG for forgetting FVs. This way the actual behaviors based on random walk are independent from the running of the CRM.


    /*Information on the experiment to run - the swarm behavior and the faulty behavior*/
    ExperimentToRun m_sExpRun;
    /* The turning parameters */
    SWheelTurningParams m_sWheelTurningParams;

    unsigned m_uRobotId, m_uRobotFV;
};

#endif
