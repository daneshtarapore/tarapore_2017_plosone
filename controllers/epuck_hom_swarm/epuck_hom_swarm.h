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

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>

/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>


/* Definitions for behaviors used to control robot */
#include "behavior.h"
#include "aggregatebehavior.h"
#include "dispersebehavior.h"
#include "randomwalkbehavior.h"
#include "phototaxisbehavior.h"
#include "antiphototaxisbehavior.h"
#include "homingtofoodbeaconbehavior.h"
#include "circlebehavior.h"

#include "propriofeaturevector.h"

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

public:

    /* Class constructor. */
    CEPuckHomSwarm();
    /* Class destructor. */
    virtual ~CEPuckHomSwarm() {}

    /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_foraging_controller> section.
    */
    virtual void Init(TConfigurationNode& t_node);

    /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
    virtual void ControlStep();

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
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
    virtual void Destroy() {}

    /*
    * Returns the experiment type
    */
    inline ExperimentToRun& GetExperimentType()
    {
        return m_sExpRun;
    }

    Real m_rTime;

private:



private:

    TBehaviorVector             m_vecBehaviors;
    bool                        b_damagedrobot;     // true if robot is damaged

    CProprioceptiveFeatureVector  m_cProprioceptiveFeatureVector;

    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
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


    /*Information on the experiment to run - the swarm behavior and the faulty behavior*/
    ExperimentToRun m_sExpRun;
    /* The turning parameters */
    SWheelTurningParams m_sWheelTurningParams;
};

#endif
