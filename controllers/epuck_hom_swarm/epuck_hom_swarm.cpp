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
    b_damagedrobot(false) {m_rTime=0.0f;}

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

    CProprioceptiveFeatureVector::m_sRobotData.robotid                  = this->GetId();
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::ControlStep()
{
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

    m_rTime+=1.0f;
    std::cout << "m_rTime = " << m_rTime << std::endl;
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

        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(30.0f); //range threshold in cm
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
            /* Sends out data with RABS that you are a beacon. Neighbouring robots will use this data to home in on your position */
            unsigned BEACON_ESTABLISHED = 3u;
            m_pcRABA->SetData(0, BEACON_ESTABLISHED);
            m_pcLEDs->SetAllColors(CColor::YELLOW);
        }
        else
        {
            CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
            m_vecBehaviors.push_back(pcDisperseBehavior);

            CHomingToFoodBeaconBehavior* pcHomingToFoodBeaconBehavior = new CHomingToFoodBeaconBehavior();
            m_vecBehaviors.push_back(pcHomingToFoodBeaconBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
            m_vecBehaviors.push_back(pcRandomWalkBehavior);
        }
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
