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
    b_damagedrobot(false) {m_fInternalRobotTimer=0.0f;}

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
    crminAgent = new CRMinRobotAgentOptimised(RobotIdStrToInt(GetId()), CProprioceptiveFeatureVector::NUMBER_OF_FEATURES);
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

    /* Broadcast the proprioceptively computed FV to whoever is in range, using the RAB sensor*/
    if (m_pcRABA->GetData(0) == BEACON_SIGNAL)
    {
        //std::cout << "ep0 - byte 0 " << m_pcRABA->GetData(0) << std::endl;
        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 and 2
        m_pcRABA->SetData(1, RobotIdStrToInt(GetId()));
        m_pcRABA->SetData(2, m_cProprioceptiveFeatureVector.GetValue());
    }
    else
    {
        //m_pcRABA->ClearData();
        m_pcRABA->SetData(0, RobotIdStrToInt(GetId()));
        m_pcRABA->SetData(1, m_cProprioceptiveFeatureVector.GetValue());
    }

    /* Listen for broadcasted feature vectors from neighbours and then assimilate them */
    Sense(1.0f);

    if(m_fInternalRobotTimer > 450.0 && listFVsSensed.size() >= 3)
    {
        //std::cout << "RobotTimer " << m_fInternalRobotTimer << " Id. " << RobotIdStrToInt(GetId()) << std::endl;
        crminAgent->SimulationStepUpdatePosition(m_fInternalRobotTimer, &listFVsSensed, &listDetailedInformationFVsSensed);
    }
}

/****************************************/
/****************************************/

bool myfunction (CCI_RangeAndBearingSensor::SPacket i, CCI_RangeAndBearingSensor::SPacket j) { return (i.Range < j.Range); }


void CEPuckHomSwarm::Sense(Real m_fProbForget)
{
    if(m_fProbForget < 1.0f)
    {
        std:cerr << "History of FVs still to be implemented";
        exit(-1);
    }

    /* Listen for broadcasted feature vectors from neighbours */
    listFVsSensed.clear();  listDetailedInformationFVsSensed.clear();

    const CCI_RangeAndBearingSensor::TReadings& tmp = m_pcRABS->GetReadings();


    CCI_RangeAndBearingSensor::TReadings tPackets = tmp;
    std::sort(tPackets.begin(), tPackets.end(), myfunction);

    for(size_t i = 0; i < tPackets.size(); ++i)
    {
        unsigned robotId, fv;
        if(tPackets[i].Data[0] == BEACON_SIGNAL) // data from a beacon  - get the next two bytes
        {
            robotId = tPackets[i].Data[1];
            fv      = tPackets[i].Data[2];
        }
        else
        {
            robotId = tPackets[i].Data[0];
            fv      = tPackets[i].Data[1];
        }

        UpdateFeatureVectorDistribution(listFVsSensed, listDetailedInformationFVsSensed, fv, robotId, m_fInternalRobotTimer);


        if(listFVsSensed.size() == 10u) // limit the number of robots observed to 10 (nearest robots) - to run the CRM
            break;
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

unsigned CEPuckHomSwarm::RobotIdStrToInt(std::string id)
{
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
