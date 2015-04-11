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

CBehavior::SensoryData CBehavior::m_sSensoryData;
CBehavior::RobotData CBehavior::m_sRobotData;

/****************************************/
/****************************************/

CEPuckForaging::ExperimentToRun::ExperimentToRun() :
    SBehavior(SWARM_FORAGING),
    FBehavior(FAULT_NONE),
    id_FaultyRobot(-1) {}


void CEPuckForaging::ExperimentToRun::Init(TConfigurationNode& t_node)
{
    std::string swarmbehav, errorbehav;

    try
    {
        GetNodeAttribute(t_node, "swarm_behavior", swarmbehav);
        GetNodeAttribute(t_node, "fault_behavior", errorbehav);
        GetNodeAttribute(t_node, "id_faulty_robot", id_FaultyRobot);
    }
    catch(CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);
    }

    if (swarmbehav.compare("SWARM_AGGREGATION") == 0)
        SBehavior = SWARM_AGGREGATION;
    else if (swarmbehav.compare("SWARM_DISPERSION") == 0)
        SBehavior = SWARM_DISPERSION;
    else if (swarmbehav.compare("SWARM_FLOCKING") == 0)
        SBehavior = SWARM_FLOCKING;
    else if (swarmbehav.compare("SWARM_HOMING") == 0)
        SBehavior = SWARM_HOMING;
    else if (swarmbehav.compare("SWARM_FORAGING") == 0)
        SBehavior = SWARM_FORAGING;
    else
    {
        std::cerr << "invalid swarm behavior"; exit(-1);
    }


    if (errorbehav.compare("FAULT_NONE") == 0)
        FBehavior = FAULT_NONE;

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
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
    }

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
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
    }
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
    m_pcWheels(NULL),
    m_pcLEDs(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcProximity(NULL),
    m_pcLight(NULL),
    m_pcGround(NULL),
    m_pcRNG(NULL) {}

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
        m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
        m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
        m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
        m_pcProximity = GetSensor  <CCI_EPuckProximitySensor        >("epuck_proximity"    );
        m_pcLight     = GetSensor  <CCI_LightUpdatedSensor          >("light_updated"        );
        m_pcGround    = GetSensor  <CCI_GroundSensor                >("ground" );
        /*
       * Parse XML parameters
       */        
        /* Wheel turning */
        m_sExpRun.Init(GetNode(t_node, "experiment_run"));
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
        /* Controller state */
        m_sStateData.Init(GetNode(t_node, "state"));
    }
    catch(CARGoSException& ex)
    {
        THROW_ARGOSEXCEPTION_NESTED("Error initializing the e-puck foraging controller for robot \"" << GetId() << "\"", ex);
    }
    /*
    * Initialize other stuff
    */
    /* Create a random number generator. We use the 'argos' category so
      that creation, reset, seeding and cleanup are managed by ARGoS. */
    m_pcRNG = CRandom::CreateRNG("argos");
    Reset();

    CBehavior::m_sRobotData.MaxSpeed = m_sWheelTurningParams.MaxSpeed;
    CBehavior::m_sRobotData.iterations_per_second = 10.0f * 10.0f; /*10 ticks per second, and 10 interations per tick so dt=0.01s*/
    CBehavior::m_sRobotData.seconds_per_iterations * 1.0f / CBehavior::m_sRobotData.iterations_per_second;
    CBehavior::m_sRobotData.HALF_INTERWHEEL_DISTANCE = 0.053f * 0.5f;
    CBehavior::m_sRobotData.INTERWHEEL_DISTANCE  = 0.053f;
    CBehavior::m_sRobotData.WHEEL_RADIUS = 0.0205f;

    CBehavior::m_sRobotData.m_cNoTurnOnAngleThreshold   = ToRadians(CDegrees(10.0f)); //10.0 - straight to food spot; 35.0 spiral to food spot
    CBehavior::m_sRobotData.m_cSoftTurnOnAngleThreshold = ToRadians(CDegrees(70.0f));
}

/****************************************/
/****************************************/

void CEPuckForaging::ControlStep()
{
    if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_FORAGING)
        RunForagingExperiment();

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION)
    {
        m_vecBehaviors.clear();

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(25.0f); //range threshold in cm
        m_vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_DISPERSION)
    {
        m_vecBehaviors.clear();

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_FLOCKING)
    {
        m_vecBehaviors.clear();

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }
    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING)
    {
        m_vecBehaviors.clear();

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(100.0f); //range threshold in cm
        m_vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }


    CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_pcProximity->GetReadings(), m_pcLight->GetReadings(), m_pcGround->GetReadings(), m_pcRABS->GetReadings());
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
    m_pcLEDs->SetAllColors(CColor::RED);
    /* Clear up the last exploration result */
    m_eLastExplorationResult = LAST_EXPLORATION_NONE;
    m_pcRABA->ClearData();
    m_pcRABA->SetData(0, LAST_EXPLORATION_NONE);
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
            break;
        }
        case SStateData::STATE_EXPLORING:
        {
            //std::cout << "SStateData::STATE_EXPLORING " << std::endl;
            Explore(); // have we transitioned from explore?
            break;
        }
        case SStateData::STATE_BEACON:
        {
            //std::cout << "SStateData::STATE_BEACON " << std::endl;
            BecomeABeacon();
            break;
        }
        case SStateData::STATE_RESTING_AT_FOOD:
        {
            //std::cout << "SStateData::STATE_RESTING_AT_FOOD " << std::endl;
            RestAtFood();
            break;
        }
        case SStateData::STATE_RETURN_TO_NEST:
        {
            //std::cout << "SStateData::STATE_RETURN_TO_NEST " << std::endl;
            ReturnToNest();
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

        CHomingToFoodBeaconBehavior* pcHomingToFoodBeaconBehavior = new CHomingToFoodBeaconBehavior();
        m_vecBehaviors.push_back(pcHomingToFoodBeaconBehavior);

        CAntiPhototaxisBehavior* pcAntiPhototaxisBehavior = new CAntiPhototaxisBehavior();
        m_vecBehaviors.push_back(pcAntiPhototaxisBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sStateData.State == SStateData::STATE_RETURN_TO_NEST) // Perform exploration
    {
        m_vecBehaviors.clear();

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        //! Oscillation of take control between CPhototaxisBehavior and Disperse can cause robots to remain stuck at food source in RETURN_TO_NEST STATE.
        //! But this happends very rarely
        CPhototaxisBehavior* pCPhototaxisBehavior = new CPhototaxisBehavior();
        m_vecBehaviors.push_back(pCPhototaxisBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.05f);
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
        m_pcLEDs->SetAllColors(CColor::GREEN);
        m_sStateData.State = SStateData::STATE_EXPLORING;
        m_sStateData.TimeRested = 0;
    }
    else
    {
        ++m_sStateData.TimeRested;
        /* Be sure not to send the last exploration result multiple times */
        if(m_sStateData.TimeRested == 1)
        {
            m_pcRABA->SetData(0, LAST_EXPLORATION_NONE);
        }

        /*
       * Social rule: listen to what other people have found and modify
       * probabilities accordingly
       */
        const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
        for(size_t i = 0; i < tPackets.size(); ++i)
            switch(tPackets[i].Data[0])
            {
                case LAST_EXPLORATION_SUCCESSFUL:
                {
                    /* ... */
                    break;
                }
                case LAST_EXPLORATION_UNSUCCESSFUL:
                {
                    /* ... */
                    break;
                }
            }

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
        m_pcLEDs->SetAllColors(CColor::BLUE);
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
    /* Send out data with RABS that you are a beacon. Neighbouring robots will use this data to home in on your position */
    m_pcRABA->SetData(0, BEACON_ESTABLISHED);
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
            /*if(tPackets[i].Data[0] == BEACON_ESTABLISHED)
                std::cout << "Packet index " << i << " packet range to beacon " << tPackets[i].Range << "cm" << std::endl;*/
            /* tPackets[i].Range is in cm and bearing is normalized [-pi pi] */
            if(tPackets[i].Data[0] == BEACON_ESTABLISHED && tPackets[i].Range < 25.0f) // Each food spot has radius of 0.2m // a slightly higher threshold is chosen to be safe
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
        m_eLastExplorationResult = BEACON_ESTABLISHED;

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
        m_pcLEDs->SetAllColors(CColor::BLUE);
        m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
    }
    else if(bBecomeBeacon) /*Do we become a beacon */
    {
        /* Yes, we do! */
        m_sStateData.TimeExploringUnsuccessfully = 0;
        m_sStateData.TimeSearchingForPlaceInNest = 0;
        m_pcLEDs->SetAllColors(CColor::YELLOW);
        m_sStateData.State = SStateData::STATE_BEACON;
    }
    else if(bRestAtFood) /* So, do we rest at the food source  */
    {
        /* Yes, we do! */
        m_sStateData.TimeExploringUnsuccessfully = 0;
        m_sStateData.TimeSearchingForPlaceInNest = 0;
        m_pcLEDs->SetAllColors(CColor::RED);
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
            /* Yes, lets rest now */
            /* Tell people about the last exploration attempt */
            m_pcRABA->SetData(0, m_eLastExplorationResult);
            /* ... and switch to state 'resting' */
            m_pcLEDs->SetAllColors(CColor::RED);
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
