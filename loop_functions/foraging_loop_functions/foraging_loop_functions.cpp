#include "foraging_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <controllers/epuck_foraging/epuck_foraging.h>

/****************************************/
/****************************************/

CForagingLoopFunctions::CForagingLoopFunctions() :
   m_cForagingArenaSideX(1.0f, 1.2f),
   m_cForagingArenaSideY(-1.2f, 1.2f),
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   m_unCollectedFood(0)
{
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Init(TConfigurationNode& t_node)
{
   try
    {
      TConfigurationNode& tForaging = GetNode(t_node, "foraging");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();
      /* Get the number of food items we want to be scattered from XML */
      UInt32 unFoodItems;
      GetNodeAttribute(tForaging, "items", unFoodItems);
      /* Get the number of food items we want to be scattered from XML */
      GetNodeAttribute(tForaging, "radius", m_fFoodSquareRadius);
      m_fFoodSquareRadius *= m_fFoodSquareRadius;
      /* Create a new RNG */
      m_pcRNG = CRandom::CreateRNG("argos");
      /* Distribute uniformly the items in the environment */
      for(UInt32 i = 0; i < unFoodItems; ++i)
         m_cFoodPos.push_back(
            CVector2(m_pcRNG->Uniform(m_cForagingArenaSideX),
                     m_pcRNG->Uniform(m_cForagingArenaSideY)));

      /* Get the output file name from XML */
      GetNodeAttribute(tForaging, "output", m_strOutput);
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      //m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
   }
   catch(CARGoSException& ex)
      THROW_ARGOSEXCEPTION_NESTED("Error parsing foraging loop function.", ex);
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Reset()
{
   /* Zero the counters */
   m_unCollectedFood = 0;
   /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   //m_cOutput << "# clock\tcollected_food" << std::endl;
   /* Distribute uniformly the items in the environment */
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i)
      m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::PreStep()
{
    /* Logic to pick and drop food items */
    /*
    * If a robot is in the nest, drop the food item // 'CHEATING' WITH LOOP FUNCTION
    * If a robot is on a food item, pick it         // 'CHEATING' WITH LOOP FUNCTION
    * Each robot can carry only one food item per time
    * Update: The collection and deposit of food is now handled in the robot's control loop
    */

    m_unCollectedFood = 0;

    /* Check whether a robot is on a food item */
    CSpace::TMapPerType& m_cEPucks = GetSpace().GetEntitiesByType("e-puck");

    for(CSpace::TMapPerType::iterator it = m_cEPucks.begin(); it != m_cEPucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckForaging& cController = dynamic_cast<CEPuckForaging&>(cEPuck.GetControllableEntity().GetController());

        /* Get food data */
        CEPuckForaging::SFoodData& sFoodData = cController.GetFoodData();
        m_unCollectedFood += sFoodData.TotalFoodItems;

    }

    /* Output stuff to file */
    m_cOutput << GetSpace().GetSimulationClock() << "\t"
              << m_unCollectedFood << std::endl;


//    Ground truth on distance traversed by robot
//    CVector2 cPos;
//    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");

//    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin();
//        it != m_cEpucks.end();
//        ++it) {
//       /* Get handle to foot-bot entity and controller */
//       CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);

//       cPos.Set(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
//                cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

//       std::cerr << " Pos X " << cPos.GetX() << " Y " << cPos.GetY() << std::endl;
//    }

//    CVector2 vecAgentPos = cPos;

//    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
//    {
//        // distance travelled in last 100 time-steps
//        Real m_fSquaredDistTravelled = (vecAgentPos.GetX() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetX()) *
//                                  (vecAgentPos.GetX() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetX())  +

//                                  (vecAgentPos.GetY() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetY()) *
//                                  (vecAgentPos.GetY() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetY());


//        // decision based on distance travelled in the last 100 time-steps
//        std::cout << "True distance travelled for step " << CurrentStepNumber << " is " << sqrt(m_fSquaredDistTravelled)*100.0f << std::endl;
//    }

//    // adding new coordinate values into the queue
//    m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex] = vecAgentPos;
//    m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;
//    CurrentStepNumber++;
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::Destroy()
{
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CForagingLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) // used to paint the floor by the floor entity
{
   if(c_position_on_plane.GetX() < -1.0f)
      return CColor::GRAY50;

   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i)
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius)
         return CColor::BLACK;

   return CColor::WHITE;
}

/****************************************/
/****************************************/


void CForagingLoopFunctions::PostStep()
{
    if(GetSpace().GetSimulationClock() <= 450.0)
        return;


//    m_cOutput << "Clock: " << GetSpace().GetSimulationClock() << "\t";
//    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
//    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
//    {
//        /* Get handle to e-puck entity and controller */
//        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
//        CEPuckForaging& cController = dynamic_cast<CEPuckForaging&>(cEPuck.GetControllableEntity().GetController());

//        unsigned observed_rob_id = cController.RobotIdStrToInt();
//        unsigned observed_rob_fv = cController.GetRobotFeatureVector();

//        m_cOutput << "Observer Robot Id: " << observed_rob_id << "  " << "Proprio. FV: " << observed_rob_fv << "\t" ;

//        for(size_t observed_index = 0; observed_index < cController.GetObservedFeatureVectors().ObservedRobotIDs.size(); ++observed_index)
//        {
//            m_cOutput << "Id: " << cController.GetObservedFeatureVectors().ObservedRobotIDs[observed_index] << "  "
//                      << "FV: " << cController.GetObservedFeatureVectors().ObservedRobotFVs[observed_index] << "\t";
//        }
//    }
//    m_cOutput << std::endl;
//    return;


    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckForaging& cController = dynamic_cast<CEPuckForaging&>(cEPuck.GetControllableEntity().GetController());

        unsigned observed_rob_id = cController.RobotIdStrToInt();
        unsigned observed_rob_fv = cController.GetRobotFeatureVector();

        std::list<unsigned> list_Consensus_Tolerators, list_Consensus_Attackers;
        list_Consensus_Tolerators.clear(); list_Consensus_Attackers.clear();


        for(CSpace::TMapPerType::iterator it_ob = m_cEpucks.begin(); it_ob != m_cEpucks.end(); ++it_ob)
        {
            CEPuckEntity& cEPuck_Observers = *any_cast<CEPuckEntity*>(it_ob->second);
            CEPuckForaging& cController_Observers = dynamic_cast<CEPuckForaging&>(cEPuck_Observers.GetControllableEntity().GetController());

            unsigned observers_rob_id = cController_Observers.RobotIdStrToInt();

            t_listConsensusInfoOnRobotIds listConsensusInfoOnRobotIds = cController_Observers.GetListConsensusInfoOnRobotIds();

            for (t_listConsensusInfoOnRobotIds ::iterator it_con = listConsensusInfoOnRobotIds.begin(); it_con != listConsensusInfoOnRobotIds.end(); ++it_con)
            {
                if(it_con->uRobotId == observed_rob_id && it_con->consensus_state == 1)
                    list_Consensus_Attackers.push_back(observers_rob_id);

                else if(it_con->uRobotId == observed_rob_id && it_con->consensus_state == 2)
                    list_Consensus_Tolerators.push_back(observers_rob_id);
            }
        }

        m_cOutput << "Clock: " << GetSpace().GetSimulationClock() << "\t"
                  << "Id: " << observed_rob_id << "\t"
                  << "FV: " << observed_rob_fv << "\t";

        m_cOutput << "Consensus_Tolerators: ";
        for (std::list<unsigned>::iterator it_tolcon = list_Consensus_Tolerators.begin(); it_tolcon != list_Consensus_Tolerators.end(); ++it_tolcon)
            m_cOutput << (*it_tolcon) << " ";
        for (int i = 0; i < 20 - list_Consensus_Tolerators.size(); ++i)
            m_cOutput << " -1 ";

        m_cOutput <<  "\t" << "Consensus_Attackers: ";
        for (std::list<unsigned>::iterator it_atkcon = list_Consensus_Attackers.begin(); it_atkcon != list_Consensus_Attackers.end(); ++it_atkcon)
            m_cOutput << (*it_atkcon) << " ";
        for (int i = 0; i < 20 - list_Consensus_Attackers.size(); ++i)
            m_cOutput << " -1 ";
        m_cOutput << std::endl;
    }
}

/****************************************/
/****************************************/

void CForagingLoopFunctions::PostExperiment()
{
    /* Writing the ids of the beacon robots */

    bool beaconspresent(false);

    m_cOutput << "BeaconRobotIds: " << "\t";

    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckForaging& cController = dynamic_cast<CEPuckForaging&>(cEPuck.GetControllableEntity().GetController());

        if(cController.GetStateData().State == CEPuckForaging::SStateData::STATE_BEACON)
        {
            beaconspresent = true;
            m_cOutput <<  cController.RobotIdStrToInt() << " ";
        }
    }

    if(!beaconspresent)
        m_cOutput << "-1 ";
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loop_functions")
