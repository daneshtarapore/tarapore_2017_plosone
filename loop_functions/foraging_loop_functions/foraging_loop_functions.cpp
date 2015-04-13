#include "foraging_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <controllers/epuck_foraging/epuck_foraging.h>

/****************************************/
/****************************************/

CForagingLoopFunctions::CForagingLoopFunctions() :
   m_cForagingArenaSideX(1.2f, 2.2f),
   m_cForagingArenaSideY(-2.2f, 2.2f),
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
      m_cOutput << "# clock\twalking\tresting\tcollected_food\tenergy" << std::endl;
   }
   catch(CARGoSException& ex)
      THROW_ARGOSEXCEPTION_NESTED("Error parsing foraging loop function.", ex);



    try
     {
       TConfigurationNode& tForaging = GetNode(t_node, "experiment_run");

    }
    catch(CARGoSException& ex)
       THROW_ARGOSEXCEPTION_NESTED("Error parsing experiment_run parameters.", ex);

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
   m_cOutput << "# clock\tcollected_food" << std::endl;
   /* Distribute uniformly the items in the environment */
   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i)
      m_cFoodPos[i].Set(m_pcRNG->Uniform(m_cForagingArenaSideX),
                        m_pcRNG->Uniform(m_cForagingArenaSideY));
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
   if(c_position_on_plane.GetX() < -1.5f)
      return CColor::GRAY50;

   for(UInt32 i = 0; i < m_cFoodPos.size(); ++i)
      if((c_position_on_plane - m_cFoodPos[i]).SquareLength() < m_fFoodSquareRadius)
         return CColor::BLACK;

   return CColor::WHITE;
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
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CForagingLoopFunctions, "foraging_loop_functions")
