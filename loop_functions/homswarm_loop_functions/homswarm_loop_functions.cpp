#include "homswarm_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

#include </home/danesh/argos3-foraging/controllers/epuck_hom_swarm/epuck_hom_swarm.h>

/****************************************/
/****************************************/

CHomSwarmLoopFunctions::CHomSwarmLoopFunctions() :
   m_pcFloor(NULL),
   m_pcRNG(NULL)
{

//    // keeping track of distance travelled by bot in last 100 time-steps
//    m_iDistTravelledTimeWindow = 100;
//    m_unCoordCurrQueueIndex    = 0;
//    m_pvecCoordAtTimeStep = new CVector2[m_iDistTravelledTimeWindow];
//    CurrentStepNumber = 0;
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::Init(TConfigurationNode& t_node)
{
   try
    {
      TConfigurationNode& tParams = GetNode(t_node, "params");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();

      /* Get the output file name from XML */
      GetNodeAttribute(tParams, "output", m_strOutput);
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      m_cOutput << "# clock\trobot_id\trobot_fv\tobserving_robots\tattack\ttolerate\tundecided" << std::endl;
   }
   catch(CARGoSException& ex)
      THROW_ARGOSEXCEPTION_NESTED("Error parsing homswarm loop function.", ex);
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::Reset()
{  /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   m_cOutput << "# clock\trobot_id\trobot_fv\tobserving_robots\tattack\ttolerate\tundecided" << std::endl;
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::PreStep()
{
//    CVector2 cPos;
//    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");

//    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin();
//        it != m_cEpucks.end();
//        ++it) {
//       /* Get handle to foot-bot entity and controller */
//       CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);

//       cPos.Set(cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
//                cEPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
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

void CHomSwarmLoopFunctions::PostStep()
{
    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");

    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
       /* Get handle to e-puck entity and controller */
       CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
       CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());

       unsigned observed_rob_id = cController.RobotIdStrToInt(cEPuck.GetId());
       int observed_rob_fv = -1;

       unsigned total_observers = 0, attackers = 0, tolerators = 0, undecided = 0;
       for(CSpace::TMapPerType::iterator it_ob = m_cEpucks.begin(); it_ob != m_cEpucks.end(); ++it_ob)
       {
           CEPuckEntity& cEPuck_Observers = *any_cast<CEPuckEntity*>(it_ob->second);
           CEPuckHomSwarm& cController_Observers = dynamic_cast<CEPuckHomSwarm&>(cEPuck_Observers.GetControllableEntity().GetController());

           unsigned observers_rob_id = cController_Observers.RobotIdStrToInt(cEPuck_Observers.GetId());

           if(observed_rob_id == observers_rob_id) // can't observe yourself
               continue;

           t_listFVsSensed listFVsSensed = cController_Observers.GetListFVsSensed();
           t_listDetailedInfoFVsSensed listDetailedInformationFVsSensed = cController_Observers.GetDetailedListFVsSensed();

           bool robot_observed(false);
           for (t_listDetailedInfoFVsSensed::iterator itd = listDetailedInformationFVsSensed.begin(); itd != listDetailedInformationFVsSensed.end(); ++itd)
           {
               if(itd->uRobotId == observed_rob_id)
               {
                   robot_observed = true;
                   observed_rob_fv = itd->uFV;
                   total_observers++;
                   break; // the observer can only observe the robot once in one time-step
               }
           }

           if(robot_observed)
               for (t_listFVsSensed::iterator itd = listFVsSensed.begin(); itd != listFVsSensed.end(); ++itd)
               {
                   if(itd->uFV == observed_rob_fv)
                   {
                       if(itd->uMostWantedState == 0)
                           undecided++;
                       else if(itd->uMostWantedState == 1)
                           attackers++;
                       else if(itd->uMostWantedState == 2)
                           tolerators++;

                        break;
                   }
               }
       }

       m_cOutput << GetSpace().GetSimulationClock() << "\t"
                 << observed_rob_id << "\t"
                 << observed_rob_fv << "\t"
                 << total_observers << "\t"
                 << tolerators << "\t"
                 << attackers << "\t"
                 << undecided << std::endl;






    }
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::Destroy()
{
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CHomSwarmLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) // used to paint the floor by the floor entity
{
   return CColor::WHITE;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CHomSwarmLoopFunctions, "homswarm_loop_functions")
