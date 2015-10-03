#include "homswarm_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <controllers/epuck_hom_swarm/epuck_hom_swarm.h>



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
      //m_cOutput << "# clock\trobot_id\trobot_fv\tobserving_robots\tattack\ttolerate\tundecided" << std::endl;
   }
   catch(CARGoSException& ex)
      THROW_ARGOSEXCEPTION_NESTED("Error parsing homswarm loop function.", ex);


    // counting the number of robots in the swarm
    u_num_epucks = 0u;
    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        u_num_epucks++;
    }

    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());
        cController.GetExperimentType().SetNumEPuckRobotsInSwarm(u_num_epucks);
    }
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::Reset()
{  /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   //m_cOutput << "# clock\trobot_id\trobot_fv\tobserving_robots\tattack\ttolerate\tundecided" << std::endl;
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


    /*CSpace::TMapPerType& m_cLights = GetSpace().GetEntitiesByType("light");
    for(CSpace::TMapPerType::iterator it = m_cLights.begin(); it != m_cLights.end(); ++it)
    {
        CLightEntity& cLight = *any_cast<CLightEntity*>(it->second);

        if((unsigned)(GetSpace().GetSimulationClock())%100 == 0)
            cLight.MoveTo(CVector3(), CQuaternion());

        cLight.GetInitPosition();
    }*/


    float start_firsttrans_sec = 500.0f, finish_firsttosecondtrans_sec = 1000.0f;


    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());

        if(GetSpace().GetSimulationClock() < start_firsttrans_sec * cController.m_sRobotDetails.iterations_per_second)
        { }

        /*transition robot ids from robot_ids_behav1 to robot_ids_behav2 */
        else if((GetSpace().GetSimulationClock() >= start_firsttrans_sec  * cController.m_sRobotDetails.iterations_per_second) &&
                (GetSpace().GetSimulationClock() < finish_firsttosecondtrans_sec  * cController.m_sRobotDetails.iterations_per_second))
        {
            Real time_between_robots_trans_behav = cController.GetExperimentType().time_between_robots_trans_behav * cController.m_sRobotDetails.iterations_per_second;

            Real m_fInternalRobotTimer = GetSpace().GetSimulationClock() - start_firsttrans_sec  * cController.m_sRobotDetails.iterations_per_second;

            if(cController.GetExperimentType().robot_ids_behav1.size() > 0u)
            {
                if(time_between_robots_trans_behav > 0.0f)
                {
                    if(((unsigned)m_fInternalRobotTimer)%((unsigned)time_between_robots_trans_behav) == 0)
                    {
                        cController.GetExperimentType().robot_ids_behav2.push_back(cController.GetExperimentType().robot_ids_behav1.front());
                        cController.GetExperimentType().robot_ids_behav1.pop_front();
                    }
                }
                else if(time_between_robots_trans_behav == 0.0f)
                {
                    while(cController.GetExperimentType().robot_ids_behav1.size() > 0u)
                    {
                        cController.GetExperimentType().robot_ids_behav2.push_back(cController.GetExperimentType().robot_ids_behav1.front());
                        cController.GetExperimentType().robot_ids_behav1.pop_front();
                    }
                }
            }
        }

        /* transition robot ids from robot_ids_behav2 back to robot_ids_behav1 */
        else if (GetSpace().GetSimulationClock() >= finish_firsttosecondtrans_sec  * cController.m_sRobotDetails.iterations_per_second)
        {
            Real time_between_robots_trans_behav = cController.GetExperimentType().time_between_robots_trans_behav * cController.m_sRobotDetails.iterations_per_second;

            Real m_fInternalRobotTimer = GetSpace().GetSimulationClock() - finish_firsttosecondtrans_sec  * cController.m_sRobotDetails.iterations_per_second;

//            if((((unsigned)m_fInternalRobotTimer)%((unsigned)time_between_robots_trans_behav) == 0) && (cController.GetExperimentType().robot_ids_behav2.size() > 0u))
//            {
//                cController.GetExperimentType().robot_ids_behav1.push_back(cController.GetExperimentType().robot_ids_behav2.front());
//                cController.GetExperimentType().robot_ids_behav2.pop_front();
//            }

            if(cController.GetExperimentType().robot_ids_behav2.size() > 0u)
            {
                if(time_between_robots_trans_behav > 0.0f)
                {
                    if(((unsigned)m_fInternalRobotTimer)%((unsigned)time_between_robots_trans_behav) == 0)
                    {
                        cController.GetExperimentType().robot_ids_behav1.push_back(cController.GetExperimentType().robot_ids_behav2.front());
                        cController.GetExperimentType().robot_ids_behav2.pop_front();
                    }
                }
                else if(time_between_robots_trans_behav == 0.0f)
                {
                    while(cController.GetExperimentType().robot_ids_behav2.size() > 0u)
                    {
                        cController.GetExperimentType().robot_ids_behav1.push_back(cController.GetExperimentType().robot_ids_behav2.front());
                        cController.GetExperimentType().robot_ids_behav2.pop_front();
                    }
                }
            }

        }
    }
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::PostStep()
{
    if(GetSpace().GetSimulationClock() <= 450.0)
        return;


//    m_cOutput << "Clock: " << GetSpace().GetSimulationClock() << "\t";
//    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
//    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
//    {
//        /* Get handle to e-puck entity and controller */
//        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
//        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());

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
        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());

        unsigned observed_rob_id = cController.RobotIdStrToInt();
        unsigned observed_rob_fv = cController.GetRobotFeatureVector();

        std::list<unsigned> list_Consensus_Tolerators, list_Consensus_Attackers;
        list_Consensus_Tolerators.clear(); list_Consensus_Attackers.clear();


        for(CSpace::TMapPerType::iterator it_ob = m_cEpucks.begin(); it_ob != m_cEpucks.end(); ++it_ob)
        {
            CEPuckEntity& cEPuck_Observers = *any_cast<CEPuckEntity*>(it_ob->second);
            CEPuckHomSwarm& cController_Observers = dynamic_cast<CEPuckHomSwarm&>(cEPuck_Observers.GetControllableEntity().GetController());

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
        for (int i = 0; i < u_num_epucks - list_Consensus_Tolerators.size(); ++i)
            m_cOutput << " -1 ";

        m_cOutput <<  "\t" << "Consensus_Attackers: ";
        for (std::list<unsigned>::iterator it_atkcon = list_Consensus_Attackers.begin(); it_atkcon != list_Consensus_Attackers.end(); ++it_atkcon)
            m_cOutput << (*it_atkcon) << " ";
        for (int i = 0; i < u_num_epucks - list_Consensus_Attackers.size(); ++i)
            m_cOutput << " -1 ";
        m_cOutput << std::endl;
    }






//    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
//    {
//       /* Get handle to e-puck entity and controller */
//       CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
//       CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());

//       unsigned observed_rob_id = cController.RobotIdStrToInt();
//       unsigned observed_rob_fv = cController.GetRobotFeatureVector();


//       unsigned total_observers = 0, attackers = 0, tolerators = 0, undecided = 0;
//       for(CSpace::TMapPerType::iterator it_ob = m_cEpucks.begin(); it_ob != m_cEpucks.end(); ++it_ob)
//       {
//           CEPuckEntity& cEPuck_Observers = *any_cast<CEPuckEntity*>(it_ob->second);
//           CEPuckHomSwarm& cController_Observers = dynamic_cast<CEPuckHomSwarm&>(cEPuck_Observers.GetControllableEntity().GetController());

//           unsigned observers_rob_id = cController_Observers.RobotIdStrToInt();

//           if(observed_rob_id == observers_rob_id) // can't observe yourself
//               continue;

//           t_listFVsSensed listFVsSensed = cController_Observers.GetListFVsSensed();
//           t_listMapFVsToRobotIds listMapFVsToRobotIds = cController_Observers.GetMapFVsSensed();

//           bool robot_observed(false);
//           for (t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin(); itd != listMapFVsToRobotIds.end(); ++itd)
//           {
//               if(itd->uRobotId == observed_rob_id) // Cheating if the FV is not computed proprioceptively. Although the FV could be then sent to the robot
//               {
//                   assert(robot_observed == false); // the observer can only observe the robot once in one time-step
//                   robot_observed = true;
//                   total_observers++;
//                   //break; // the observer can only observe the robot once in one time-step
//               }
//           }

//           if(robot_observed)
//               for (t_listFVsSensed::iterator itd = listFVsSensed.begin(); itd != listFVsSensed.end(); ++itd)
//               {
//                   if(itd->uFV == observed_rob_fv)
//                   {
//                       if(itd->uMostWantedState == 0)
//                           undecided++;
//                       else if(itd->uMostWantedState == 1)
//                           attackers++;
//                       else if(itd->uMostWantedState == 2)
//                           tolerators++;
//                       else
//                       {
//                            //LOGERR << "Not an attacker or tolerator. We can't be here, there's a bug! " << itd->uMostWantedState << std::endl;
//                            //UPDATE: This is not a bug. It is just that this observer has not observed enough another robots of the swarm to run the CRM and make a decision
//                       }

//                        break;
//                   }
//               }
//       }

//       m_cOutput << "Clock: " << GetSpace().GetSimulationClock() << "\t"
//                 << "Id: " << observed_rob_id << "\t"
//                 << "FV: " << observed_rob_fv << "\t"
//                 << "NumObservers: " << total_observers << "\t"
//                 << "NumTolerators: " << tolerators << "\t"
//                 << "NumAttackers: " << attackers << "\t"
//                 << "NumDontKnows: " << undecided << std::endl;
//    }
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
