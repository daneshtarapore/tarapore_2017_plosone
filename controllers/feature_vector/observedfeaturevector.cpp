#include "observedfeaturevector.h"
#include "assert.h"
#include <iostream>

/******************************************************************************/
/******************************************************************************/

unsigned CObservedFeatureVector::NUMBER_OF_FEATURES        = 6;
unsigned CObservedFeatureVector::MAX_NUMBER_OF_FEATURES    = 15;
unsigned CObservedFeatureVector::NUMBER_OF_FEATURE_VECTORS = 0;
double   CObservedFeatureVector::FEATURE_RANGE             = 60.0; //cm

/******************************************************************************/
/******************************************************************************/

Real CObservedFeatureVector::m_fVelocityThreshold            = 0.0f;
Real CObservedFeatureVector::m_fAccelerationThreshold        = 0.0f;

Real CObservedFeatureVector::m_tAngularVelocityThreshold     = 0.0f;
Real CObservedFeatureVector::m_tAngularAccelerationThreshold = 0.0f;


int CObservedFeatureVector::m_iEventSelectionTimeWindow      = 0.0f;
int CObservedFeatureVector::m_iDistTravelledTimeWindow       = 0.0f;

// the time window is in ticks and spped is in cm / tick
Real CObservedFeatureVector::m_fSquaredDistThreshold    = 0.0f;
Real CObservedFeatureVector::m_fCumulativeDistThreshold = 0.0f;


/******************************************************************************/
/******************************************************************************/

CObservedFeatureVector::CObservedFeatureVector()
{
    NUMBER_OF_FEATURE_VECTORS = 1 << NUMBER_OF_FEATURES;

    // too small thresholded distance. robot moves very little in one control-cycle. better to intergrate distance over time and use threshold on that
    m_fVelocityThreshold            = 0.05f  * (m_sRobotData.MaxLinearSpeed); // max speed is 1 cm per control cycle
    m_fAccelerationThreshold        = 0.05f  * (m_sRobotData.MaxLinearAcceleration); // max change in speed is \[PlusMinus]1 cm per control cycle per control cycle

    m_tAngularVelocityThreshold     = 0.15f  * (m_sRobotData.MaxAngularSpeed); //Maximum angular speed is \[PlusMinus]21.621 degrees per control cycle
    m_tAngularAccelerationThreshold = 0.15f  * (m_sRobotData.MaxAngularAcceleration); //Maximum angular acceleation is \[PlusMinus]43.242 degrees per control cycle per control cycle


    m_iEventSelectionTimeWindow      = MODELSTARTTIME; // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    m_iDistTravelledTimeWindow       = 100; // keeping track of distance travelled by bot in last 100 time-steps

    // the time window is in ticks and spped is in cm / tick
    m_fSquaredDistThreshold =    (0.05f * (m_sRobotData.MaxLinearSpeed*(Real)m_iDistTravelledTimeWindow)) *
                                                              (0.05f * (m_sRobotData.MaxLinearSpeed*(Real)m_iDistTravelledTimeWindow));
    m_fCumulativeDistThreshold = (0.05f * (m_sRobotData.MaxLinearSpeed*(Real)m_iDistTravelledTimeWindow));


    m_pcListObservedRobots.clear();
}

/******************************************************************************/
/******************************************************************************/

CObservedFeatureVector::~CObservedFeatureVector()
{}

/******************************************************************************/
/******************************************************************************/

unsigned CObservedFeatureVector::SimulationStep()
{
    assert(m_tAngularVelocityThreshold > 0.0f);

//    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
//    {
//        if(m_sSensoryData.m_RABSensorData[i].Range <= sensor_range)
//            count_nbrs++;
//    }


//    for (t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin(); it_listobrob != m_pcListObservedRobots.end(); ++it_listobrob)
//    {
//        it_listobrob->ComputeFeatureValues();
//    }



}

/******************************************************************************/
/******************************************************************************/

CObservedFeatureVector::ObservedRobots_FeatureVector::ObservedRobots_FeatureVector(CObservedFeatureVector &owner): owner(owner)
{
    m_unValue  = 0;

    m_pfFeatureValues         = new Real[CObservedFeatureVector::NUMBER_OF_FEATURES];
    m_piLastOccuranceEvent    = new int [CObservedFeatureVector::NUMBER_OF_FEATURES];
    m_piLastOccuranceNegEvent = new int [CObservedFeatureVector::NUMBER_OF_FEATURES];

    m_pfAllFeatureValues     = new Real[CObservedFeatureVector::NUMBER_OF_FEATURES];


    for(unsigned int i = 0; i < CObservedFeatureVector::NUMBER_OF_FEATURES; i++)
    {
        m_piLastOccuranceEvent[i]    = 0;
        m_piLastOccuranceNegEvent[i] = 0;

        m_pfFeatureValues[i]         = 0.0;
    }

    m_unNbrsCurrQueueIndex = 0;

    m_unSumTimeStepsNbrsRange0to30 = 0;
    m_unSumTimeStepsNbrsRange30to60 = 0;

    m_punNbrsRange0to30AtTimeStep  = new unsigned int[owner.m_iEventSelectionTimeWindow];
    m_punNbrsRange30to60AtTimeStep = new unsigned int[owner.m_iEventSelectionTimeWindow];


    // keeping track of distance travelled by bot in last 100 time-steps
    m_unCoordCurrQueueIndex    = 0;

    m_fSquaredDistTravelled = 0.0;
    m_fCumulativeDistTravelled = 0.0f;

    m_pvecCoordAtTimeStep = new CVector2[owner.m_iDistTravelledTimeWindow];
    m_pfDistAtTimeStep    = new Real[owner.m_iDistTravelledTimeWindow];
}

/******************************************************************************/
/******************************************************************************/

CObservedFeatureVector::ObservedRobots_FeatureVector::~ObservedRobots_FeatureVector()
{
    delete m_pfFeatureValues;

    delete m_piLastOccuranceEvent;
    delete m_piLastOccuranceNegEvent;

    delete m_punNbrsRange0to30AtTimeStep;
    delete m_punNbrsRange30to60AtTimeStep;

    delete m_pvecCoordAtTimeStep;
    delete m_pfDistAtTimeStep;
}

/******************************************************************************/
/******************************************************************************/

void CObservedFeatureVector::ObservedRobots_FeatureVector::ComputeFeatureValues()
{
//    unsigned  unCloseRangeNbrCount = CountNeighbors(FEATURE_RANGE/2.0f);
//    unsigned  unFarRangeNbrCount   = CountNeighbors(FEATURE_RANGE) - unCloseRangeNbrCount;

//    bool neighbours_present = ((unCloseRangeNbrCount + unFarRangeNbrCount) > 0) ? true : false;


//    int CurrentStepNumber = m_sSensoryData.m_rTime;

//    // Feature (from LS to MS bits in FV)
//    // Sensors
//    //1st: set if bot has atleast one neighbor in range 0-30cm in the majority of of past X time-steps
//    //2nd: set if bot has atleast one neighbor in range 30-60cm in the majority of of past X time-steps
//    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
//    {
//        // decision based on the last X time-steps
//        if(m_unSumTimeStepsNbrsRange0to30 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
//            m_pfAllFeatureValues[0] = 1.0;
//        else
//            m_pfAllFeatureValues[0] = 0.0;

//        if(m_unSumTimeStepsNbrsRange30to60 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
//            m_pfAllFeatureValues[1] = 1.0;
//        else
//            m_pfAllFeatureValues[1] = 0.0;

//        // removing the fist entry of the moving time window  from the sum
//        m_unSumTimeStepsNbrsRange0to30  -=  m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex];
//        m_unSumTimeStepsNbrsRange30to60 -=  m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex];
//    }

//    // adding new values into the queue
//    if (unCloseRangeNbrCount > 0)
//    {
//        m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
//        m_unSumTimeStepsNbrsRange0to30++;
//    }
//    else
//        m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

//    if (unFarRangeNbrCount > 0)
//    {
//        m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
//        m_unSumTimeStepsNbrsRange30to60++;
//    }
//    else
//        m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex] = 0;


//    m_unNbrsCurrQueueIndex = (m_unNbrsCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;


//    // Sensors-motor interactions
//    // Set if the occurance of the following event, atleast once in time window X
//    // 3rd: distance to nbrs 0-6 && change in angular acceleration
//    // 4th: no neighbors detected  && change in angular acceleration
//    if(neighbours_present &&
//            (m_sSensoryData.AngularAcceleration > m_tAngularAccelerationThreshold ||
//             m_sSensoryData.AngularAcceleration < -m_tAngularAccelerationThreshold))
//    {
//        m_piLastOccuranceEvent[2] = CurrentStepNumber;
//    }

//    if(!neighbours_present &&
//            (m_sSensoryData.AngularAcceleration > m_tAngularAccelerationThreshold ||
//             m_sSensoryData.AngularAcceleration < -m_tAngularAccelerationThreshold))
//    {
//        m_piLastOccuranceEvent[3] = CurrentStepNumber;
//    }

//    for(unsigned int featureindex = 2; featureindex <=3; featureindex++)
//    {
//        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
//        {
//            m_pfAllFeatureValues[featureindex] = 1.0;
//        }
//        else
//        {
//            m_pfAllFeatureValues[featureindex] = 0.0;
//        }
//    }


//    // Motors
//    //5th: distance travelled by bot in past 100 time-steps. Higher than 5% of max-possible distance travelled is accepted as feature=1.
//    CVector2 vecAgentPos = ObservedRobots_FeatureVector::m_sSensoryData.pos;

//    m_fCumulativeDistTravelled += ObservedRobots_FeatureVector::m_sSensoryData.dist;

//    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
//    {
//        // distance travelled in last 100 time-steps
//        m_fSquaredDistTravelled = (vecAgentPos.GetX() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetX()) *
//                (vecAgentPos.GetX() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetX())  +
//                (vecAgentPos.GetY() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetY()) *
//                (vecAgentPos.GetY() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetY());


//        //pos_ref = m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex];


//        // removing the distance travelled in the first time-step of the moving time-window from the queue
//        m_fCumulativeDistTravelled -= m_pfDistAtTimeStep[m_unCoordCurrQueueIndex];


//        // decision based on distance travelled in the last 100 time-steps
//        if(m_fCumulativeDistTravelled >= m_fCumulativeDistThreshold)
//            m_pfAllFeatureValues[4] = 1.0;
//        else
//            m_pfAllFeatureValues[4] = 0.0;

//        /*
//        if(m_fSquaredDistTravelled >= m_fSquaredDistThreshold)
//            m_pfAllFeatureValues[4] = 1.0;
//        else
//            m_pfAllFeatureValues[4] = 0.0;*/


//    }

//    //std::cout << " cumulative distance " << m_fCumulativeDistTravelled << std::endl;
//    //std::cout << " LinearSpeed " <<  m_sSensoryData.LinearSpeed << " angular speed " << m_sSensoryData.AngularSpeed << std::endl;
//    //std::cout << " LinearAcceleration " <<  m_sSensoryData.LinearAcceleration << " angular acceleration " << m_sSensoryData.AngularAcceleration << std::endl;

//    // adding new coordinate values into the queue
//    m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex] = vecAgentPos;

//    // adding distance travelled at last time-step into queue
//    m_pfDistAtTimeStep[m_unCoordCurrQueueIndex] = ObservedRobots_FeatureVector::m_sSensoryData.dist;

//    m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;


//    if((ObservedRobots_FeatureVector::m_sSensoryData.AngularAcceleration > m_tAngularAccelerationThreshold ||
//        ObservedRobots_FeatureVector::m_sSensoryData.AngularAcceleration < -m_tAngularAccelerationThreshold))
//    {
//         m_piLastOccuranceEvent[5] = CurrentStepNumber;
//    }


//    // 6th: set if the robot changed its angular speed (per control cycle) atleast once in the past time-window.
//    if ((CurrentStepNumber - m_piLastOccuranceEvent[5]) <= m_iEventSelectionTimeWindow)
//    {
//        m_pfAllFeatureValues[5] = 1.0;
//    }
//    else
//    {
//        m_pfAllFeatureValues[5] = 0.0;
//    }

//    //6th: linear speed, higher than 5% of max. speed is accepted as feature=1 OR angular speed higher than 5% of max. angular speed is accepted as feature=1
//    /*m_pfAllFeatureValues[5] = (m_sSensoryData.LinearSpeed  >= m_fVelocityThreshold) ? 1.0:0.0;*/


//    // adding the selected features into the feature vector
//    for(size_t i = 0; i <  NUMBER_OF_FEATURES; ++i)
//        m_pfFeatureValues[i] = m_pfAllFeatureValues[i];


//    m_unValue = 0;
//    for (unsigned int i = 0; i < m_unLength; i++)
//        m_unValue += (unsigned int)m_pfFeatureValues[i] * (1 << i);
}

/******************************************************************************/
/******************************************************************************/

unsigned CObservedFeatureVector::ObservedRobots_FeatureVector::CountNeighbors(Real sensor_range)
{
    unsigned count_nbrs = 0;
    for(size_t i = 0; i <  owner.m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        if(owner.m_sSensoryData.m_RABSensorData[i].Range <= sensor_range)
            count_nbrs++;
    }

    return count_nbrs;
}

/******************************************************************************/
/******************************************************************************/

void CObservedFeatureVector::ObservedRobots_FeatureVector::PrintFeatureDetails()
{
    int CurrentStepNumber = (int) owner.m_sSensoryData.m_rTime;

    std::cout << "Step: " << CurrentStepNumber << " TimeSteps_NbrsInRange0to3:  " << m_unSumTimeStepsNbrsRange0to30 <<
                 " TimeSteps_NbrsInRange3to6: " << m_unSumTimeStepsNbrsRange30to60 << " SquaredDistTravelled:  " << m_fSquaredDistTravelled <<
                 " SquaredDistThreshold: " << owner.m_fSquaredDistThreshold << " Linear speed: " << owner.m_sSensoryData.LinearSpeed <<
                 " Angular speed: " << owner.m_sSensoryData.AngularSpeed <<
                 " Linear acceleration: " << owner.m_sSensoryData.LinearAcceleration <<
                 " Angular acceleration: " << owner.m_sSensoryData.AngularAcceleration << std::endl;

}

/******************************************************************************/
/******************************************************************************/

