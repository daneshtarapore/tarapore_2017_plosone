#include "propriofeaturevector.h"
#include "assert.h"
#include <iostream>

/******************************************************************************/
/******************************************************************************/

unsigned int CProprioceptiveFeatureVector::NUMBER_OF_FEATURES        = 6;
unsigned int CProprioceptiveFeatureVector::MAX_NUMBER_OF_FEATURES    = 15;
unsigned int CProprioceptiveFeatureVector::NUMBER_OF_FEATURE_VECTORS = 0;
double       CProprioceptiveFeatureVector::FEATURE_RANGE             = 60.0; //cm

/******************************************************************************/
/******************************************************************************/

//CVector2 pos_ref(0.0f, 0.0f);


CProprioceptiveFeatureVector::CProprioceptiveFeatureVector()
{
    m_unValue  = 0;
    m_unLength = NUMBER_OF_FEATURES;

    NUMBER_OF_FEATURE_VECTORS = 1 << NUMBER_OF_FEATURES;

    m_pfFeatureValues         = new Real[m_unLength];
    m_piLastOccuranceEvent    = new int[m_unLength];
    m_piLastOccuranceNegEvent = new int[m_unLength];

    m_pfAllFeatureValues     = new Real[NUMBER_OF_FEATURES];


    m_iEventSelectionTimeWindow = MODELSTARTTIME; //1500;

    for(unsigned int i = 0; i < NUMBER_OF_FEATURES; i++)
    {
        m_piLastOccuranceEvent[i]    = 0;
        m_piLastOccuranceNegEvent[i] = 0;

        m_pfFeatureValues[i]         = 0.0;
    }


    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    m_unNbrsCurrQueueIndex = 0;

    m_unSumTimeStepsNbrsRange0to30 = 0;
    m_unSumTimeStepsNbrsRange30to60 = 0;

    m_punNbrsRange0to30AtTimeStep  = new unsigned int[m_iEventSelectionTimeWindow];
    m_punNbrsRange30to60AtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];


    // keeping track of distance travelled by bot in last 100 time-steps
    m_iDistTravelledTimeWindow = 100;
    m_unCoordCurrQueueIndex    = 0;

    m_fSquaredDistTravelled = 0.0;

    m_pvecCoordAtTimeStep = new CVector2[m_iDistTravelledTimeWindow];
}

/******************************************************************************/
/******************************************************************************/

CProprioceptiveFeatureVector::~CProprioceptiveFeatureVector()
{
    delete m_pfFeatureValues;

    delete m_piLastOccuranceEvent;
    delete m_piLastOccuranceNegEvent;

    delete m_punNbrsRange0to30AtTimeStep;
    delete m_punNbrsRange30to60AtTimeStep;

    delete m_pvecCoordAtTimeStep;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CProprioceptiveFeatureVector::GetValue() const
{
    return m_unValue;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CProprioceptiveFeatureVector::GetLength() const
{
    return m_unLength;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CProprioceptiveFeatureVector::SimulationStep()
{
    m_fVelocityThreshold            = 0.05  * (m_sRobotData.MaxLinearSpeed);
    m_fAccelerationThreshold        = 0.05  * (m_sRobotData.MaxLinearAcceleration);

    m_tAngularVelocityThreshold     = 0.032  * (m_sRobotData.MaxAngularSpeed);
    m_tAngularAccelerationThreshold = 0.032  * (m_sRobotData.MaxAngularAcceleration);

    // the time window is in ticks and is to be convered to seconds
    m_fSquaredDistThreshold = (0.05 * (m_sRobotData.MaxLinearSpeed*(Real)m_iDistTravelledTimeWindow*m_sRobotData.seconds_per_iterations)) *
            (0.05 * (m_sRobotData.MaxLinearSpeed*(Real)m_iDistTravelledTimeWindow*m_sRobotData.seconds_per_iterations));


    ComputeFeatureValues();
    m_unValue = 0;
    
    for (unsigned int i = 0; i < m_unLength; i++)
        m_unValue += (unsigned int)m_pfFeatureValues[i] * (1 << i);
}

/******************************************************************************/
/******************************************************************************/

void CProprioceptiveFeatureVector::ComputeFeatureValues()
{
    unsigned  unCloseRangeNbrCount = CountNeighbors(FEATURE_RANGE/2.0f);
    unsigned  unFarRangeNbrCount   = CountNeighbors(FEATURE_RANGE) - unCloseRangeNbrCount;

    bool neighbours_present = ((unCloseRangeNbrCount + unFarRangeNbrCount) > 0) ? true : false;


    int CurrentStepNumber = m_sSensoryData.m_rTime;

    // Feature (from LS to MS bits in FV)
    // Sensors
    //1st: set if bot has atleast one neighbor in range 0-30cm in the majority of of past X time-steps
    //2nd: set if bot has atleast one neighbor in range 30-60cm in the majority of of past X time-steps
    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
    {
        // decision based on the last X time-steps
        if(m_unSumTimeStepsNbrsRange0to30 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
            m_pfAllFeatureValues[0] = 1.0;
        else
            m_pfAllFeatureValues[0] = 0.0;

        if(m_unSumTimeStepsNbrsRange30to60 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
            m_pfAllFeatureValues[1] = 1.0;
        else
            m_pfAllFeatureValues[1] = 0.0;

        // removing the fist entry of the moving time window  from the sum
        m_unSumTimeStepsNbrsRange0to30  -=  m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex];
        m_unSumTimeStepsNbrsRange30to60 -=  m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex];
    }

    // adding new values into the queue
    if (unCloseRangeNbrCount > 0)
    {
        m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange0to30++;
    }
    else
        m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

    if (unFarRangeNbrCount > 0)
    {
        m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange30to60++;
    }
    else
        m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex] = 0;


    m_unNbrsCurrQueueIndex = (m_unNbrsCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;



    // Sensors-motor interactions
    // Set if the occurance of the following event, atleast once in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration

    if(neighbours_present &&
            (m_sSensoryData.AngularAcceleration > m_tAngularAccelerationThreshold ||
             m_sSensoryData.AngularAcceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[2] = CurrentStepNumber;
    }

    if(!neighbours_present &&
            (m_sSensoryData.AngularAcceleration > m_tAngularAccelerationThreshold ||
             m_sSensoryData.AngularAcceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[3] = CurrentStepNumber;
    }

    for(unsigned int featureindex = 2; featureindex <=3; featureindex++)
    {
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfAllFeatureValues[featureindex] = 1.0;
        }
        else
        {
            m_pfAllFeatureValues[featureindex] = 0.0;
        }
    }


    // Motors
    //5th: distance travelled by bot in past 100 time-steps. Higher than 5% of max-possible distance travelled is accepted as feature=1.
    CVector2 vecAgentPos = m_sSensoryData.pos;

    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
    {
        // distance travelled in last 100 time-steps
        m_fSquaredDistTravelled = (vecAgentPos.GetX() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetX()) *
                (vecAgentPos.GetX() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetX())  +
                (vecAgentPos.GetY() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetY()) *
                (vecAgentPos.GetY() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetY());


        //pos_ref = m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex];


        // decision based on distance travelled in the last 100 time-steps
        if(m_fSquaredDistTravelled >= m_fSquaredDistThreshold)
            m_pfAllFeatureValues[4] = 1.0;
        else
            m_pfAllFeatureValues[4] = 0.0;
    }

    // adding new coordinate values into the queue
    m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex] = vecAgentPos;
    m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;


    //6th: linear speed, higher than 5% of max. speed is accepted as feature=1 OR angular speed higher than 5% of max. angular speed is accepted as feature=1
    m_pfAllFeatureValues[5] = (m_sSensoryData.LinearSpeed  >= m_fVelocityThreshold  ||
                               m_sSensoryData.AngularSpeed >= m_tAngularVelocityThreshold) ? 1.0:0.0;


    // adding the selected features into the feature vector
    for(size_t i = 0; i <  NUMBER_OF_FEATURES; ++i)
        m_pfFeatureValues[i] = m_pfAllFeatureValues[i];
}

/******************************************************************************/
/******************************************************************************/

unsigned CProprioceptiveFeatureVector::CountNeighbors(Real sensor_range)
{
    unsigned count_nbrs = 0;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        if(m_sSensoryData.m_RABSensorData[i].Range <= sensor_range)
            count_nbrs++;
    }

    return count_nbrs;

}

/******************************************************************************/
/******************************************************************************/

void CProprioceptiveFeatureVector::PrintFeatureDetails()
{
    int CurrentStepNumber = (int) m_sSensoryData.m_rTime;

    std::cout << "Step: " << CurrentStepNumber << " TimeSteps_NbrsInRange0to3:  " << m_unSumTimeStepsNbrsRange0to30 <<
                 " TimeSteps_NbrsInRange3to6: " << m_unSumTimeStepsNbrsRange30to60 << " SquaredDistTravelled:  " << m_fSquaredDistTravelled <<
                 " SquaredDistThreshold: " << m_fSquaredDistThreshold << " Linear speed: " << m_sSensoryData.LinearSpeed << " Angular speed: " << m_sSensoryData.AngularSpeed <<
                 " Linear acceleration: " << m_sSensoryData.LinearAcceleration << " Angular acceleration: " << m_sSensoryData.AngularAcceleration <<
                 std::endl;


    //        std::cout << "Step: " << CurrentStepNumber << " DistTravelled:  " << sqrt(m_fSquaredDistTravelled) <<
    //                     " X " << m_sSensoryData.pos.GetX() << " Y " << m_sSensoryData.pos.GetY() <<
    //                     " X-ref " << pos_ref.GetX() << " Y-ref " << pos_ref.GetY() <<
    //                     " Linear speed: " << m_sSensoryData.LinearSpeed << " Angular speed: " << m_sSensoryData.AngularSpeed <<
    //                     std::endl;
}

/******************************************************************************/
/******************************************************************************/

//std::string CProprioceptiveFeatureVector::ToString()
//{
//    char pchTemp[4096];

//    if(NUMBER_OF_FEATURES == 6U)
//        sprintf(pchTemp, "Values - "
//                "TS_nbrs:0to3: %f - "
//                "TS_nbrs:3to6: %f - "
//                "TW450_dist0to6_angacc: %1.1f - "
//                "TW450_dist6_angacc: %1.1f - "
//                "DistTW100: %1.1f - "
//                "speed: %1.1f - fv: %u",

//                m_pfFeatureValues[0],
//                m_pfFeatureValues[1],
//                m_pfFeatureValues[2],
//                m_pfFeatureValues[3],
//                m_pfFeatureValues[4],
//                m_pfFeatureValues[5],
//                m_unValue);



//    return string(pchTemp);
//}

/******************************************************************************/
/******************************************************************************/

