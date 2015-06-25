#include "bayesianinferencefeaturevector.h"
#include "assert.h"
#include <iostream>

/******************************************************************************/
/******************************************************************************/

#define MODELSTARTTIME 450.0 // used for your sliding window - computing features the old method with ComputeFeatureValues_Old()

/******************************************************************************/
/******************************************************************************/

unsigned CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES        = 6;
unsigned CBayesianInferenceFeatureVector::MAX_NUMBER_OF_FEATURES    = 15;
double   CBayesianInferenceFeatureVector::FEATURE_RANGE             = 15.0; //cm // was 60 cm for the old features

/******************************************************************************/
/******************************************************************************/

int CBayesianInferenceFeatureVector::m_iShortTimeWindowLength  = 0u;
int CBayesianInferenceFeatureVector::m_iMediumTimeWindowLength = 0u;
int CBayesianInferenceFeatureVector::m_iLongTimeWindowLength   = 0u;


/******************************************************************************/
/******************************************************************************/
#define ROBOTID_NOT_IN_SIGNAL          999
#define ROBOTSELFBEARING_NOT_IN_SIGNAL 999.0f
#define ROBOTSELFACCELERATION_NOT_IN_SIGNAL 999.0f
/******************************************************************************/
/******************************************************************************/

CBayesianInferenceFeatureVector::CBayesianInferenceFeatureVector()
{
    //m_pcListObservedRobots.clear();
}

/******************************************************************************/
/******************************************************************************/

CBayesianInferenceFeatureVector::~CBayesianInferenceFeatureVector()
{}

/******************************************************************************/
/******************************************************************************/

unsigned CBayesianInferenceFeatureVector::SimulationStep()
{
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        /* this loop is quadratic in computational complexity. we can improve on this.*/

        bool     b_ObservedRobotFound(false);
        unsigned u_ObservedRobotId = GetIdFromRABPacket(m_sSensoryData.m_RABSensorData, i);
        for (t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin(); it_listobrob != m_pcListObservedRobots.end(); ++it_listobrob)
        {
            if(u_ObservedRobotId == it_listobrob->m_unRobotId)
            {
                b_ObservedRobotFound = true;
                break; // each robot id is represented only once in the m_pcListObservedRobots list
            }
        }

        /* add the robot u_ObservedRobotId to the list of observed robots*/
        if (!b_ObservedRobotFound && u_ObservedRobotId != ROBOTID_NOT_IN_SIGNAL)
            m_pcListObservedRobots.push_back(BayesInference_ObservedRobots_FeatureVector((*this), m_sSensoryData.m_rTime, u_ObservedRobotId));
    }


    /*
    But when do we delete the observed robot from the list of observed robots, if it has not been observed for a long time.
    We don't delete it, but we can just refresh the priors perodically, as we would do for the observed robots
    */
    t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin();
    while (it_listobrob != m_pcListObservedRobots.end())
    {
        unsigned RefreshPriorsThreshold = 1000000u; /* For now we don't refresh it. In the future we might want to, specially since the distributions may not be stationary */
        if((it_listobrob->u_TimeSinceLastRefresh) > RefreshPriorsThreshold)
            it_listobrob->RefreshPriors();

        /*unsigned RobotId = it_listobrob->m_unRobotId;
        bool b_ObservedRobotFound(false);
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
        {
            if (RobotId == GetIdFromRABPacket(m_sSensoryData.m_RABSensorData, i))
            {
                b_ObservedRobotFound = true;
                break;
            }
        }
        if (!b_ObservedRobotFound)
        {
            it_listobrob = m_pcListObservedRobots.erase(it_listobrob);
            continue;
        }*/

        ++it_listobrob;
    }


    //printf("m_pcListObservedRobots.size() %d \n\n\n", m_pcListObservedRobots.size());

    ObservedRobotIDs.clear(); ObservedRobotFVs.clear();
    for (it_listobrob = m_pcListObservedRobots.begin(); it_listobrob != m_pcListObservedRobots.end(); ++it_listobrob)
    {

        // Only update the posterior distribution for robots you have new observations for
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
        {
            unsigned u_ObservedRobotId = GetIdFromRABPacket(m_sSensoryData.m_RABSensorData, i);
            if(u_ObservedRobotId == (it_listobrob->m_unRobotId))
            {
                //printf("it_listobrob->m_unRobotId %d \n\n\n", it_listobrob->m_unRobotId);

                assert(u_ObservedRobotId != ROBOTID_NOT_IN_SIGNAL);
                it_listobrob->ComputeFeatureValues();

                /*
                 *  WHEN DO WE REPORT ON ID AND FV OF OF OBSERVED ROBOT
                 *  AFTER FIXED NUMBER OF OBSERVATIONS?
                 *  VARIANCE OF DISTRIBUTION LESS THAN THRESHOLD
                 *  BASED ON BAYESIAN CREDIBLE REGIONS?
                 */

                // if((m_sSensoryData.m_rTime - it_listobrob->m_fTimeFirstObserved) >= MODELSTARTTIME)
                if(it_listobrob->max_posterior_variance < 0.1f)
                {
                    ObservedRobotIDs.push_back(it_listobrob->m_unRobotId);
                    ObservedRobotFVs.push_back(it_listobrob->GetValue());
                }
            }
        }
    }
}

/******************************************************************************/
/******************************************************************************/

unsigned CBayesianInferenceFeatureVector::GetIdFromRABPacket(CCI_RangeAndBearingSensor::TReadings& rab_packet, size_t rab_packet_index)
{
    if (rab_packet[rab_packet_index].Data[0] == m_sRobotData.BEACON_SIGNAL_MARKER)
    {
        if (rab_packet[rab_packet_index].Data[1] == m_sRobotData.VOTER_PACKET_MARKER || rab_packet[rab_packet_index].Data[1] == m_sRobotData.SELF_INFO_PACKET_MARKER)
        {
            return rab_packet[rab_packet_index].Data[2];
        }
        else
        {
            return ROBOTID_NOT_IN_SIGNAL;
        }
    }
    else
    {
        if (rab_packet[rab_packet_index].Data[0] == m_sRobotData.VOTER_PACKET_MARKER || rab_packet[rab_packet_index].Data[0] == m_sRobotData.SELF_INFO_PACKET_MARKER)
        {
            return rab_packet[rab_packet_index].Data[1];
        }
        else
        {
            return ROBOTID_NOT_IN_SIGNAL;
        }
    }
}

/******************************************************************************/
/******************************************************************************/

float CBayesianInferenceFeatureVector::GetAnguAccelerFromRABPacket(CCI_RangeAndBearingSensor::TReadings& rab_packet, size_t rab_packet_index, unsigned observerd_robot_id)
{
    size_t index_start = 0;
    if (rab_packet[rab_packet_index].Data[0] == m_sRobotData.BEACON_SIGNAL_MARKER)
        index_start = 1;


    if (rab_packet[rab_packet_index].Data[index_start] == m_sRobotData.SELF_INFO_PACKET_MARKER)
    {
        unsigned chk_observerd_robot_id = rab_packet[rab_packet_index].Data[index_start+1]; // the id comes after the message header.
        assert(chk_observerd_robot_id == observerd_robot_id);
        return (((Real)(rab_packet[rab_packet_index].Data[index_start+2]) / m_sRobotData.DATA_BYTE_BOUND_MARKER) * 2.0f) - 1.0f; // return [-1, +1]
    }

    return ROBOTSELFACCELERATION_NOT_IN_SIGNAL;
}

/******************************************************************************/
/******************************************************************************/

CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::BayesInference_ObservedRobots_FeatureVector(CBayesianInferenceFeatureVector &owner_class,
                                                                                                                          Real TimeFirstObserved, unsigned ObservedRobotId):
    owner(owner_class), m_fTimeFirstObserved(TimeFirstObserved), m_unRobotId(ObservedRobotId), m_unValue(0)
{
    average_angularacceleration = (0.0f);

    m_pfFeatureValues        = new Real[CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES];
    m_pfAllFeatureValues     = new Real[CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES];


    for(unsigned int i = 0; i < CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES; i++)
    {
        m_pfFeatureValues[i]         = 0.0;
    }

    f_likelihood_variance = 2.0f;

    /************************************************************************************/
    m_fEstimated_Dist_ShortTimeWindow = 0.0f; m_fEstimated_Dist_MediumTimeWindow = 0.0f; m_fEstimated_Dist_LongTimeWindow = 0.0f;

    std::cout << "m_iShortTimeWindowLength " << m_iShortTimeWindowLength << std::endl;

    vec_RobPos_ShortRangeTimeWindow.resize(owner.m_iShortTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(owner.m_iMediumTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(owner.m_iLongTimeWindowLength);
    /************************************************************************************/

     /* Lets initialise the priors */
     RefreshPriors();
}

/******************************************************************************/
/******************************************************************************/

/* Use the copy constructor since your structure has raw pointers and push_back the structure instance to list will just copy the pointer value */
CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::BayesInference_ObservedRobots_FeatureVector(const BayesInference_ObservedRobots_FeatureVector &ClassToCopy):
    owner(ClassToCopy.owner), m_fTimeFirstObserved(ClassToCopy.m_fTimeFirstObserved), m_unRobotId(ClassToCopy.m_unRobotId), m_unValue(0)
{
    average_angularacceleration = ClassToCopy.average_angularacceleration;

    m_pfFeatureValues         = new Real[CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES];
    m_pfAllFeatureValues      = new Real[CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES];


    for(unsigned int i = 0; i < CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES; i++)
    {
        m_pfFeatureValues[i]         = 0.0;
    }


    f_likelihood_variance = ClassToCopy.f_likelihood_variance;

    /************************************************************************************/
    /* Keeping track of neighbours at different time scales*/
    m_fEstimated_Dist_ShortTimeWindow = 0.0f; m_fEstimated_Dist_MediumTimeWindow = 0.0f; m_fEstimated_Dist_LongTimeWindow = 0.0f;

    vec_RobPos_ShortRangeTimeWindow.resize(owner.m_iShortTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(owner.m_iMediumTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(owner.m_iLongTimeWindowLength);
    /************************************************************************************/

    /* Lets initialise the priors */
    RefreshPriors();
}

/******************************************************************************/
/******************************************************************************/

CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::~BayesInference_ObservedRobots_FeatureVector()
{
    std::cout << "deleting " << std::endl;

    delete m_pfFeatureValues;
    delete m_pfAllFeatureValues;

    vec_RobPos_ShortRangeTimeWindow.clear();
    vec_RobPos_MediumRangeTimeWindow.clear();
    vec_RobPos_LongRangeTimeWindow.clear();
}

/******************************************************************************/
/******************************************************************************/

void CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::ComputeFeatureValues()
{
    // We are updating the priors now. Increment the time since we last refreshed the priors.
    u_TimeSinceLastRefresh++;

    EstimateOdometry(); // gets observation of average_angularacceleration and of distance travelled by robot in last 1s, 5s, and 10s

    FEATURE_RANGE = 30.0f; // cm
    Real DistToNearestNbr, CoM_closerangenbrs = 0.0f, CoM_farrangenbrs = 0.0f;

    /*
     * Remember that CountNeighbors returns -1 if the robot was not observed at the current time-step
    */

    Real  f_unCloseRangeNbrCount = CountNeighbors(0.0, FEATURE_RANGE/2.0f, DistToNearestNbr, CoM_closerangenbrs);
    Real  f_unFarRangeNbrCount   = CountNeighbors(FEATURE_RANGE/2.0f, FEATURE_RANGE, DistToNearestNbr, CoM_farrangenbrs);


    /* we need to be able to observe the robot to make any inference */

    if (f_unCloseRangeNbrCount != -1.0f)
    {
        assert(f_unFarRangeNbrCount != -1.0f); // just to be sure that the neighbour was observed. if this assertion fails there is a bug in the code

        /*Update priors*/

        /* presence of neighbours in close range - is the robot part of a small aggregate */
        CoM_closerangenbrs /= (FEATURE_RANGE/2.0f); // normalise to [0, 1]
        sensclosePrior_Gaussian_mu = ((sensclosePrior_Gaussian_mu / sensclosePrior_Gaussian_variance) + (CoM_closerangenbrs / f_likelihood_variance)) /
                                      (1.0f/sensclosePrior_Gaussian_variance + 1.0f/f_likelihood_variance);
        sensclosePrior_Gaussian_variance = 1.0f / (1.0f/sensclosePrior_Gaussian_variance + 1.0f/f_likelihood_variance);






        /* presence of neighbours in long range - is the robot part of a large aggregate */
        CoM_farrangenbrs /= (FEATURE_RANGE); // normalise to [0, 1]
        sensfarPrior_Gaussian_mu = ((sensfarPrior_Gaussian_mu / sensfarPrior_Gaussian_variance) + (CoM_farrangenbrs / f_likelihood_variance)) /
                                    (1.0f / sensfarPrior_Gaussian_variance + 1.0f / f_likelihood_variance);
        sensfarPrior_Gaussian_variance = 1.0f / (1.0f / sensfarPrior_Gaussian_variance + 1.0f / f_likelihood_variance);


        max_posterior_variance = std::max(sensclosePrior_Gaussian_variance, sensfarPrior_Gaussian_variance);
    }



    /* we need to be able to observe the robot to make any inference */

    if (m_fEstimated_Dist_LongTimeWindow != -1.0f)
    {
        /*Update priors*/
        /* distance moved by robot in past 10s. is the robot moving a lot */

        m_fEstimated_Dist_LongTimeWindow /=  (m_iMediumTimeWindowLength * m_sRobotData.MaxLinearSpeed); // normalise to [0, 1]
        motPrior_Gaussian_mu = ((motPrior_Gaussian_mu / motPrior_Gaussian_variance) + (m_fEstimated_Dist_LongTimeWindow / f_likelihood_variance)) /
                                (1.0f / motPrior_Gaussian_variance + 1.0f / f_likelihood_variance);
        motPrior_Gaussian_variance = 1.0f / (1.0f / motPrior_Gaussian_variance + 1.0f / f_likelihood_variance);

        max_posterior_variance = std::max(max_posterior_variance, motPrior_Gaussian_variance);
    }



    Real COM_nbrs_tmp;
    bool neighbours_present = (CountNeighbors(0.0, FEATURE_RANGE, DistToNearestNbr, COM_nbrs_tmp) > 0.0f) ? true : false;

    /* we need to have motor observations to make an inference */
    if(average_angularacceleration != ROBOTSELFACCELERATION_NOT_IN_SIGNAL && m_fEstimated_Dist_MediumTimeWindow != -1.0f)
    {
        Real f_MotorOutput      = fabs(average_angularacceleration) * (m_fEstimated_Dist_MediumTimeWindow / (m_iMediumTimeWindowLength * m_sRobotData.MaxLinearSpeed));

        /* we need to discretise the data. we assume that a motor interaction occurs if f_MotorOutput exceeds 10% of max motor output */
        bool un_MotorOutput      = (f_MotorOutput > 0.10f)? 1u : 0u;

        /*Update priors*/
        /* Motor interactions in the presence of sensors */
        if (neighbours_present)
        {
            sensmotPrior_Beta_a = sensmotPrior_Beta_a + un_MotorOutput;
            sensmotPrior_Beta_b = sensmotPrior_Beta_b + 1u - un_MotorOutput;

            max_posterior_variance = std::max(max_posterior_variance,
                                               ((Real)(sensmotPrior_Beta_a * sensmotPrior_Beta_b)) /
                                               ((Real)((sensmotPrior_Beta_a + sensmotPrior_Beta_b)*
                                               (sensmotPrior_Beta_a + sensmotPrior_Beta_b)*
                                               (sensmotPrior_Beta_a + sensmotPrior_Beta_b + 1u))) );
        }

        /* Motor interactions in the absence of sensors */
        if (!neighbours_present)
        {
            nosensmotPrior_Beta_a = nosensmotPrior_Beta_a + un_MotorOutput;
            nosensmotPrior_Beta_b = nosensmotPrior_Beta_b + 1u - un_MotorOutput;

            max_posterior_variance = std::max(max_posterior_variance,
                                              ((Real)(nosensmotPrior_Beta_a * nosensmotPrior_Beta_b)) /
                                              ((Real)((nosensmotPrior_Beta_a + nosensmotPrior_Beta_b)*
                                               (nosensmotPrior_Beta_a + nosensmotPrior_Beta_b)*
                                               (nosensmotPrior_Beta_a + nosensmotPrior_Beta_b + 1u))) );
        }

        irrespsensmotPrior_Beta_a = irrespsensmotPrior_Beta_a + un_MotorOutput;
        irrespsensmotPrior_Beta_b = irrespsensmotPrior_Beta_b + 1u - un_MotorOutput;

        max_posterior_variance = std::max(max_posterior_variance,
                                          ((Real)(irrespsensmotPrior_Beta_a * irrespsensmotPrior_Beta_b)) /
                                          ((Real)((irrespsensmotPrior_Beta_a + irrespsensmotPrior_Beta_b)*
                                           (irrespsensmotPrior_Beta_a + irrespsensmotPrior_Beta_b)*
                                           (irrespsensmotPrior_Beta_a + irrespsensmotPrior_Beta_b + 1u))) );
    }



    /*if(CurrentStepNumber > m_iLongTimeWindowLength)
    printf("\n%f\t%d\t%d\t%f\t%f\t%f\t%f\t%f \n", owner.m_sSensoryData.m_rTime, m_unRobotId, owner.m_sSensoryData.m_unRobotId, m_fEstimated_Dist_ShortTimeWindow,
                                                                        m_fEstimated_Dist_MediumTimeWindow,
                                                                        m_fEstimated_Dist_LongTimeWindow, average_angularacceleration,DistToNearestNbr); */

    m_pfFeatureValues[0] = sensclosePrior_Gaussian_mu > 0.50f ? 1.0f : 0.0f;
    m_pfFeatureValues[1] = sensfarPrior_Gaussian_mu   > 0.50f ? 1.0f : 0.0f;
    m_pfFeatureValues[2] = (sensmotPrior_Beta_a       / (sensmotPrior_Beta_a       + sensmotPrior_Beta_b))       > 0.05f ? 1.0f : 0.0f;
    m_pfFeatureValues[3] = (nosensmotPrior_Beta_a     / (nosensmotPrior_Beta_a     + nosensmotPrior_Beta_b))     > 0.05f ? 1.0f : 0.0f;
    m_pfFeatureValues[4] = (irrespsensmotPrior_Beta_a / (irrespsensmotPrior_Beta_a + irrespsensmotPrior_Beta_b)) > 0.05f ? 1.0f : 0.0f;
    m_pfFeatureValues[5] = sensfarPrior_Gaussian_mu > 0.15f ? 1.0f : 0.0f;



    assert(CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES == 6);

    m_unValue = 0;
    for (unsigned int i = 0; i < CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES; i++)
        m_unValue += (unsigned)m_pfFeatureValues[i] * (1 << i);
}

/******************************************************************************/
/******************************************************************************/

Real CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::CountNeighbors(Real lb_sensor_range, Real hb_sensor_range, Real& dist_nearest_nbr, Real& CoM_nbrs)
{
    Real count_nbrs = 0.0f; // Use Real instead of unsigned so that we can return -1 if the robot is not observed at the current time-step.
    CoM_nbrs   = 0.0f;

    dist_nearest_nbr = 1000000u;



    /*
     * counting the number of neighbours to observedRobotId_1
     */
    unsigned observedRobotId_1 = m_unRobotId;
    Real observedRobotId_1_Range; CRadians observedRobotId_1_Bearing; Real observedRobotId_1_SelfBearingOrAngularAcceleration;

    bool b_DataAvailable = GetObservedRobotRangeBearing(observedRobotId_1_Range, observedRobotId_1_Bearing, observedRobotId_1_SelfBearingOrAngularAcceleration);
    // WHAT IF THE ROBOT IS NOT OBSERVED AT THE CURRENT TIME-STEP
    if(!b_DataAvailable)
        return -1.0f;


    for(size_t i = 0; i <  owner.m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        unsigned observedRobotId_2 = owner.GetIdFromRABPacket(owner.m_sSensoryData.m_RABSensorData, i);

        if(observedRobotId_1 == observedRobotId_2 || observedRobotId_2 == ROBOTID_NOT_IN_SIGNAL)
            continue;

        Real observedRobotId_2_Range       = owner.m_sSensoryData.m_RABSensorData[i].Range;
        CRadians observedRobotId_2_Bearing = owner.m_sSensoryData.m_RABSensorData[i].HorizontalBearing;

        CRadians diffBearing = observedRobotId_1_Bearing - observedRobotId_2_Bearing;

        Real Dist_ObsRobId1_ObsRobId2 = sqrt(observedRobotId_1_Range * observedRobotId_1_Range +
                                             observedRobotId_2_Range * observedRobotId_2_Range -
                                             2.0f * observedRobotId_1_Range * observedRobotId_2_Range * Cos(diffBearing));

        if((Dist_ObsRobId1_ObsRobId2 > lb_sensor_range) && (Dist_ObsRobId1_ObsRobId2 <= hb_sensor_range))
        {
            count_nbrs+=1.0f;
            CoM_nbrs += Dist_ObsRobId1_ObsRobId2;
        }

        if(Dist_ObsRobId1_ObsRobId2 < dist_nearest_nbr)
            dist_nearest_nbr = Dist_ObsRobId1_ObsRobId2;
    }


    // dont forget to add youself as neighbour of observedRobotId_1
    if((observedRobotId_1_Range > lb_sensor_range) && (observedRobotId_1_Range <= hb_sensor_range))
    {
        count_nbrs+=1.0f;
        CoM_nbrs  += observedRobotId_1_Range;

    }

    if(observedRobotId_1_Range < dist_nearest_nbr)
        dist_nearest_nbr = observedRobotId_1_Range;

    if(dist_nearest_nbr == 1000000u)
        dist_nearest_nbr = 101u; // max range is 100 cm. if no neighbours were detected set yo 101u

    if (count_nbrs > 0.0f)
        CoM_nbrs = CoM_nbrs / count_nbrs;

    return count_nbrs;
}


/******************************************************************************/
/******************************************************************************/

template <typename T> Real sgn(T val)
{
    /* val >= 0: return 1 else return -1  */
    return (Real)((T(0) <= val) - (val < T(0)));
}

template <typename T> Real diff_angle(T estimated_heading, T prev_estimated_heading)
{
    Real arg = fmod(estimated_heading-prev_estimated_heading, 360.00f);
    if (arg < 0.0f )  arg  = arg + 360.00f;
    if (arg > 180.0f) arg  = arg - 360.00f;
    return (-arg);
}


/******************************************************************************/
/******************************************************************************/

void CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::EstimateOdometry()
{
    Real observedRobotId_1_Range; CRadians observedRobotId_1_Bearing; Real observedRobotId_1_SelfBearingOrAngularAcceleration;

    bool b_DataAvailable = GetObservedRobotRangeBearing(observedRobotId_1_Range, observedRobotId_1_Bearing, observedRobotId_1_SelfBearingOrAngularAcceleration);
    // assert(b_DataAvailable); // WHAT IF THE ROBOT IS NOT OBSERVED AT THE CURRENT TIME-STEP


    //assert(observedRobotId_1_SelfBearingOrAngularAcceleration != ROBOTSELFACCELERATION_NOT_IN_SIGNAL);
    average_angularacceleration = observedRobotId_1_SelfBearingOrAngularAcceleration;


    /*
     * Computing angle rotated by robot in one tick
    */
    CRadians delta_orientation = CRadians(owner.m_sRobotData.seconds_per_iterations * ((-owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed)
                                                                                       / (owner.m_sRobotData.INTERWHEEL_DISTANCE*100.0f)));

    /*if (owner.m_sSensoryData.m_unRobotId == 0u)
        printf("%f\t",ToDegrees(delta_orientation).GetValue());*/


    Real step = owner.m_sSensoryData.m_rTime - m_fTimeFirstObserved;



    if(step < 0.0f)
    {
        printf( " owner.m_sSensoryData.m_rTime %f  m_fTimeFirstObserved %f \n\n", owner.m_sSensoryData.m_rTime, m_fTimeFirstObserved );
    }
    assert(step >= 0.0f);


    std::cout << " vec_RobPos_ShortRangeTimeWindow.size() " << vec_RobPos_ShortRangeTimeWindow.size() << " vec_RobPos_MediumRangeTimeWindow.size() " << vec_RobPos_MediumRangeTimeWindow.size() << " vec_RobPos_LongRangeTimeWindow.size() " << vec_RobPos_LongRangeTimeWindow.size() << std::endl;


    /*TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                           vec_RobPos_ShortRangeTimeWindow, b_DataAvailable);
    TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                            vec_RobPos_MediumRangeTimeWindow, b_DataAvailable);
    TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                            vec_RobPos_LongRangeTimeWindow, b_DataAvailable);*/
}

/******************************************************************************/
/******************************************************************************/

Real CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::TrackRobotDisplacement(Real step, Real observedRobotId_1_Range, CRadians observedRobotId_1_Bearing, CRadians delta_orientation, std::vector<RobotRelativePosData>& displacement_vector, bool b_DataAvailable)
{
    Real displacement = 0.0f;

    /*
     * Returns magnitude of displacement vector of the robot in the pre-specified time interval. If the robot is unobservable at the end of the pre-specified time interval, the function returns -1.
     */
    if(step < (Real)displacement_vector.size())
    {

        for (size_t t = 0 ; t < (unsigned)(step); ++t)
        {
            displacement_vector[t].TimeSinceStart++;

            CRadians prev_orientation = displacement_vector[t].NetRotationSinceStart;

            Real rX = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Cos(prev_orientation + delta_orientation / (2.0f));
            Real rY = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Sin(prev_orientation + delta_orientation / (2.0f));

            displacement_vector[t].NetTranslationSinceStart += CVector2(rX, rY);
            displacement_vector[t].NetRotationSinceStart    += delta_orientation;
        }

        displacement_vector[(unsigned)(step)].b_DataAvailable = b_DataAvailable;
        if (displacement_vector[(unsigned)(step)].b_DataAvailable)
        {
            displacement_vector[(unsigned)(step)].Range_At_Start           = observedRobotId_1_Range;
            displacement_vector[(unsigned)(step)].Bearing_At_Start         = observedRobotId_1_Bearing;
            displacement_vector[(unsigned)(step)].Pos_At_Start.Set(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing),
                                                                   observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
        }
        else
        {
            displacement_vector[(unsigned)(step)].Range_At_Start           = -1.0f;
            displacement_vector[(unsigned)(step)].Bearing_At_Start.SetValue(-1.0f);
            displacement_vector[(unsigned)(step)].Pos_At_Start.Set(-1.0f, -1.0f);
        }


        displacement_vector[(unsigned)(step)].TimeSinceStart           = 0.0f;
        displacement_vector[(unsigned)(step)].NetTranslationSinceStart.Set(0.0f, 0.0f);
        displacement_vector[(unsigned)(step)].NetRotationSinceStart.SetValue(0.0f);
    }
    else
    {
        for (size_t t = 0 ; t < displacement_vector.size(); ++t)
        {
            displacement_vector[t].TimeSinceStart++;

            CRadians prev_orientation = displacement_vector[t].NetRotationSinceStart;

            Real rX = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Cos(prev_orientation + delta_orientation / (2.0f));
            Real rY = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed_prev) / 2.0f) * Sin(prev_orientation + delta_orientation / (2.0f));

            displacement_vector[t].NetTranslationSinceStart += CVector2(rX, rY);
            displacement_vector[t].NetRotationSinceStart    += delta_orientation;
        }


        size_t t = ((unsigned)(step)%displacement_vector.size());


        if(b_DataAvailable)
        {
            CVector2 tmp_pos           = CVector2(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing), observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
            CVector2 tmp_pos_rot       = tmp_pos.Rotate(displacement_vector[t].NetRotationSinceStart);
            CVector2 pos_rot_trans     = tmp_pos_rot + displacement_vector[t].NetTranslationSinceStart;

            CVector2 Pos_At_Start      = displacement_vector[t].Pos_At_Start;

            /*
             * Computing average displacement
             */
            displacement              = (pos_rot_trans - Pos_At_Start).Length();
        }
        else
        {
            /*
                We don't have the range and bearing observations of the robot to position it at the end of the pre-specified time interval. So we can't compute the displacement.
            */
            displacement              = -1.0f;
        }



        /* Preparing the start recorded data to be used at the end of the pre-specified time interval */
        displacement_vector[(unsigned)(step)].b_DataAvailable = b_DataAvailable;
        if (displacement_vector[(unsigned)(step)].b_DataAvailable)
        {
            displacement_vector[t].Range_At_Start           = observedRobotId_1_Range;
            displacement_vector[t].Bearing_At_Start         = observedRobotId_1_Bearing;
            displacement_vector[t].Pos_At_Start.Set(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing), observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
        }
        else
        {
            displacement_vector[t].Range_At_Start           = -1.0f;
            displacement_vector[t].Bearing_At_Start.SetValue(-1.0f);
            displacement_vector[t].Pos_At_Start.Set(-1.0f, -1.0f);
        }
        displacement_vector[t].TimeSinceStart = 0.0f;
        displacement_vector[t].NetTranslationSinceStart.Set(0.0f, 0.0f);
        displacement_vector[t].NetRotationSinceStart.SetValue(0.0f);
    }

    return displacement;
}

/******************************************************************************/
/******************************************************************************/

bool CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::GetObservedRobotRangeBearing(Real& observedRobotId_1_Range, CRadians& observedRobotId_1_Bearing,
                                                                                        Real&  observedRobotId_1_SelfBearingORAngularAcceleration)
{
    bool observedRobotFound(false);
    for(size_t i = 0; i <  owner.m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        if(m_unRobotId == owner.GetIdFromRABPacket(owner.m_sSensoryData.m_RABSensorData, i))
        {
            observedRobotId_1_Range   = owner.m_sSensoryData.m_RABSensorData[i].Range;
            observedRobotId_1_Bearing = owner.m_sSensoryData.m_RABSensorData[i].HorizontalBearing;
            observedRobotFound = true;

            observedRobotId_1_SelfBearingORAngularAcceleration = owner.GetAnguAccelerFromRABPacket(owner.m_sSensoryData.m_RABSensorData, i, m_unRobotId);

            if (m_sRobotData.OBSERVATION_MODE_TYPE != 3)
            {
                std::cerr << "Observation modes can be of one of three types";
                exit(-1);
            }

            break;
        }
    }

    return observedRobotFound;
}

/******************************************************************************/
/******************************************************************************/

