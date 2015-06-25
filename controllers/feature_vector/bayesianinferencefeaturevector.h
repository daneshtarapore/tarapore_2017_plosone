#ifndef BAYESIANINFERENCE_FEATUREVECTOR_H_
#define BAYESIANINFERENCE_FEATUREVECTOR_H_


/******************************************************************************/
/******************************************************************************/
#include <string>
#include <list>
#include <numeric>
/******************************************************************************/
/******************************************************************************/
#include <argos3/core/utility/math/vector2.h>

#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_updated_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

template <typename T> Real diff_angle(T estimated_heading, T prev_estimated_heading);

class CBayesianInferenceFeatureVector
{
public:

    struct RobotData
    {
        Real     MaxLinearSpeed;
        Real     MaxAngularSpeed;

        Real     MaxLinearAcceleration;
        Real     MaxAngularAcceleration;

        Real     iterations_per_second, seconds_per_iterations;
        Real     INTERWHEEL_DISTANCE, HALF_INTERWHEEL_DISTANCE;
        Real     WHEEL_RADIUS;

        size_t   BEACON_SIGNAL_MARKER;

        size_t   SELF_INFO_PACKET_MARKER;
        size_t   SELF_INFO_PACKET_FOOTER_MARKER;

        size_t   RELAY_FVS_PACKET_MARKER;
        size_t   RELAY_FVS_PACKET_FOOTER_MARKER;

        size_t   VOTER_PACKET_MARKER;
        size_t   VOTER_PACKET_FOOTER_MARKER;

        size_t   DATA_BYTE_BOUND_MARKER;

        size_t   OBSERVATION_MODE_TYPE;

        void SetLengthOdometryTimeWindows()
        {
            /* Length of time windows for observing neighbours of your neighbours and the distance they travel */
            m_iShortTimeWindowLength  = (unsigned)iterations_per_second;
            m_iMediumTimeWindowLength = (unsigned)iterations_per_second * 5u;
            m_iLongTimeWindowLength   = (unsigned)iterations_per_second * 10u;
        }
    };

    struct SensoryData
    {
       unsigned m_unRobotId;

       Real m_rTime;
       Real f_LeftWheelSpeed, f_RightWheelSpeed;
       Real f_LeftWheelSpeed_prev, f_RightWheelSpeed_prev;
       Real f_LeftWheelSpeed_prev_prev, f_RightWheelSpeed_prev_prev;

       CCI_EPuckProximitySensor::TReadings m_ProximitySensorData;
       CCI_LightUpdatedSensor::TReadings m_LightSensorData;
       CCI_GroundSensor::TReadings m_GroundSensorData;
       CCI_RangeAndBearingSensor::TReadings  m_RABSensorData;

       SensoryData()
       {
           m_rTime = 0.0f;
           f_LeftWheelSpeed = 0.0f; f_RightWheelSpeed = 0.0f; f_LeftWheelSpeed_prev = 0.0f; f_RightWheelSpeed_prev = 0.0f;
       }

       void SetSensoryData(unsigned RobId, Real time, CCI_EPuckProximitySensor::TReadings proximity, CCI_LightUpdatedSensor::TReadings light, CCI_GroundSensor::TReadings ground,
                           CCI_RangeAndBearingSensor::TReadings  rab, Real LeftWheelSpeed, Real RightWheelSpeed)
       {
           m_unRobotId = RobId;

           m_rTime = time;
           m_ProximitySensorData = proximity;
           m_LightSensorData = light;
           m_GroundSensorData = ground;
           m_RABSensorData = rab;

           f_LeftWheelSpeed_prev_prev = f_LeftWheelSpeed_prev; f_RightWheelSpeed_prev_prev = f_RightWheelSpeed_prev;
           f_LeftWheelSpeed_prev = f_LeftWheelSpeed; f_RightWheelSpeed_prev = f_RightWheelSpeed;
           f_LeftWheelSpeed = LeftWheelSpeed; f_RightWheelSpeed = RightWheelSpeed;
       }

       void SetSensoryData(unsigned RobId, Real time, CCI_EPuckProximitySensor::TReadings proximity, CCI_RangeAndBearingSensor::TReadings  rab, Real LeftWheelSpeed, Real RightWheelSpeed)
       {
           m_unRobotId = RobId;

           m_rTime = time;
           m_ProximitySensorData = proximity;
           m_RABSensorData = rab;

           f_LeftWheelSpeed_prev_prev = f_LeftWheelSpeed_prev; f_RightWheelSpeed_prev_prev = f_RightWheelSpeed_prev;
           f_LeftWheelSpeed_prev = f_LeftWheelSpeed; f_RightWheelSpeed_prev = f_RightWheelSpeed;
           f_LeftWheelSpeed = LeftWheelSpeed; f_RightWheelSpeed = RightWheelSpeed;
       }
    };

    struct RobotRelativePosData
    {
        Real     Range_At_Start;
        CRadians Bearing_At_Start;

        CVector2 Pos_At_Start, NetTranslationSinceStart;
        CRadians NetRotationSinceStart;

        Real     TimeSinceStart;

        bool     b_DataAvailable; // true if range and bearing data is available at the start of the pre-specified time interval (1s, 5s, 10s). False otherwise.
    };


    struct BayesInference_ObservedRobots_FeatureVector
    {
        BayesInference_ObservedRobots_FeatureVector(CBayesianInferenceFeatureVector &owner, Real TimeFirstObserved, unsigned ObservedRobotId);
        BayesInference_ObservedRobots_FeatureVector(const BayesInference_ObservedRobots_FeatureVector& ClassToCopy);
        ~BayesInference_ObservedRobots_FeatureVector();

        unsigned GetValue() const
        {
            return m_unValue;
        }

        void ComputeFeatureValues();
        Real CountNeighbors(Real lb_sensor_range, Real hb_sensor_range, Real &dist_nearest_nbr, Real &CoM_nbrs);
        void EstimateOdometry();
        Real TrackRobotDisplacement(Real step, Real observed_range, CRadians observed_bearing, CRadians self_delta_orientation, std::vector<RobotRelativePosData>& displacement_vector, bool b_DataAvailable);
        bool GetObservedRobotRangeBearing(Real& observedRobotId_1_Range, CRadians& observedRobotId_1_Bearing, Real &observedRobotId_1_SelfBearingORAngularAcceleration);

        void RefreshPriors()
        {
            /* Cleaning the slate */
            u_TimeSinceLastRefresh = 0u;


            /* Sensor motor interactions. Motor reactions in the presence of sensor input from nbrs, and in the absence of sensor input from nbrs. Associated with the belief  */
            sensmotPrior_Beta_a = 1u;   sensmotPrior_Beta_b = 1u;
            nosensmotPrior_Beta_a = 1u; nosensmotPrior_Beta_b = 1u;


            /* prior pertaining to actuation - i.e. distance travelled by robot in past 10s. Associated to the belief, is the robot moving large distances? */
            motPrior_Gaussian_mu = 0.5f; motPrior_Gaussian_variance = 4.0f;
            /* prior pertaining to motor reactions irrespective of sensor input. How does the robot react in a general sense */
            irrespsensmotPrior_Beta_a = 1u; irrespsensmotPrior_Beta_b = 1u;


            /* prior pertaining to sensing - i.e. number of neighbours in close and far proximity. Associated to the belief, is the robot part of a small aggregate?, and is the robot part of a large aggregate?  */
            sensclosePrior_Gaussian_mu = 0.5f; sensclosePrior_Gaussian_variance = 4.0f;
            sensfarPrior_Gaussian_mu   = 0.5f; sensfarPrior_Gaussian_variance   = 4.0f;


            max_posterior_variance = 10.0f;
        }


        unsigned u_TimeSinceLastRefresh;

        Real f_likelihood_variance;

        /* Sensor motor interactions. Motor reactions in the presence of sensor input from nbrs, and in the absence of sensor input from nbrs. Associated with the belief the robot is highly reactive to its neighbours. Or more precisely the fraction of times the robot reacts to its neighbours across its whole life-time - so say the robot is reacting to its neighbours about half its life-time. */
        unsigned sensmotPrior_Beta_a,   sensmotPrior_Beta_b;
        unsigned nosensmotPrior_Beta_a, nosensmotPrior_Beta_b;

        /* prior pertaining to actuation - i.e. distance travelled by robot in past 10s. Associated to the belief, is the robot moving large distances? */
        Real motPrior_Gaussian_mu, motPrior_Gaussian_variance;
        /* prior pertaining to motor reactions irrespective of sensor input. How does the robot react in a general sense */
        unsigned irrespsensmotPrior_Beta_a, irrespsensmotPrior_Beta_b;


        /* prior pertaining to sensing - i.e. number of neighbours in close and far proximity. Associated to the belief, is the robot part of a small aggregate?, and is the robot part of a large aggregate?  */
        Real sensclosePrior_Gaussian_mu, sensclosePrior_Gaussian_variance;
        Real sensfarPrior_Gaussian_mu, sensfarPrior_Gaussian_variance;


        Real max_posterior_variance;



        /*
         * ID of the observed Robot
         */
        unsigned  m_unRobotId;

        /*
         * Time the robot was first observed
         */
        Real      m_fTimeFirstObserved;


        /*
         * Value of feature vector of observed robot
         */
        unsigned  m_unValue;

        Real average_angularacceleration; // in rad / tick^2*/

        private:

            CBayesianInferenceFeatureVector& owner;

            Real*        m_pfFeatureValues;
            Real*        m_pfAllFeatureValues;


            /************************************************************************************/
            /* Keeping track of neighbours at different time scales*/
            Real m_fEstimated_Dist_ShortTimeWindow, m_fEstimated_Dist_MediumTimeWindow, m_fEstimated_Dist_LongTimeWindow;
            std::vector<RobotRelativePosData> vec_RobPos_ShortRangeTimeWindow, vec_RobPos_MediumRangeTimeWindow, vec_RobPos_LongRangeTimeWindow;
            /************************************************************************************/
    };


    CBayesianInferenceFeatureVector();
    virtual ~CBayesianInferenceFeatureVector();

    virtual unsigned SimulationStep();

    virtual unsigned GetIdFromRABPacket(CCI_RangeAndBearingSensor::TReadings &rab_packet, size_t rab_packet_index);
    virtual float    GetAnguAccelerFromRABPacket(CCI_RangeAndBearingSensor::TReadings &rab_packet, size_t rab_packet_index, unsigned observerd_robot_id);


    static RobotData m_sRobotData;
    SensoryData m_sSensoryData;

    typedef std::list <BayesInference_ObservedRobots_FeatureVector> t_ListObservedRobots;
    t_ListObservedRobots m_pcListObservedRobots;

    std::vector<unsigned> ObservedRobotIDs, ObservedRobotFVs;




protected:

    static unsigned int NUMBER_OF_FEATURES;
    static unsigned int MAX_NUMBER_OF_FEATURES;
    static double       FEATURE_RANGE;

    // keeping track of nbrs in time windows of different lengths
    static int        m_iShortTimeWindowLength, m_iMediumTimeWindowLength, m_iLongTimeWindowLength;
};

/******************************************************************************/
/******************************************************************************/


#endif
