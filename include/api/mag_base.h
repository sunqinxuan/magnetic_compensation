#ifndef MAG_BASE
#define MAG_BASE

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mag
{
    using float64_t = double;
    using float32_t = float;
    using char_t = char;
    using vector_3f_t = Eigen::Vector3f;
    using quaternionf_t = Eigen::Quaternionf;

    enum MagBool
    {
        MAG_FALSE               = 0,
        MAG_TRUE                = 1
    };
    using bool_t = MagBool;
    #define true (mag::MAG_TRUE)
    #define false (mag::MAG_FALSE)

    enum class MagRet : uint8_t
    {
        MAG_RET_FAILED                = 0,
        MAG_RET_SUCCESSED             = 1
    };
    using ret_t = MagRet;

    struct MagMeasureVector
    {
        float64_t time_stamp; // ?
        vector_3f_t flux;
        float32_t confidence;
    };
    using mag_measure_vector_t = MagMeasureVector;

    struct MagMeasureValue
    {
        float64_t time_stamp; // ?
        float32_t mag_value;
        float32_t confidence;
    };
    using mag_measure_value_t = MagMeasureValue;

    struct MagInsData
    {
        float64_t time_stamp; // ?
        vector_3f_t gyr;
        vector_3f_t acc;
        vector_3f_t gyr_bias;
        vector_3f_t acc_bias;
    };
    using mag_ins_data_t = MagInsData;

    struct MagPose
    {
        vector_3f_t position;
        quaternionf_t attitude;
        vector_3f_t velocity;
    };
    using mag_pose_t = MagPose;
}
#endif