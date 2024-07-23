#ifndef MAG_COMPENSATOR
#define MAG_COMPENSATOR

#include "common/mag_prerequisite.h"
#include "common/mag_utils.h"
#include "data_storer/mag_data_storer.h"
#include "mag_ins/mag_ins.h"

MAG_NAMESPACE_START

using mag_measure_storer_t =
    mag_data_storer_t<mag_measure_vector_t, mag_measure_value_t>;

class MagCompensator;
using mag_compensator_t = MagCompensator;

enum class MagCompensatorState : uint8_t
{
    MAG_COMPENSATE_CALIBRATION = 0,
    MAG_COMPENSATE_NORMAL = 1,
    MAG_COMPENSATE_ABNORMAL = 2,
    MAG_COMPENSATE_ERROR = 3
};
using mag_compensator_state_t = MagCompensatorState;

class MagCompensator : public MagObservable<MagCompensator>
{
public:
    MagCompensator();
    ~MagCompensator();

    mag_measure_storer_t& get_data();
    mag_ins_t& get_ins();

    ret_t calibrate();
    ret_t compenste();

    ret_t serialize(json_t& node);
    ret_t deserialize(json_t& node);

protected:

    void init_ins();

    /* data processor*/
    mag_measure_storer_t _data_storer;
    /* state */
    mag_compensator_state_t _state;
    /* ins */
    mag_ins_unique_ptr _ins;
};

MAG_NAMESPACE_END

#endif