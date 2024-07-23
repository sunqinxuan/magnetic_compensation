#include "mag_compensator/mag_compensator.h"

#include "mag_ins/impl/mag_kf_gins.h"

MAG_NAMESPACE_START

MagCompensator::MagCompensator()
{
    init_ins();
}

MagCompensator::~MagCompensator()
{

}

void MagCompensator::init_ins()
{
    _ins = std::make_unique<mag_kf_gins_t>();
}

mag_measure_storer_t& MagCompensator::get_data()
{
    return _data_storer;
}

mag_ins_t& MagCompensator::get_ins()
{
    if (_ins == nullptr) init_ins();
    return *_ins;
}

ret_t MagCompensator::calibrate()
{
    // auto data_range = _data_storer.get_data_range()
    if (true) // judge data size
    {
        // calibration
    }
    return ret_t::MAG_RET_FAILED;
}

ret_t MagCompensator::compenste()
{
    // for observer updating
    notify(*this);
    return ret_t::MAG_RET_FAILED;
}

ret_t serialize(json_t& node)
{
    return ret_t::MAG_RET_FAILED;
}

ret_t deserialize(json_t& node)
{
    return ret_t::MAG_RET_FAILED;
}

MAG_NAMESPACE_END