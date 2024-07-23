/*
 * Magnetic Interference Compensation
 *
 * Copyright (C) 2024 Qinxuan Sun. All rights reserved.
 *
 *     Author : Qinxuan Sun
 *    Contact : sunqinxuan@outlook.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "mic_compensator/mic_compensator.h"

#include "mic_ins/impl/mic_kf_gins.h"

MIC_NAMESPACE_START

MicCompensator::MicCompensator()
{
    init_ins();
}

MicCompensator::~MicCompensator()
{

}

void MicCompensator::init_ins()
{
    _ins = std::make_unique<mic_kf_gins_t>();
}

mic_measure_storer_t& MicCompensator::get_data()
{
    return _data_storer;
}

mic_ins_t& MicCompensator::get_ins()
{
    if (_ins == nullptr) init_ins();
    return *_ins;
}

ret_t MicCompensator::calibrate()
{
    // auto data_range = _data_storer.get_data_range()
    if (true) // judge data size
    {
        // calibration
    }
    return ret_t::MIC_RET_FAILED;
}

ret_t MicCompensator::compenste()
{
    // for observer updating
    notify(*this);
    return ret_t::MIC_RET_FAILED;
}

ret_t serialize(json_t& node)
{
    return ret_t::MIC_RET_FAILED;
}

ret_t deserialize(json_t& node)
{
    return ret_t::MIC_RET_FAILED;
}

MIC_NAMESPACE_END