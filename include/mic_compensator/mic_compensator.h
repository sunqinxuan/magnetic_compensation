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
#ifndef MIC_COMPENSATOR
#define MIC_COMPENSATOR

#include "common/mic_prerequisite.h"
#include "common/mic_utils.h"
#include "data_storer/mic_data_storer.h"
#include "mic_ins/mic_ins.h"

MIC_NAMESPACE_START

using mic_measure_storer_t =
    mic_data_storer_t<mic_measure_vector_t, mic_measure_value_t>;

class MicCompensator;
using mic_compensator_t = MicCompensator;

enum class MicCompensatorState : uint8_t
{
    MIC_COMPENSATE_CALIBRATION = 0,
    MIC_COMPENSATE_NORMAL = 1,
    MIC_COMPENSATE_ABNORMAL = 2,
    MIC_COMPENSATE_ERROR = 3
};
using mic_compensator_state_t = MicCompensatorState;

class MicCompensator : public MicObservable<MicCompensator>
{
public:
    MicCompensator();
    ~MicCompensator();

    mic_measure_storer_t& get_data();
    mic_ins_t& get_ins();

    ret_t calibrate();
    ret_t compenste();

    ret_t serialize(json_t& node);
    ret_t deserialize(json_t& node);

protected:

    void init_ins();

    /* data processor*/
    mic_measure_storer_t _data_storer;
    /* state */
    mic_compensator_state_t _state;
    /* ins */
    mic_ins_unique_ptr _ins;
};

MIC_NAMESPACE_END

#endif