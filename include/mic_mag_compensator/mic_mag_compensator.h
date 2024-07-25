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
#ifndef MIC_MAG_COMPENSATOR
#define MIC_MAG_COMPENSATOR

#include "common/mic_prerequisite.h"
#include "common/mic_utils.h"
#include "data_storer/mic_data_storer.h"
#include "mic_nav_state_estimator/mic_nav_state_estimator.h"

MIC_NAMESPACE_START

using mic_mag_storer_t =
    mic_data_storer_t<mic_mag_flux_t, mic_mag_op_t>;

class MicMagCompensator;
using mic_mag_compensator_t = MicMagCompensator;

enum class MicMagCompensatorState : uint8_t
{
    MIC_MAG_COMPENSATE_CALIBRATION = 0,
    MIC_MAG_COMPENSATE_NORMAL = 1,
    MIC_MAG_COMPENSATE_ABNORMAL = 2,
    MIC_MAG_COMPENSATE_ERROR = 3
};
using mic_mag_compensator_state_t = MicMagCompensatorState;

class MicMagCompensator : public MicObservable<MicMagCompensator>
{
public:
    MicMagCompensator();
    ~MicMagCompensator();

    mic_mag_storer_t& get_data_storer();
    mic_nav_state_estimator_t &get_nav_state_estimator();
    float64_t get_curr_time() {return _curr_time_stamp;}

    ret_t add_mag_flux(
        const float64_t ts,
        const mic_mag_flux_t &mag_flux_data);

    ret_t add_mag_op(
        const float64_t ts,
        const mic_mag_op_t &mag_op_data);

    ret_t calibrate();
    // virtual ret_t calibrate() = 0;
    // virtual ret_t compenste() = 0;

    ret_t serialize(json_t &node);
    ret_t deserialize(json_t &node);

protected:
    void init_nav_state_estimator();

    float64_t _curr_time_stamp;

    /* data storer*/
    mic_mag_storer_t _data_storer;
    /* working state */
    mic_mag_compensator_state_t _state;
    /* navigation state estimator */
    mic_nav_state_estimator_unique_ptr _nav_state_estimator;
};

MIC_NAMESPACE_END

#endif