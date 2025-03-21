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
#ifndef MIC_STATE_EVALUATOR
#define MIC_STATE_EVALUATOR

#include "common/mic_utils.h"

#include "mic_mag_compensator/mic_mag_compensator.h"

MIC_NAMESPACE_START

class MicStateEvaluator;
using mic_state_evaluator_t = class MicStateEvaluator;

class MicStateEvaluator : public MicObserver<MicMagCompensator>
{
public:
    MicStateEvaluator() = default;
    ~MicStateEvaluator() = default;

    virtual void update(mic_mag_compensator_t& comp) override;

    void add_ground_truth(const float64_t ts, const mic_mag_t& mag);

protected:

    ret_t find_gt_state_by_ts(
        const float64_t ts,
        mic_mag_t& mag,
        const float64_t tolerant_time
    );

    std::map<float64_t, mic_mag_t> _gts;

};

MIC_NAMESPACE_END

#endif