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
#include "mic_mag_compensator/obeserver/mic_state_evaluator.h"

MIC_NAMESPACE_START

void MicStateEvaluator::update(mic_mag_compensator_t &comp)
{
    float64_t ts = comp.get_curr_time();
    mic_mag_t mag;
    ret_t find_gt = find_gt_state_by_ts(ts, mag, 0.01);
    if (find_gt == ret_t::MIC_RET_SUCCESSED)
    {
        // compare current state with gt state
        auto& data_storer = comp.get_data_storer();
        printf("compare\n!");
    }
    else
    {
        printf("no compare\n!");
        // output some log or just skip
    }
}

ret_t MicStateEvaluator::find_gt_state_by_ts(
    const float64_t ts,
    mic_mag_t &mag,
    const float64_t tolerant_time)
{
    auto precise_iter = _gts.find(ts);
    if (precise_iter != _gts.end())
    {
        mag = precise_iter->second;
        return ret_t::MIC_RET_SUCCESSED;
    }
    ret_t find_gt = ret_t::MIC_RET_FAILED;
    auto rough_iter_1 = _gts.lower_bound(ts);
    auto rough_iter_2 = rough_iter_1;
    if (rough_iter_2 != _gts.begin())
    {
        rough_iter_2--;
    }
    if (rough_iter_1 != _gts.end()
        && fabs(ts - rough_iter_1->first) < tolerant_time)
    {
        mag = rough_iter_1->second;
        find_gt = ret_t::MIC_RET_SUCCESSED;
    }
    else if (rough_iter_2 != _gts.end()
        && fabs(ts - rough_iter_2->first) < tolerant_time)
    {
        mag = rough_iter_2->second;
        find_gt = ret_t::MIC_RET_SUCCESSED;
    }
    return find_gt;
}

void MicStateEvaluator::add_ground_truth(const float64_t ts, const mic_mag_t &mag)
{
    if (ts > 0.)
    {
        _gts.emplace(ts, mag);
    }
}

MIC_NAMESPACE_END