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
    auto comp_ellipsoid = dynamic_cast<mic_ellipsoid_mag_compensator_t *>(&comp);
    if (comp_ellipsoid)
    {
        mic_state_t state = comp.get_working_state();
        if (state == mic_state_t::MIC_MAG_COMPENSATE_CALIBRATED)
        {
            auto &ds_measure = comp_ellipsoid->get_data_storer_measure();
            auto &ds_truth = comp_ellipsoid->get_data_storer_truth();
            if (ds_measure.get_data_size<mic_mag_t>() == 0 || ds_truth.get_data_size<mic_mag_t>() == 0)
            {
                return;
            }

            auto data_range = ds_measure.get_data_range<mic_mag_t>(0.0, comp_ellipsoid->get_curr_time() + 1);
            auto it_start = data_range.first;
            auto it_end = data_range.second;

            std::vector<float64_t> mag_t, mag_x, mag_y, mag_z;
            std::vector<float64_t> mag_b_t, mag_b_x, mag_b_y, mag_b_z;
            for (auto it = it_start; it != it_end; ++it)
            {
                float64_t ts = it->first;
                mic_mag_t mag_truth;
                mic_nav_state_t nav_state;
                if (ds_truth.get_data<mic_mag_t>(ts, mag_truth) &&
                    ds_measure.get_data<mic_nav_state_t>(ts, nav_state))
                {
                    vector_3f_t mag = it->second.vector;
                    matrix_3f_t D_tilde_inv = comp_ellipsoid->get_D_tilde_inv();
                    vector_3f_t o_hat = comp_ellipsoid->get_o_hat();
                    matrix_3f_t R_opt = comp_ellipsoid->get_R_opt();
                    matrix_3f_t R_nb = nav_state.attitude.matrix();

                    vector_3f_t mag_comp = R_opt.transpose() * D_tilde_inv * (mag - o_hat);
                    vector_3f_t mag_b = R_nb.transpose() * mag_truth.vector;

                    mag_t.push_back(mag_comp.norm());
                    mag_x.push_back(mag_comp(0));
                    mag_y.push_back(mag_comp(1));
                    mag_z.push_back(mag_comp(2));
                    mag_b_t.push_back(mag_truth.value);
                    mag_b_x.push_back(mag_b(0));
                    mag_b_y.push_back(mag_b(1));
                    mag_b_z.push_back(mag_b(2));
                }
            }
            float64_t rmse_t = MicUtils::rmse(mag_b_t, mag_t);
            float64_t rmse_x = MicUtils::rmse(mag_b_x, mag_x);
            float64_t rmse_y = MicUtils::rmse(mag_b_y, mag_y);
            float64_t rmse_z = MicUtils::rmse(mag_b_z, mag_z);

            MIC_LOG_BASIC_INFO("");
            MIC_LOG_BASIC_INFO("RMSE (intensity): \t\t%.2f nT", rmse_t);
            MIC_LOG_BASIC_INFO("RMSE (x_component): \t%.2f nT", rmse_x);
            MIC_LOG_BASIC_INFO("RMSE (y_component): \t%.2f nT", rmse_y);
            MIC_LOG_BASIC_INFO("RMSE (z_component): \t%.2f nT", rmse_z);
        }
        if (state == mic_state_t::MIC_MAG_COMPENSATE_COMPENSATING)
        {
            float64_t ts = comp_ellipsoid->get_curr_time();
            auto &ds_measure = comp_ellipsoid->get_data_storer_measure();
            auto &ds_truth = comp_ellipsoid->get_data_storer_truth();
            auto &ds_comp = comp_ellipsoid->get_data_storer_comp();
            mic_mag_t mag_truth, mag_comp;
            mic_nav_state_t nav_state;
            if (ds_measure.get_data<mic_nav_state_t>(ts, nav_state) &&
                ds_truth.get_data<mic_mag_t>(ts, mag_truth) &&
                ds_comp.get_data<mic_mag_t>(ts, mag_comp))
            {
                matrix_3f_t R_nb = nav_state.attitude.matrix();
            }

            vector_4f_t rmse_sq= comp_ellipsoid->get_rmse_sq();
            MIC_LOG_BASIC_INFO("");
            MIC_LOG_BASIC_INFO("RMSE (total-field, x, y, z): \t%.2f nT, %.2f nT, %.2f nT, %.2f nT", 
                sqrt(rmse_sq(0)),sqrt(rmse_sq(1)),sqrt(rmse_sq(2)),sqrt(rmse_sq(3)));
            // MIC_LOG_BASIC_INFO("RMSE (x_component): \t%.2f nT", sqrt(rmse_sq(1)));
            // MIC_LOG_BASIC_INFO("RMSE (y_component): \t%.2f nT", sqrt(rmse_sq(2)));
            // MIC_LOG_BASIC_INFO("RMSE (z_component): \t%.2f nT", sqrt(rmse_sq(3)));
        }
    }
    else
    {
        return;
    }
    // float64_t ts = comp.get_curr_time();
    // mic_mag_t mag;
    // ret_t find_gt = find_gt_state_by_ts(ts, mag, 0.01);
    // if (find_gt == ret_t::MIC_RET_SUCCESSED)
    // {
    //     // compare current state with gt state
    //     auto& data_storer = comp.get_data_storer_measure();
    //     printf("compare\n!");
    // }
    // else
    // {
    //     printf("no compare\n!");
    //     // output some log or just skip
    // }
}

// ret_t MicStateEvaluator::find_gt_state_by_ts(
//     const float64_t ts,
//     mic_mag_t &mag,
//     const float64_t tolerant_time)
// {
//     auto precise_iter = _gts.find(ts);
//     if (precise_iter != _gts.end())
//     {
//         mag = precise_iter->second;
//         return ret_t::MIC_RET_SUCCESSED;
//     }
//     ret_t find_gt = ret_t::MIC_RET_FAILED;
//     auto rough_iter_1 = _gts.lower_bound(ts);
//     auto rough_iter_2 = rough_iter_1;
//     if (rough_iter_2 != _gts.begin())
//     {
//         rough_iter_2--;
//     }
//     if (rough_iter_1 != _gts.end()
//         && fabs(ts - rough_iter_1->first) < tolerant_time)
//     {
//         mag = rough_iter_1->second;
//         find_gt = ret_t::MIC_RET_SUCCESSED;
//     }
//     else if (rough_iter_2 != _gts.end()
//         && fabs(ts - rough_iter_2->first) < tolerant_time)
//     {
//         mag = rough_iter_2->second;
//         find_gt = ret_t::MIC_RET_SUCCESSED;
//     }
//     return find_gt;
// }

// void MicStateEvaluator::add_ground_truth(const float64_t ts, const mic_mag_t &mag)
// {
//     if (ts > 0.)
//     {
//         _gts.emplace(ts, mag);
//     }
// }

MIC_NAMESPACE_END