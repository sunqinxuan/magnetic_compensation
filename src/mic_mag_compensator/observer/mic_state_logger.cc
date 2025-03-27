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
#include "mic_mag_compensator/obeserver/mic_state_logger.h"

MIC_NAMESPACE_START

void MicStateLogger::update(mic_mag_compensator_t &comp)
{
    // MIC_LOG_DEBUG_INFO("");
    // MIC_LOG_BASIC_INFO("Compensator working state: %d", comp.get_working_state());
    auto comp_ellipsoid_nav = dynamic_cast<mic_ellipsoid_nav_mag_compensator_t *>(&comp);
    if (comp_ellipsoid_nav)
    {
        mic_state_t state = comp_ellipsoid_nav->get_working_state();
        if (state == mic_state_t::MIC_MAG_COMPENSATE_CALIBRATED)
        {
            matrix_3f_t coeff_D = comp_ellipsoid_nav->get_D_tilde_inv().inverse();
            vector_3f_t coeff_o = comp_ellipsoid_nav->get_o_hat();
            matrix_3f_t coeff_R = comp_ellipsoid_nav->get_R_opt();
            MIC_LOG_BASIC_INFO("");
            MIC_LOG_BASIC_INFO("Loaded model:");
            print_model_coeffs(coeff_D, coeff_o, coeff_R);
        }
    }
    else
    {
        auto comp_ellipsoid = dynamic_cast<mic_ellipsoid_mag_compensator_t *>(&comp);
        if (comp_ellipsoid)
        {
            mic_state_t state = comp_ellipsoid->get_working_state();
            if (state == mic_state_t::MIC_MAG_COMPENSATE_CALIBRATED)
            {
                MIC_LOG_DEBUG_INFO("");
                MIC_LOG_DEBUG_INFO("ceres report:\n%s", comp_ellipsoid->get_ceres_report().c_str());
                matrix_3f_t coeff_D = comp_ellipsoid->get_D_tilde_inv().inverse();
                vector_3f_t coeff_o = comp_ellipsoid->get_o_hat();
                matrix_3f_t coeff_R = comp_ellipsoid->get_R_opt();
                MIC_LOG_BASIC_INFO("");
                MIC_LOG_BASIC_INFO("Compensation model:");
                print_model_coeffs(coeff_D, coeff_o, coeff_R);
                // MIC_LOG_DEBUG_INFO("coeff_D: \n\t%.2f\t%.2f\t%.2f\n\t%.2f\t%.2f\t%.2f\n\t%.2f\t%.2f\t%.2f",
                //                    coeff_D(0, 0), coeff_D(0, 1), coeff_D(0, 2),
                //                    coeff_D(1, 0), coeff_D(1, 1), coeff_D(1, 2),
                //                    coeff_D(2, 0), coeff_D(2, 1), coeff_D(2, 2));
                // MIC_LOG_DEBUG_INFO("coeff_o: \n\t%.2f\t%.2f\t%.2f",
                //                    coeff_o(0), coeff_o(1), coeff_o(2));
                // MIC_LOG_DEBUG_INFO("coeff_R: \n\t%.2f\t%.2f\t%.2f\n\t%.2f\t%.2f\t%.2f\n\t%.2f\t%.2f\t%.2f",
                //                    coeff_R(0, 0), coeff_R(0, 1), coeff_R(0, 2),
                //                    coeff_R(1, 0), coeff_R(1, 1), coeff_R(1, 2),
                //                    coeff_R(2, 0), coeff_R(2, 1), coeff_R(2, 2));
            }
        }
    }

    // auto alarm=std::dynamic_cast<mic_ellipsoid_mag_compensator_t>(comp)
    // MIC_LOG_BASIC_INFO("Compensator current time: %f", comp.get_curr_time());

    // mic_mag_storer_t data_storer = comp.get_data_storer();
    // auto data_range = data_storer.get_data_range<mic_mag_flux_t>(0.0, ts);
    // auto it_start = data_range.first;
    // auto it_end = data_range.second;

    // for (auto it = it_start; it != it_end; ++it)
    // {
    //     float64_t time = it->first;
    //     mic_mag_flux_t mag_flux = it->second;
    //     // MIC_LOG_BASIC_INFO("%f\t%f\t%f\t%f", time, mag_flux.vector(0), mag_flux.vector(1), mag_flux.vector(2));
    // }
}

void MicStateLogger::print_model_coeffs(const matrix_3f_t &coeff_D,
                                        const vector_3f_t &coeff_o,
                                        const matrix_3f_t &coeff_R)
{
    MIC_LOG_BASIC_INFO("");
    MIC_LOG_BASIC_INFO("coeff_D: \n\t%.2f\t%.2f\t%.2f\n\t%.2f\t%.2f\t%.2f\n\t%.2f\t%.2f\t%.2f",
                       coeff_D(0, 0), coeff_D(0, 1), coeff_D(0, 2),
                       coeff_D(1, 0), coeff_D(1, 1), coeff_D(1, 2),
                       coeff_D(2, 0), coeff_D(2, 1), coeff_D(2, 2));
    MIC_LOG_BASIC_INFO("coeff_o: \n\t%.2f\t%.2f\t%.2f",
                       coeff_o(0), coeff_o(1), coeff_o(2));
    MIC_LOG_BASIC_INFO("coeff_R: \n\t%.2f\t%.2f\t%.2f\n\t%.2f\t%.2f\t%.2f\n\t%.2f\t%.2f\t%.2f",
                       coeff_R(0, 0), coeff_R(0, 1), coeff_R(0, 2),
                       coeff_R(1, 0), coeff_R(1, 1), coeff_R(1, 2),
                       coeff_R(2, 0), coeff_R(2, 1), coeff_R(2, 2));
    MIC_LOG_BASIC_INFO("");
}

MIC_NAMESPACE_END