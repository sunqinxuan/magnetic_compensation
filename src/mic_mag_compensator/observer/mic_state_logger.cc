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

// Auxiliary function: Split string
std::vector<std::string> split(const std::string &s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

// Convert the date string to MATLAB datenum value
double dateStringToDatenum(const std::string &date_str)
{
    // supported fromat: "yyyy-mm-dd", "mm/dd/yyyy"
    std::vector<std::string> parts;

    if (date_str.find('-') != std::string::npos)
    {
        parts = split(date_str, '-');
    }
    else if (date_str.find('/') != std::string::npos)
    {
        parts = split(date_str, '/');
    }
    else
    {
        throw std::runtime_error("Unsupported date format");
    }

    if (parts.size() != 3)
    {
        throw std::runtime_error("Invalid date string format");
    }

    int year, month, day;

    // yyyy-mm-dd
    try
    {
        year = std::stoi(parts[0]);
        month = std::stoi(parts[1]);
        day = std::stoi(parts[2]);
    }
    catch (...)
    {
        // mm/dd/yyyy
        try
        {
            month = std::stoi(parts[0]);
            day = std::stoi(parts[1]);
            year = std::stoi(parts[2]);
        }
        catch (...)
        {
            throw std::runtime_error("Failed to parse date string");
        }
    }

    // Simple datenum calculation
    //(simplified version, without considering leap seconds or other complexities)
    // MATLAB datenum: 1 = January 1, 0000
    std::tm tm = {0};
    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;

    // convert to time_t
    std::time_t tt = mktime(&tm);
    if (tt == -1)
    {
        throw std::runtime_error("Failed to convert date to time_t");
    }

    // convert to datenum:
    // Conversion between MATLAB datenum and Unix timestamp
    const double unix_epoch_datenum = 719529.0; // datenum('1970-01-01')
    double datenum = unix_epoch_datenum + tt / (24.0 * 3600.0);

    return datenum;
}

// convert datenum to time_point
std::chrono::system_clock::time_point datenum_to_timepoint(double datenum)
{
    const double matlab_to_unix_epoch_days = 719529.0;
    double unix_time_seconds = (datenum - matlab_to_unix_epoch_days) * 86400.0;
    auto duration = std::chrono::duration<double>(unix_time_seconds);
    return std::chrono::system_clock::time_point(
        std::chrono::duration_cast<std::chrono::system_clock::duration>(duration));
}

std::string datetime2str(const std::string &date_str, double timestamp)
{
    // 1. Convert date string to datenum
    double date = dateStringToDatenum(date_str);

    // 2. Calculate full datenum (including timestamp)
    double dateNumWithTime = date + timestamp / (24.0 * 3600.0);

    // 3. Convert to time_point
    auto tp = datenum_to_timepoint(dateNumWithTime);

    // 4. Convert to time_t (second precision)
    auto tt = std::chrono::system_clock::to_time_t(tp);

    // 5. Get milliseconds component
    auto since_epoch = tp.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch - seconds).count();

    // 6. Format as string
    std::tm tm = *std::localtime(&tt);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << "." << std::setfill('0') << std::setw(3) << milliseconds;

    return oss.str();
}

void MicStateLogger::update(mic_mag_compensator_t &comp)
{
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
        if (state == mic_state_t::MIC_MAG_COMPENSATE_COMPENSATING)
        {
            auto &ds_comp = comp_ellipsoid_nav->get_data_storer_comp();
            float64_t ts = comp_ellipsoid_nav->get_curr_time();
            mic_mag_t mag_comp;
            if (ds_comp.get_data<mic_mag_t>(ts, mag_comp))
            {
                std::string date = "2024-09-09";
                std::string tt_str_m = datetime2str(date, ts);
                // MIC_LOG_BASIC_INFO("");
                // MIC_LOG_BASIC_INFO("");
                // MIC_LOG_BASIC_INFO("compensation output at time %.2f (total-field, x, y, z):\n\t%.2f nT, %.2f nT, %.2f nT, %.2f nT",
                //                    ts, mag_comp.value, mag_comp.vector(0), mag_comp.vector(1), mag_comp.vector(2));
                MIC_LOG_BASIC_INFO("");
                MIC_LOG_BASIC_INFO("**************************************");
                MIC_LOG_BASIC_INFO("current time: %s", tt_str_m.c_str());
                MIC_LOG_BASIC_INFO("");
                MIC_LOG_BASIC_INFO("compensation output (total-field, x, y, z):");
                MIC_LOG_BASIC_INFO("\t%.2f nT, %.2f nT, %.2f nT, %.2f nT",
                                   mag_comp.value, mag_comp.vector(0), mag_comp.vector(1), mag_comp.vector(2));
            }
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
            }
        }
    }
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