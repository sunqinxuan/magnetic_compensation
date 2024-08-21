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
#ifndef MIC_TL_COMPONENT_MAG_COMPENSATOR
#define MIC_TL_COMPONENT_MAG_COMPENSATOR

#include "mic_mag_compensator/impl/mic_tl_mag_compensator.h"

MIC_NAMESPACE_START

class MicTLComponentMagCompensator;
using mic_tl_component_mag_compensator_t = MicTLComponentMagCompensator;

using mic_tolles_lawson_t = tl::TollesLawson;
using mic_tolles_lawson_shared_ptr = std::shared_ptr<tl::TollesLawson>;

class MicTLComponentMagCompensator : public MicTLMagCompensator
{
public:
    MicTLComponentMagCompensator(): MicTLMagCompensator(){}
    virtual ~MicTLComponentMagCompensator() = default;

protected:
    virtual ret_t do_calibrate() override;
    virtual ret_t do_compenste(const float64_t ts, mic_mag_t &out) override;

    // virtual ret_t serialize(json_t &node) override;
    // virtual ret_t deserialize(json_t &node) override;

    // mic_tolles_lawson_shared_ptr _tl_model;
    // vector_18f_t _tl_coeffs;
};

MIC_NAMESPACE_END

#endif