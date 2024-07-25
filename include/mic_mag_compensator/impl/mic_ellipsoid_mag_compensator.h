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
#ifndef MIC_ELLIPSOID_MAG_COMPENSATOR
#define MIC_ELLIPSOID_MAG_COMPENSATOR

#include "mic_mag_compensator/mic_mag_compensator.h"

MIC_NAMESPACE_START

class MicEllipsoidMagCompensator;
using mic_ellipsoid_mag_compensator_t = MicEllipsoidMagCompensator;

class MicEllipsoidMagCompensator : public MicMagCompensator
{
public:
    MicEllipsoidMagCompensator() : MicMagCompensator() {}
    virtual ~MicEllipsoidMagCompensator() = default;

    virtual ret_t calibrate() override;
    virtual ret_t compenste() override;

protected:
};

MIC_NAMESPACE_END

#endif