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
#ifndef MIC_UTILS
#define MIC_UTILS

#include "common/mic_prerequisite.h"
#include <mutex>
#include <vector>
#include <string>

MIC_NAMESPACE_START

template <typename T>
class MicObserver
{
public:
    MicObserver() {}
    virtual ~MicObserver() {}

    virtual void update(T &source) = 0;
};

template <typename T>
class MicObservable
{
public:
    MicObservable() {}
    virtual ~MicObservable() {}

    void notify(T &source)
    {
        std::unique_lock<std::mutex> lock{mtx};
        for (auto &&obs : observers)
        {
            if (obs != nullptr)
            {
                obs->update(source);
            }
        }
    }
    void subscrible(mic_observer_shared_ptr<T> obs)
    {
        std::unique_lock<std::mutex> lock{mtx};
        observers.emplace_back(obs);
    };
    void unsubscrible(mic_observer_shared_ptr<T> obs)
    {
        std::unique_lock<std::mutex> lock{mtx};
        observers.erase(
            std::remove(observers.begin(), observers.end(), obs),
            observers.end());
    }

protected:
    std::vector<mic_observer_shared_ptr<T>> observers;
    std::mutex mtx;
};

class MicUtils
{
public:
    /*
    euler2dcm(roll, pitch, yaw)

    Converts a (Euler) roll-pitch-yaw (`X`-`Y`-`Z`) right-handed body to navigation
    frame rotation (or the opposite rotation), to a DCM (direction cosine matrix).
    Yaw is synonymous with azimuth and heading here.

    the body frame is rotated in the standard
    -roll, -pitch, -yaw sequence to the navigation frame. For example, if v1 is
    a 3x1 vector in the body frame [nose, right wing, down], then that vector
    rotated into the navigation frame [north, east, down] would be v2 = dcm * v1.

    Reference: Titterton & Weston, Strapdown Inertial Navigation Technology, 2004,
    Section 3.6 (pg. 36-41 & 537).

    **Arguments:**
    - `roll`:  length roll  angle [rad], right-handed rotation about x-axis
    - `pitch`: length pitch angle [rad], right-handed rotation about y-axis
    - `yaw`:   length yaw   angle [rad], right-handed rotation about z-axis

    **Returns:**
    - `dcm`: `3` x `3` direction cosine matrix [-]
    */
    static matrix_3f_t euler2dcm(float32_t roll, float32_t pitch, float32_t yaw)
    {
        float32_t cr = cos(roll);
        float32_t sr = sin(roll);
        float32_t cp = cos(pitch);
        float32_t sp = sin(pitch);
        float32_t cy = cos(yaw);
        float32_t sy = sin(yaw);

        matrix_3f_t dcm = matrix_3f_t::Zero();
        dcm(0, 0) = cp * cy;
        dcm(0, 1) = -cr * sy + sr * sp * cy;
        dcm(0, 2) = sr * sy + cr * sp * cy;
        dcm(1, 0) = cp * sy;
        dcm(1, 1) = cr * cy + sr * sp * sy;
        dcm(1, 2) = -sr * cy + cr * sp * sy;
        dcm(2, 0) = -sp;
        dcm(2, 1) = sr * cp;
        dcm(2, 2) = cr * cp;

        return dcm;
    }
};

MIC_NAMESPACE_END

#endif