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

    void notify(T& source)
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
    std::vector<mic_observer_shared_ptr<T> > observers;
    std::mutex mtx;
};

class MicUtils
{
public:
};

MIC_NAMESPACE_END

#endif