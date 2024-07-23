#ifndef MAG_UTILS
#define MAG_UTILS

#include "common/mag_prerequisite.h"
#include <mutex>
#include <vector>

MAG_NAMESPACE_START

template <typename T>
class MagObserver
{
public:
    MagObserver() {}
    virtual ~MagObserver() {}

    virtual void update(T &source) = 0;
};

template <typename T>
class MagObservable
{
public:
    MagObservable() {}
    virtual ~MagObservable() {}

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
    void subscrible(mag_observer_shared_ptr<T> obs)
    {
        std::unique_lock<std::mutex> lock{mtx};
        observers.emplace_back(obs);
    };
    void unsubscrible(mag_observer_shared_ptr<T> obs)
    {
        std::unique_lock<std::mutex> lock{mtx};
        observers.erase(
            std::remove(observers.begin(), observers.end(), obs),
                  observers.end());
    }

protected:
    std::vector<mag_observer_shared_ptr<T> > observers;
    std::mutex mtx;
};

class MagUtils
{
public:
};

MAG_NAMESPACE_END

#endif