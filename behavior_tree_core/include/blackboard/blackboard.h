/* Copyright (C) 2015-2017 Michele Colledanchise - All Rights Reserved
*
*   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
*   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
*   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
*   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*
*   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef BLACKBOARD_BLACKBOARD_H
#define BLACKBOARD_BLACKBOARD_H

#include <boost/any.hpp>
#include <boost/thread/mutex.hpp>

#include <stdexcept>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace BT
{
class Blackboard
{
protected:
    std::map<std::string, boost::any> map_;

    mutable boost::mutex mutex_;

public:
    explicit Blackboard(std::vector< std::pair<std::string, boost::any> > kv_pairs);

    template <typename T>
    inline T GetValue(std::string key) const
    {
        if (!KeyExists(key))
            throw std::invalid_argument("Key " + key + " does not exist");

        boost::mutex::scoped_lock lock(mutex_);
        return boost::any_cast<T>(map_.find(key)->second);
    }

    template<typename T>
    inline void UpdateKv(std::string key, T value)
    {
        // Updates key-value pair if exists, else adds it to map_
        boost::mutex::scoped_lock lock(mutex_);
        map_[key] = value;
    }

    bool KeyExists(std::string key) const;

    ~Blackboard()
    {}
};
}  // namespace BT
#endif  // BLACKBOARD_BLACKBOARD_H
