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

#ifndef CONDITIONS_CONDITION_H
#define CONDITIONS_CONDITION_H

#include <condition_node.h>
#include "blackboard/blackboard.h"

#include <boost/shared_ptr.hpp>

#include <string>

namespace BT
{
class Condition : public ConditionNode
{
public:
    // Constructor
    Condition(std::string name, boost::shared_ptr<BT::Blackboard> blkbrd_ptr)
      :  ConditionNode::ConditionNode(name),
         blkbrd_ptr_(blkbrd_ptr)
    {}

    ~Condition() {}

protected:
    boost::shared_ptr<BT::Blackboard> blkbrd_ptr_;
};

}  // namespace BT
#endif  // CONDITIONS_CONDITION_H
