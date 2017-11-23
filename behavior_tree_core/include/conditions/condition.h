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

    ~Condition();

protected:
	boost::shared_ptr<BT::Blackboard> blkbrd_ptr_;
};

}  // namespace BT
#endif  // CONDITIONS_CONDITION_H
