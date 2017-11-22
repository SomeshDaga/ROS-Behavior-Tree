#ifndef CONDITIONS_CONDITION_H
#define CONDITIONS_CONDITION_H

#include <condition_node.h>
#include <string>

namespace BT
{
class Condition : public ConditionNode
{

public:
    // Constructor
    Condition(std::string name)
      :  ConditionNode::ConditionNode(name)
    {}

    ~Condition();
};

}  // namespace BT
#endif  // CONDITIONS_CONDITION_H
