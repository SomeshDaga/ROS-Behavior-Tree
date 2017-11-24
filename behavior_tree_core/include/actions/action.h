#ifndef ACTIONS_ACTION_H
#define ACTIONS_ACTION_H

#include <action_node.h>
#include "blackboard/blackboard.h"

#include <boost/shared_ptr.hpp>

#include <string>

namespace BT
{
class Action : public ActionNode
{
protected:
	// Blackboard pointer
	boost::shared_ptr<BT::Blackboard> blkbrd_ptr_;

	// Node Status
	BT::ReturnStatus status_;
public:
    // Constructor
    Action(std::string name, boost::shared_ptr<BT::Blackboard> blkbrd_ptr)
      :  ActionNode::ActionNode(name),
         blkbrd_ptr_(blkbrd_ptr)
    {
	    thread_ = std::thread(&Action::WaitForTick, this);
    }

    void WaitForTick()
    {
    	while (true)
    	{
    		tick_engine.Wait();

    		set_status(BT::RUNNING);
    		status_ = BT::RUNNING;

    		OnTick();
    	}
    }

	void Halt()
	{
	    set_status(BT::HALTED);
	}

	// Implement what happens on a tick in derived classes
	virtual void OnTick() = 0;

    ~Action()
    {}
};

}  // namespace BT
#endif  // ACTIONS_ACTION_H
