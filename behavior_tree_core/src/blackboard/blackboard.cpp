#include "blackboard/blackboard.h"

#include <boost/any.hpp>
#include <utility>

namespace BT
{
	Blackboard::Blackboard(std::vector<std::pair<std::string, boost::any>> kv_pairs)
	{
		for (std::vector<std::pair<std::string, boost::any>>::iterator it = kv_pairs.begin(); it != kv_pairs.end(); it++)
		{
			map_.insert(*it);
		}
	}

	bool Blackboard::key_exists(std::string key) const
	{
		boost::mutex::scoped_lock lock(mutex_);
		if (map_.find(key) == map_.end())
			return false;
		else
			return true;
	}

}  // namespace BT
