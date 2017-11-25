#include "blackboard/blackboard.h"

#include <boost/any.hpp>
#include <utility>

namespace BT
{
	Blackboard::Blackboard(std::vector<std::pair<std::string, boost::any>> kv_pairs)
	{
		for (auto kv : kv_pairs)
			map_.insert(kv);
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
