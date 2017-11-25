#include <boost/any.hpp>
#include <boost/thread/mutex.hpp>

#include <map>
#include <utility>
#include <vector>

#ifndef BLACKBOARD_BLACKBOARD_H
#define BLACKBOARD_BLACKBOARD_H

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
	bool add_kv(std::string key, T value)
	{
		boost::mutex::scoped_lock lock(mutex_);
		std::pair<std::map<std::string, boost::any>::iterator, bool> result =
			map_.insert(std::pair<std::string, boost::any>(key, value));

		return result.second;
	}

	template <typename T>
	inline T get_value(std::string key) const
	{
		boost::mutex::scoped_lock lock(mutex_);
		return boost::any_cast<T>(map_.find(key)->second);
	}

	template<typename T>
	inline void update_kv(std::string key, T value)
	{
		if (key_exists(key))
		{
			boost::mutex::scoped_lock lock(mutex_);
			map_.find(key)->second = value;
		}
		else
			add_kv<T>(key, value);
	}

	bool key_exists(std::string key) const;

	~Blackboard()
	{}

};
}  //namespace BT
#endif  // BLACKBOARD_BLACKBOARD_H