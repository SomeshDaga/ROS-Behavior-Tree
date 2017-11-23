#include <boost/any.hpp>
#include <map>
#include <utility>
#include <vector>

#ifndef BLACKBOARD_BLACKBOARD_H
#define BLACKBOARD_BLACKBOARD_H

namespace BT
{
class Blackboard
{
private:
	std::map<std::string, boost::any> map_;

public:
	explicit Blackboard(std::vector< std::pair<std::string, boost::any> > kv_pairs);

	template <typename T>
	void add_kv(std::string key, T value)
	{
		if (!key_exists(key))
			map_.insert(std::pair<std::string, boost::any>(key, value));
	}

	template <typename T>
	inline T get_value(std::string key) const
	{
		return boost::any_cast<T>(map_.find(key)->second);
	}

	template<typename T>
	inline void update_kv(std::string key, T value)
	{
		if (key_exists(key))
			map_.find(key)->second = value;
	}

	bool key_exists(std::string key) const;

	~Blackboard()
	{}

};
}  //namespace BT
#endif  // BLACKBOARD_BLACKBOARD_H