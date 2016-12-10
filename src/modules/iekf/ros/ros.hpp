#include <string>

///
// ROS to uORB API translator
//

namespace ros
{

class Node
{
public:
	void spin();
};

void spin();

void init(int argc, char **argv, const std::string &node_name);

class Rate
{
public:
	Rate(float frequency);
	void sleep();
private:
	float _frequency;
	uint64_t _wake_timestamp;
};

class Subscriber
{
public:
	Subscriber(const std::string &topic, size_t queue_size);
	virtual ~Subscriber();
private:
	std::string _topic;
	size_t _queue_size;
};

class Publisher
{
public:
	Publisher(const std::string &name, size_t queue_size);
	virtual ~Publisher();

	template <class T>
	void publish(const T &msg)
	{
	}

private:
	std::string _name;
	size_t _queue_size;;
};

class NodeHandle
{
public:

	bool ok();

	void param(std::string name, float val, std::string topic);

	template <class M, class T>
	Subscriber subscribe(const std::string &topic, size_t queue_size,
			     void (T::*cb)(const M *msg), T *obj)
	{
		return Subscriber(topic, queue_size);
	}

	template <class T>
	Publisher advertise(const std::string &name, size_t queue_size)
	{
		return Publisher(name, queue_size);
	}

};

}
