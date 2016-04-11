// Client object
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef CONFIGSERVER_PARAMETER_H
#define CONFIGSERVER_PARAMETER_H

#include <config_server/Subscribe.h>
#include <config_server/SetParameter.h>
#include <config_server/ParameterDescription.h>

#include <ros/service_server.h>
#include <ros/callback_queue.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

namespace config_server
{

/**
 * Base class for all parameters
 **/
class ParameterBase
{
public:
	ParameterBase() : m_haveDesc(false) {}
	virtual ~ParameterBase();

	void reinit();

	inline std::string name() const { return m_name; }

protected:
	void notifyServer();
	virtual void notifyClient();

	virtual std::string serialize() const = 0;
	virtual bool deserialize(const std::string& value) = 0;

	void init(const ParameterDescription& desc, ros::NodeHandle* nh = NULL, bool create = true);

	bool handleSet(const std::string& value);
private:
	friend class ParameterClient;

	std::string m_name;
	ParameterDescription m_desc;
	bool m_haveDesc;
};

template<class T>
class TypedParameter : public ParameterBase
{
public:
	void set(const T& value)
	{
		m_value = value;
		notifyServer();
		notifyClient();
	}

	inline T get() const { return m_value; }
	inline T operator()() const { return get(); }
	
	inline void setCallback(const boost::function<void (const T&)>& callback, bool callFunction = false)
	{
		m_callback = callback;
		if(callFunction && m_callback)
			m_callback(m_value);
	}
	inline void callCallback()
	{
		if(m_callback)
			m_callback(m_value);
	}

protected:
	T m_value;
	boost::function<void (const T&)> m_callback;

	virtual void notifyClient() { callCallback(); }
};

/**
 * User API for parameters on the config_server.
 *
 * Example usage:
 * @code
 *  // Create a parameter with range [0.0, 1.0], step size 0.01, default value 0.5
 *  Parameter<float> myParam("myParam", 0.0, 0.01, 1.0, 0.5);
 *
 *  // fetch its value
 *  float value = myParam();
 *
 *  // and set it
 *  myParam.set(0.8);
 *
 *  // set a callback (use boost::bind for more complex setups)
 *  void callback(float value) {}
 *  myParam.setCallback(&callback);
 * @endcode
 **/
template<class T>
class Parameter : TypedParameter<T>
{
};

// Following are specializations for different value types.

template<>
class Parameter<std::string> : public TypedParameter<std::string>
{
public:
	Parameter(const std::string& name, const std::string& value = "")
	{
		config_server::ParameterDescription desc;
		desc.name = name;
		desc.type = "string";
		desc.default_value = value;
		init(desc);
	}

	Parameter(const ParameterDescription& desc, ros::NodeHandle* nh = 0, bool create = true)
	{
		init(desc, nh, create);
	}

protected:
	virtual bool deserialize(const std::string& value)
	{
		m_value = value;
		return true;
	}

	virtual std::string serialize() const
	{
		return m_value;
	}
};

template<class T>
inline std::string toString(const T& value)
{
	std::stringstream ss;
	ss << value;
	return ss.str();
}

template<>
class Parameter<int> : public TypedParameter<int>
{
public:
	Parameter(const std::string& name, int min, int step, int max, int value)
	{
		ParameterDescription desc;
		desc.name = name;
		desc.type = "int";
		desc.default_value = toString(value);
		desc.step = step;
		desc.min = min;
		desc.max = max;
		init(desc);
	}

	Parameter(const ParameterDescription& desc, ros::NodeHandle* nh = 0, bool create = true)
	{
		init(desc, nh, create);
	}
protected:
	virtual bool deserialize(const std::string& value)
	{
		std::stringstream ss(value);

		int v;
		ss >> v;

		if(ss.fail() || ss.bad() || !ss.eof())
			return false;

		m_value = v;
		return true;
	}
	virtual std::string serialize() const
	{
		std::stringstream ss;
		ss << m_value;

		return ss.str();
	}
};

template<>
class Parameter<float> : public TypedParameter<float>
{
public:
	Parameter(const std::string& name, float min, float step, float max, float value)
	{
		ParameterDescription desc;
		desc.name = name;
		desc.type = "float";
		desc.default_value = toString(value);
		desc.step = step;
		desc.min = min;
		desc.max = max;
		init(desc);
	}

	Parameter(const ParameterDescription& desc, ros::NodeHandle* nh = 0, bool create = true)
	{
		init(desc, nh, create);
	}
protected:
	virtual bool deserialize(const std::string& value)
	{
		std::stringstream ss(value);

		float v;
		ss >> v;

		if(ss.fail() || ss.bad() || !ss.eof())
			return false;

		m_value = v;
		return true;
	}
	virtual std::string serialize() const
	{
		std::stringstream ss;
		ss << m_value;

		return ss.str();
	}
};

template<>
class Parameter<bool> : public TypedParameter<bool>
{
public:
	Parameter(const std::string& name, bool default_value)
	{
		ParameterDescription desc;
		desc.name = name;
		desc.type = "bool";
		desc.default_value = default_value ? "1" : "0";
		desc.step = 1;
		desc.min = 0;
		desc.max = 1;
		init(desc);
	}

	Parameter(const ParameterDescription& desc, ros::NodeHandle* nh = 0, bool create = true)
	{
		init(desc, nh, create);
	}
protected:
	virtual bool deserialize(const std::string& value)
	{
		if(value.size() == 0 || value == "0")
			m_value = false;
		else
			m_value = true;

		return true;
	}

	virtual std::string serialize() const
	{
		return m_value ? "1" : "0";
	}
};

}

#endif
