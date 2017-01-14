#include <inttypes.h>
#include "HashMap/hashmap/HashMap.h"
#include "HashMap/hashmap/KeyHash.h"

class Param
{
public:
	enum Type {
		FLOAT = 0,
		INT32 = 1,
	};
	typedef HashMap<const char *, Param *, 8, KeyHash<const char *, 8> > DictType;
	union Value {
		static Value fromInt(int val) {
			Value v;
			v.i = val;
			return v;
		}
		static Value fromFloat(float val) {
			Value v;
			v.f = val;
			return v;
		}
		int i;
		float f;
	};
	template <class T>
	Param(const char *name, T defaultValue, Type type) :
		_type(type)
	{
		if (type == FLOAT) {
			_def = Value::fromFloat(float(defaultValue));

		} else if (type == INT32) {
			_def = Value::fromInt(int(defaultValue));
		}

		getDict().put(name, this);
	}
	Type getType()
	{
		return (Type)_type;
	}
	int getInt()
	{
		if (getType() == FLOAT) {
			return int(_def.f);

		} else if (getType() == INT32) {
			return _def.i;

		} else {
			return 0;
		}
	}
	float getFloat()
	{
		if (getType() == FLOAT) {
			return _def.f;

		} else if (getType() == INT32) {
			return float(_def.i);

		} else {
			return 0;
		}
	}
	static DictType &getDict()
	{
		return _dict;
	}
private:
	Value _def;
	Type _type;
	static DictType _dict;
	Param(const Param &p);
	Param &operator=(const Param &p);
};
Param::DictType Param::_dict = Param::DictType();
