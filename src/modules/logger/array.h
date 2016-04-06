#pragma once

#include <stdlib.h>

namespace px4
{

template <typename TYPE, size_t N>
class Array
{
	typedef TYPE &reference;
	typedef const TYPE &const_reference;
	typedef TYPE *iterator;
	typedef const TYPE *const_iterator;

public:
	Array()
		: _size(0), _overflow(false)
	{}

	bool push_back(const TYPE &x)
	{
		if (_size == N) {
			_overflow = true;
			return false;

		} else {
			_items[_size] = x;
			++_size;
			return true;
		}
	}

	void remove(unsigned idx)
	{
		if (idx < _size) {
			--_size;

			for (unsigned i = idx; i < _size; ++i) {
				_items[i] = _items[i + 1];
			}
		}
	}

	reference operator[](size_t n)
	{
		return _items[n];
	}

	const_reference operator[](size_t n) const
	{
		return _items[n];
	}

	reference at(size_t n)
	{
		return _items[n];
	}

	const_reference at(size_t n) const
	{
		return _items[n];
	}

	size_t size() const
	{
		return _size;
	}

	size_t max_size() const
	{
		return N;
	}

	size_t capacity() const
	{
		return N;
	}

	bool empty() const
	{
		return _size == 0;
	}

	bool is_overflowed()
	{
		return _overflow;
	}

	iterator begin()
	{
		return &_items[0];
	}

	iterator end()
	{
		return &_items[_size];
	}

	const_iterator begin() const
	{
		return &_items[0];
	}

	const_iterator end() const
	{
		return &_items[_size];
	}

	void erase(iterator item)
	{
		if (item - _items < static_cast<int>(_size)) {
			--_size;

			for (iterator it = item; it != &_items[_size]; ++it) {
				*it = *(it + 1);
			}
		}
	}

private:
	TYPE        _items[N];
	size_t      _size;
	bool        _overflow;
};

}
