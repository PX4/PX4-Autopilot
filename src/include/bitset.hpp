
#pragma once

template <const size_t N>
class bitset
{

private:

	static constexpr uint8_t BITS_PER_ELEMENT = 8;
	static constexpr size_t ARRAY_SIZE = (N % BITS_PER_ELEMENT == 0) ? N / BITS_PER_ELEMENT : N / BITS_PER_ELEMENT + 1;
	static constexpr size_t ALLOCATED_BITS = ARRAY_SIZE * BITS_PER_ELEMENT;

	uint8_t _data[ARRAY_SIZE] {};

	size_t array_index(size_t position) const
	{
		return position / BITS_PER_ELEMENT;
	}

	uint8_t element_index(size_t position) const
	{
		return (1 << position % BITS_PER_ELEMENT);
	}

public:

	size_t count() const
	{
		size_t total = 0;

		for (auto &x : _data) {
			for (uint8_t i = 0; i < BITS_PER_ELEMENT; i++) {
				const uint8_t mask = 1 << i;

				if (x & mask) {
					total++;
				}
			}
		}

		return total;
	}

	size_t size() const
	{
		return N;
	}

	bool test(size_t position) const
	{
		return _data[array_index(position)] & element_index(position);
	}

	bool operator[](size_t position) const
	{
		return test(position);
	}

	void set(size_t pos, bool val = true)
	{
		const uint8_t bitmask = (1 << pos % BITS_PER_ELEMENT);

		if (val) {
			_data[array_index(pos)] |= bitmask;

		} else {
			_data[array_index(pos)] &= ~bitmask;
		}
	}

};
