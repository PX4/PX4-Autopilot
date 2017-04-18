/*
 * CRingBuffer.hpp
 *
 *  Created on: Aug 7, 2015
 *      Author: friedrich
 *
 *  a simple implementation of a ring buffer,
 *  where the oldest element becomes overwritten
 *  by the newest one. element [0] is always the
 *  oldest element in the buffer.
 */

#ifndef SRC_MODULES_PARACHUTE_CRINGBUFFER_HPP_
#define SRC_MODULES_PARACHUTE_CRINGBUFFER_HPP_

#define DEFAULT_SIZE 100

template<class T, unsigned int S = DEFAULT_SIZE>
class CRingBuffer
{
public:

	CRingBuffer()
		: m_uiIndex(0)
		, m_uiLength(0)
	{}
	~CRingBuffer(){}

	inline void add(T&);
	inline unsigned int length() const;

	const T& operator[](unsigned int const &) const;

private:

	unsigned int 	m_uiIndex;
	unsigned int 	m_uiLength;
	T	 			m_aItem[S];
};


/**
 * add()
 * adds and element to the ring buffer
 * where the oldest element gets overwritten
 *
 * @param		reference to item to insert
 */
template<class T, unsigned int S>
void
CRingBuffer<T, S>::add(T& item)
{
	if(m_uiLength < S)
	{
		m_uiLength++;
	}

	if(m_uiIndex < S)
	{
		m_uiIndex++;
	}
	else
	{
		m_uiIndex = 1;
	}

	m_aItem[m_uiIndex - 1] = item;
}


/**
 * operator[]
 * returns a reference to an item where
 * index = 0: oldest element
 * ...
 * index[length-1]: newest element
 *
 * @param 		index
 * @return 		reference to item
 */
template<class T, unsigned int S>
const T&
CRingBuffer<T, S>::operator [](unsigned int const & i) const
{
	return m_aItem[(S - m_uiLength + m_uiIndex + i) % S];
}


/**
 * length()
 * returns S when buffer is fully filled
 *
 * @return		length of buffer that is
 * 				always less or equal than S
 */

template<class T, unsigned int S>
unsigned int
CRingBuffer<T, S>::length() const
{
	return m_uiLength;
}


#endif /* SRC_MODULES_PARACHUTE_CRINGBUFFER_HPP_ */
