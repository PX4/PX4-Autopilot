/**
 * @file SparseVector.hpp
 *
 * SparseVector class.
 *
 * @author Kamil Ritz <kritz@ethz.ch>
 * @author Julian Kent <julian@auterion.com>
 *
 */

#pragma once

#include "Vector.hpp"

namespace matrix
{
template<int N> struct force_constexpr_eval {
	static const int value = N;
};

// Vector that only store nonzero elements,
// which indices are specified as parameter pack
template<typename Type, size_t M, size_t... Idxs>
class SparseVector
{
private:
	static constexpr size_t N = sizeof...(Idxs);
	static constexpr size_t _indices[N] {Idxs...};

	static constexpr bool duplicateIndices()
	{
		for (size_t i = 0; i < N; i++) {
			for (size_t j = 0; j < i; j++) {
				if (_indices[i] == _indices[j]) {
					return true;
				}
			}
		}

		return false;
	}
	static constexpr size_t findMaxIndex()
	{
		size_t maxIndex = 0;

		for (size_t i = 0; i < N; i++) {
			if (maxIndex < _indices[i]) {
				maxIndex = _indices[i];
			}
		}

		return maxIndex;
	}

	static_assert(!duplicateIndices(), "Duplicate indices");
	static_assert(N < M, "More entries than elements, use a dense vector");
	static_assert(N > 0, "A sparse vector needs at least one element");
	static_assert(findMaxIndex() < M, "Largest entry doesn't fit in sparse vector");

	Type _data[N] {};

	static constexpr int findCompressedIndex(size_t index)
	{
		int compressedIndex = -1;

		for (size_t i = 0; i < N; i++) {
			if (index == _indices[i]) {
				compressedIndex = static_cast<int>(i);
			}
		}

		return compressedIndex;
	}

public:
	constexpr size_t non_zeros() const
	{
		return N;
	}

	constexpr size_t index(size_t i) const
	{
		return SparseVector::_indices[i];
	}

	SparseVector() = default;

	SparseVector(const matrix::Vector<Type, M> &data)
	{
		for (size_t i = 0; i < N; i++) {
			_data[i] = data(_indices[i]);
		}
	}

	explicit SparseVector(const Type data[N])
	{
		memcpy(_data, data, sizeof(_data));
	}

	template <size_t i>
	inline Type at() const
	{
		static constexpr int compressed_index = force_constexpr_eval<findCompressedIndex(i)>::value;
		static_assert(compressed_index >= 0, "cannot access unpopulated indices");
		return _data[compressed_index];
	}

	template <size_t i>
	inline Type &at()
	{
		static constexpr int compressed_index = force_constexpr_eval<findCompressedIndex(i)>::value;
		static_assert(compressed_index >= 0, "cannot access unpopulated indices");
		return _data[compressed_index];
	}

	inline Type atCompressedIndex(size_t i) const
	{
		assert(i < N);
		return _data[i];
	}

	inline Type &atCompressedIndex(size_t i)
	{
		assert(i < N);
		return _data[i];
	}

	void setZero()
	{
		for (size_t i = 0; i < N; i++) {
			_data[i] = Type(0);
		}
	}

	Type dot(const matrix::Vector<Type, M> &other) const
	{
		Type accum(0);

		for (size_t i = 0; i < N; i++) {
			accum += _data[i] * other(_indices[i]);
		}

		return accum;
	}

	matrix::Vector<Type, M> operator+(const matrix::Vector<Type, M> &other) const
	{
		matrix::Vector<Type, M> vec = other;

		for (size_t i = 0; i < N; i++) {
			vec(_indices[i]) +=  _data[i];
		}

		return vec;
	}

	SparseVector &operator+=(Type t)
	{
		for (size_t i = 0; i < N; i++) {
			_data[i] += t;
		}

		return *this;
	}

	Type norm_squared() const
	{
		Type accum(0);

		for (size_t i = 0; i < N; i++) {
			accum += _data[i] * _data[i];
		}

		return accum;
	}

	Type norm() const
	{
		return std::sqrt(norm_squared());
	}

	bool longerThan(Type testVal) const
	{
		return norm_squared() > testVal * testVal;
	}
};

template<typename Type, size_t Q, size_t M, size_t ... Idxs>
matrix::Vector<Type, Q> operator*(const matrix::Matrix<Type, Q, M> &mat,
				  const matrix::SparseVector<Type, M, Idxs...> &vec)
{
	matrix::Vector<Type, Q> res;

	for (size_t i = 0; i < Q; i++) {
		const Vector<Type, M> row = mat.row(i);
		res(i) = vec.dot(row);
	}

	return res;
}

// returns x.T * A * x
template<typename Type, size_t M, size_t ... Idxs>
Type quadraticForm(const matrix::SquareMatrix<Type, M> &A, const matrix::SparseVector<Type, M, Idxs...> &x)
{
	Type res = Type(0);

	for (size_t i = 0; i < x.non_zeros(); i++) {
		Type tmp = Type(0);

		for (size_t j = 0; j < x.non_zeros(); j++) {
			tmp += A(x.index(i), x.index(j)) * x.atCompressedIndex(j);
		}

		res += x.atCompressedIndex(i) * tmp;
	}

	return res;
}

template<typename Type, size_t M, size_t... Idxs>
constexpr size_t SparseVector<Type, M, Idxs...>::_indices[SparseVector<Type, M, Idxs...>::N];

template<size_t M, size_t ... Idxs>
using SparseVectorf = SparseVector<float, M, Idxs...>;

}
