/****************************************************************************
 *
 *   Copyright (C) 2012-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file List.hpp
 *
 * An intrusive linked list.
 */

#pragma once

#include <stdlib.h>

template<class T>
class ListNode
{
public:

	void setSibling(T sibling) { _sibling = sibling; }
	const T getSibling() const { return _sibling; }

protected:

	T _sibling{nullptr};

};

template<class T>
class List
{
public:

	void add(T newNode)
	{
		newNode->setSibling(getHead());
		_head = newNode;
	}

	bool remove(T removeNode)
	{
		// base case
		if (removeNode == _head) {
			_head = nullptr;
			return true;
		}

		for (T node = getHead(); node != nullptr; node = node->getSibling()) {
			// is sibling the node to remove?
			if (node->getSibling() == removeNode) {
				// replace sibling
				if (node->getSibling() != nullptr) {
					node->setSibling(node->getSibling()->getSibling());

				} else {
					node->setSibling(nullptr);
				}

				return true;
			}
		}

		return false;
	}

	struct Iterator {
		T node;
		Iterator(T v) : node(v) {}

		operator T() const { return node; }
		operator T &() { return node; }
		T operator* () const { return node; }
		Iterator &operator++ ()
		{
			if (node) {
				node = node->getSibling();
			};

			return *this;
		}
	};

	Iterator begin() { return Iterator(getHead()); }
	Iterator end() { return Iterator(nullptr); }

	const T getHead() const { return _head; }

	bool empty() const { return getHead() == nullptr; }

	size_t size() const
	{
		size_t sz = 0;

		for (auto node = getHead(); node != nullptr; node = node->getSibling()) {
			sz++;
		}

		return sz;
	}

	void deleteNode(T node)
	{
		if (remove(node)) {
			// only delete if node was successfully removed
			delete node;
		}
	}

	void clear()
	{
		auto node = getHead();

		while (node != nullptr) {
			auto next = node->getSibling();
			delete node;
			node = next;
		}

		_head = nullptr;
	}

protected:

	T _head{nullptr};
};
