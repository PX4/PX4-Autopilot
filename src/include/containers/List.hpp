/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * A linked list.
 */

#pragma once

template<class T>
class __EXPORT ListNode
{
public:
	ListNode() : _sibling(nullptr)
	{
	}
	virtual ~ListNode() {};
	void setSibling(T sibling) { _sibling = sibling; }
	T getSibling() { return _sibling; }
	T get()
	{
		return _sibling;
	}
protected:
	T _sibling;
private:
	// forbid copy
	ListNode(const ListNode &other);
	// forbid assignment
	ListNode &operator = (const ListNode &);
};

template<class T>
class __EXPORT List
{
public:
	List() : _head()
	{
	}
	virtual ~List() {};
	void add(T newNode)
	{
		newNode->setSibling(getHead());
		setHead(newNode);
	}
	T getHead() { return _head; }
protected:
	void setHead(T &head) { _head = head; }
	T _head;
private:
	// forbid copy
	List(const List &other);
	// forbid assignment
	List &operator = (const List &);
};
