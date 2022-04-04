#pragma once

#include <stdint.h>

/*
 * This is a single-linked list/stack which items can be accessed via handle of
 * type H. The handle can be either an index (int8_t) to a pre-allocated list,
 * or a direct pointer to the item. The items should have a "next" member
 * variable of the intended handle type.
*/

template<class T, typename H, int8_t S> class IndexedStack;

/* The list can be used via the IndexedStackHandle element */

template<class T, typename H, int8_t S>
class IndexedStackHandle
{
public:
	IndexedStackHandle(class IndexedStack<T, H, S> &stack) : _stack(stack) {}

	void push(H handle) { _stack.push(handle);}
	H pop() { return _stack.pop(); }
	void push_free(H handle) { _stack.push_free(handle); }
	H pop_free() { return _stack.pop_free(); }
	bool rm(H handle) { return _stack.rm(handle); }
	H head() {return _stack._head;}
	H next(H handle) {return peek(handle)->next;}
	bool empty() {return !_stack.handle_valid(_stack.head());}
	T *peek(H handle) { return _stack.handle_valid(handle) ? _stack.peek(handle) : nullptr; }
	bool handle_valid(H handle) {return _stack.handle_valid(handle); }

private:

	class IndexedStack<T, H, S> &_stack;
};


/* The actual implementation of the list. This supports two types of handles;
 * void * for NuttX flat build / single process environment
 * int8_t for share memory / communication between processes
 */

template<class T, typename H, int8_t S>
class IndexedStack
{
public:
	friend class IndexedStackHandle<T, H, S>;

	IndexedStack()
	{
		clear_handle(_head);
		clear_handle(_free_head);
		init_freelist(_free_head);
	}

private:
	void push(H handle) { push(handle, _head);}
	H pop() { return pop(_head); }
	void push_free(H handle) { push(handle, _free_head); }
	H pop_free() { return pop(_free_head); }

	bool rm(H handle)
	{
		H p = _head;
		H r; clear_handle(r);

		if (!handle_valid(handle) ||
		    !handle_valid(p)) {
			return r;
		}

		if (p == handle) {
			// remove the first item
			T *item = peek(_head);
			_head = item->next;
			r  = p;

		} else {
			while (handle_valid((r = peek(p)->next))) {
				if (r == handle) {
					T *prev = peek(p);
					T *item = peek(r);
					// remove the item
					prev->next = item->next;
					break;
				}

				p = r;
			}
		}

		return handle_valid(r) ? true : false;
	}

	/* Helper functions for constructor; initialize the list of free items */
	void init_freelist(int8_t handle)
	{
		/* Add all items into free list */
		IndexedStackHandle<T, int8_t, S> self(*this);

		for (int8_t i = 0; i < S ; i++) {
			self.push_free(i);
		}

		_free_head = S - 1;
	}

	void init_freelist(void *handle) {}

	/* Push & pop internal implementations for actual and freelist */
	void push(H handle, H &head)
	{
		if (handle_valid(handle)) {
			T *item = peek(handle);
			item->next = head;
			head = handle;
		}
	}

	H pop(H &head)
	{
		H ret = head;

		if (handle_valid(ret)) {
			T *item = peek(head);
			head = item->next;
		}

		return ret;
	}

	T *peek(int8_t handle) { return &_item[handle]; }
	T *peek(void *handle) { return static_cast<T *>(handle); }
	static bool handle_valid(int8_t handle) { return handle >= 0; }
	static bool handle_valid(void *handle) { return handle != nullptr; }
	static void clear_handle(int8_t &x) { x = -1; };
	static void clear_handle(void *&x) { x = nullptr; };

	H _head;
	H _free_head;
	T _item[S];
};
