#include <new>
#include "alloc_test_main.h"

struct mem_test_node {
	uint8_t data[1024 - sizeof(void*)];
	void *next;
};

void check_for_zero(uint8_t *buffer, int len) {
	for (int i = 0; i < len; i++) {
		if (buffer[i] != 0) {
			PX4_INFO("buffer not zero (%u) at offset %d", buffer[i], i);
		}
	}
}

// void* test_new_null_handling() {
// 	struct mem_test_node* test_new_null_ptr = new (std::nothrow) struct mem_test_node;
// 	if (test_new_null_ptr == nullptr) {
// 		return nullptr;
// 	} else {
// 		if (test_new_null_ptr->data[0] != 0) {
// 			PX4_INFO("First buffer spot not zeroed");
// 		}
// 	}
// 	return (void*) test_new_null_ptr;
// }

void new_test_we(void) {
	PX4_INFO("Starting new test with exceptions");

	uint32_t counter = 1;

	struct mem_test_node* new_test_head = (struct mem_test_node*) new struct mem_test_node;
	// struct mem_test_node* new_test_head = (struct mem_test_node*) test_new_null_handling();
	struct mem_test_node* new_test_current = new_test_head;
	if (new_test_head == NULL) {
		PX4_INFO("new test failed to allocate any memory");
		return;
	}
	struct mem_test_node* temp = (struct mem_test_node*) new struct mem_test_node;
	while (temp != NULL) {
		counter++;
		new_test_current->next = (void*) temp;
		new_test_current = temp;
		try {
			temp = (struct mem_test_node*) new struct mem_test_node;
		}
		// catch(std::bad_alloc) {
		catch(...) {
			PX4_INFO("Got new exception bad_alloc");
			break;
		}
	}
	PX4_INFO("new test allocated %u buffers", counter);
	new_test_current = new_test_head;
	while (true) {
		if (new_test_current->next != NULL) {
			new_test_current = (struct mem_test_node*) new_test_current->next;
			delete new_test_head;
			new_test_head = new_test_current;
		} else {
			delete new_test_current;
			break;
		}
	}
}

extern "C" {
	void *fc_heap_alloc(size_t size);
	void fc_heap_free(void* ptr);
}

void fc_heap_alloc_test(void) {
	PX4_INFO("Starting fc_heap_alloc test");

	uint32_t counter = 1;

	struct mem_test_node* fc_heap_alloc_test_head = (struct mem_test_node*) fc_heap_alloc(sizeof(struct mem_test_node));
	struct mem_test_node* fc_heap_alloc_test_current = fc_heap_alloc_test_head;
	if (fc_heap_alloc_test_head == NULL) {
		PX4_INFO("fc_heap_alloc test failed to allocate any memory");
		return;
	}
	check_for_zero((uint8_t*) fc_heap_alloc_test_head, sizeof(struct mem_test_node));
	struct mem_test_node* temp = (struct mem_test_node*) fc_heap_alloc(sizeof(struct mem_test_node));
	while (temp != NULL) {
		counter++;
		check_for_zero((uint8_t*) temp, sizeof(struct mem_test_node));
		fc_heap_alloc_test_current->next = (void*) temp;
		fc_heap_alloc_test_current = temp;
		temp = (struct mem_test_node*) fc_heap_alloc(sizeof(struct mem_test_node));
	}
	PX4_INFO("fc_heap_alloc test allocated %u buffers", counter);
	fc_heap_alloc_test_current = fc_heap_alloc_test_head;
	while (true) {
		if (fc_heap_alloc_test_current->next != NULL) {
			fc_heap_alloc_test_current = (struct mem_test_node*) fc_heap_alloc_test_current->next;
			fc_heap_free(fc_heap_alloc_test_head);
			fc_heap_alloc_test_head = fc_heap_alloc_test_current;
		} else {
			fc_heap_free(fc_heap_alloc_test_current);
			break;
		}
	}
}

int test_alloc() {
	PX4_INFO("Testing memory allocation from heap");

	// Obviously these tests are very dangerous. They use up all the heap, at least
	// for a small amount of time, which can (often does) cause the system to crash.
	// Only for debug use when a crash can be tolerated.
	fc_heap_alloc_test();
	// new_test_we();

	return 0;
}