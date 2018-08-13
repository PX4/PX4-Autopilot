/* A FIFO buffer for uint8 bytes implemented with a fixed size circular 
 * buffer.  A ByteQueue struct maintains the buffer data and state for one 
 * buffer instance, and many simultaneous instances are supported. An operation 
 * on one buffer must not be interrupted by another operation on that same 
 * buffer (possibly via an interrupt). This restriction is not enforced and no 
 * error code is generated.
 */

#ifndef BYTE_QUEUE_H
#define	BYTE_QUEUE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// ByteQueue instance state struct, voluntarily opaque.
struct ByteQueue {
	uint8_t* data;      // pointer to array allocated for data
	uint8_t *start;     // points to first data byte
	uint8_t *end;       // points to empty byte past last data byte
	uint16_t data_size; // size allocated for data
	uint16_t count;     // current contained element count
};

// initialize buffer as empty
// must provide pointer to allocated array for buffer to use
void InitBQ(struct ByteQueue *b, uint8_t* data, uint16_t data_size);

// return 1 if buffer full, 0 else
int8_t IsFullBQ(struct ByteQueue *b);

// return 1 if buffer empty, 0 else
int8_t IsEmptyBQ(struct ByteQueue *b);

// return number elements in buffer
uint16_t CountBQ(struct ByteQueue *p);

// read and remove next character from buffer
uint8_t GetByteBQ(struct ByteQueue *b);

// read but don't remove next character from buffer
uint8_t PeekByteBQ(struct ByteQueue *b);

// add one character to buffer
// return 1 for success, 0 for failure (buffer overflow)
int8_t PutByteBQ(struct ByteQueue *b, uint8_t item);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // BYTE_QUEUE_H
