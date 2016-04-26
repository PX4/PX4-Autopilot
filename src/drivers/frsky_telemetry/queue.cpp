/* Code provided by Eric Suh
    |    ===========================================================    |
    |    This Queue Class has been implemented with templates and       |
    |    the size is determined dynamically at initialization.          |
    |                                                                   |
    |    The actual amount of space allocated for the Queue will be     |
    |    one more element space than the defined maximum size. This     |
    |    is useful for implementing the Queue in a circular method.     |
    |                                                                   |
    |    To understand the circular implementation, think of the        |
    |    array as a circle. When you reach the end of the array, you    |
    |    wrap around to the beginning of the array.                     |
    |                                                                   |
    |    So, when an element is dequeued, the Queue doesn't shift.      |
    |    Instead, you updated an indicator of the start of the queue.   |
    |                                                                   |
     -------------------------------------------------------------------
*/

#ifndef __QueueClassH__
#define __QueueClassH__

//-------------------------------------------------
// Main structure of Queue Class:
//-------------------------------------------------

template <class Elem>
class Queue
{
  public:
    Queue(int MaxSize=500);
    Queue(const Queue<Elem> &OtherQueue);
    ~Queue(void);

    void       Enqueue(const Elem &Item);    // Adds Item to Queue end
    Elem       Dequeue(void);                // Returns Item from Queue
    inline int ElemNum(void);                // Returns Number of Elements

  protected:
    Elem     *Data;      // The actual Data array
    const int MAX_NUM;   // The actual spaces will be one more than this
    int       Beginning, // Numbered location of the start and end
              End;

    // Instead of calculating the number of elements, using this variable
    // is much more convenient.
    int       ElemCount;
};

//-------------------------------------------------
// Implementation of Queue Class:
//-------------------------------------------------

// Queue Constructor function
template <class Elem>
Queue<Elem>::Queue(int MaxSize) :
    MAX_NUM( MaxSize )   // Initialize the constant
{
  // This extra space added will allow us to distinguish between
  // the Beginning and the End locations.
  Data      = new Elem[MAX_NUM + 1];
  Beginning = 0;
  End       = 0;
  ElemCount = 0;
}

// Queue Copy Constructor function
template <class Elem>
Queue<Elem>::Queue(const Queue &OtherQueue) :
                MAX_NUM( OtherQueue.MAX_NUM )  // Initialize the constant
{
  Beginning = OtherQueue.Beginning;
  End       = OtherQueue.End;
  ElemCount = OtherQueue.ElemCount;

  Data      = new Elem[MAX_NUM + 1];
  for (int i = 0; i < MAX_NUM; i++)
    Data[i] = OtherQueue.Data[i];
}

// Queue Destructor function
template <class Elem>
Queue<Elem>::~Queue(void)
{
  delete[] Data;
}

// Enqueue() function
template <class Elem>
void Queue<Elem>::Enqueue(const Elem &Item)
{
  // Error Check: Make sure we aren't exceeding our maximum storage space
  if(ElemCount < MAX_NUM) return;

  Data[ End++ ] = Item;
  ++ElemCount;

  // Check for wrap-around
  if (End > MAX_NUM)
    End -= (MAX_NUM + 1);
}

// Dequeue() function
template <class Elem>
Elem Queue<Elem>::Dequeue(void)
{
  // Error Check: Make sure we aren't dequeueing from an empty queue
  if(ElemCount == 0) return nullptr;

  Elem ReturnValue = Data[ Beginning++ ];
  --ElemCount;

  // Check for wrap-around
  if (Beginning > MAX_NUM)
    Beginning -= (MAX_NUM + 1);

  return ReturnValue;
}

// ElemNum() function
template <class Elem>
inline int Queue<Elem>::ElemNum(void)
{
  return ElemCount;
}

#endif /*__QueueClassH__*/
