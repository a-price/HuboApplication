/**
 * \file RingBuffer.cpp
 * \brief Implements a simple ring buffer.
 * 
 * \author Andrew Price
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#define BUFFER_LOW_THRESHOLD 20

template <class templateClass>
class RingBuffer
{
public:
	BufferManager(void)
	{
		readIndex = 0;
		writeIndex = 1;
	}

	BufferManager(int size)
	{
		readIndex = 0;
		writeIndex = 1;
		bufferSize = size;
	}

	~BufferManager(void){}

	void Put(templateClass item)
	{
		if (!IsFull())
		{
			Buffer[writeIndex] = item;
			//Increment address and roll over if necessary
			writeIndex++;
			writeIndex %= BUFFER_SIZE;
		}
	}

	templateClass Get()
	{
		templateClass next = Buffer[readIndex];
		if (ItemsQueued() > 3)
		{
			//Increment address and roll over if necessary
			readIndex++;
			readIndex %= BUFFER_SIZE;
		}

		
		return next;
	}

	templateClass Peek()
	{
		return Buffer[readIndex];
	}

	templateClass PeekEnd()
	{
		return Buffer[writeIndex - 1];
	}

	bool IsFull()
	{
		return (((writeIndex + 1) % BUFFER_SIZE) == readIndex);
	}

	int ItemsQueued()
	{
		return ((writeIndex + BUFFER_SIZE) - readIndex) % BUFFER_SIZE;
	}

private:
	templateClass Buffer[BUFFER_SIZE];
	int readIndex;
	int writeIndex;
	int bufferSize;
};

#endif //RING_BUFFER_H