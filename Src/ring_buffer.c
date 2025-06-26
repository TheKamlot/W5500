/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include "ring_buffer.h"
#include <string.h>




bool RingBuffer_Init(RingBuffer *ringBuffer, char *dataBuffer, size_t dataBufferSize) 
{
	assert(ringBuffer);
	assert(dataBuffer);
	assert(dataBufferSize > 0);
	
	if ((ringBuffer) && (dataBuffer) && (dataBufferSize > 0)) {
	  ringBuffer->dataBuffer = dataBuffer;
	  ringBuffer->dataBufferSize = dataBufferSize;
	  ringBuffer->head = 0;
	  ringBuffer->tail = 0;
	  ringBuffer->length = 0;
	  return true;
	}
	
	return false;
}

bool RingBuffer_Clear(RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
	    memset(ringBuffer->dataBuffer, 0, ringBuffer->dataBufferSize);
	    ringBuffer->head = 0;
	    ringBuffer->tail = 0;
	    ringBuffer->length = 0;
	    return true;
	}
	return false;
}

bool RingBuffer_IsEmpty(const RingBuffer *ringBuffer)
{
    assert(ringBuffer);	
    return (ringBuffer->length == 0);
}

size_t RingBuffer_GetLen(const RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		return ringBuffer->length;
	}
	return 0;
	
}

size_t RingBuffer_GetCapacity(const RingBuffer *ringBuffer)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		return (ringBuffer->dataBufferSize);
	}
	return 0;	
}


bool RingBuffer_PutChar(RingBuffer *ringBuffer, char c)
{
	assert(ringBuffer);
	
	if (ringBuffer) {
		if (ringBuffer->length >= ringBuffer->dataBufferSize){
		    return false;
		}
		
		ringBuffer->dataBuffer[ringBuffer->head] = c;
		
		if((ringBuffer->head + 1) >= ringBuffer->dataBufferSize){
		   ringBuffer->head = 0;
		}else{
		    (ringBuffer->head ++);
		}
		ringBuffer->length ++;
        return true;
	}
	return false;
}

bool RingBuffer_GetChar(RingBuffer *ringBuffer,char *c)
{
	assert(ringBuffer);
	assert(c);
	
    if ((ringBuffer) && (c)) {
		if(RingBuffer_IsEmpty(ringBuffer)){
		    return false;
		}
		*c = ringBuffer->dataBuffer[ringBuffer->tail];
		if((ringBuffer->tail + 1) >= ringBuffer->dataBufferSize){
		   (ringBuffer->tail = 0);
		}else{
		    (ringBuffer->tail ++);
		}
		ringBuffer->length --;
		return true;
	}
	return false;
}