/*
 * RingBuffer.h
 *
 *  Created on: Jul 4, 2019
 *      Author: levin
 */

#ifndef SRC_RINGBUFFER_H_
#define SRC_RINGBUFFER_H_
#include <vector>

//This is a simplified version of ring bufffer, which can only push data into the buffer, but can't remove data.
template<typename T>
class RingBuffer {
public:
	std::vector<T> m_vec;
	int m_head;
	int m_tail;
	int m_capacity;



	RingBuffer(int capacity)
	{
		m_vec.resize(capacity);
		m_data_size = 0;
		m_head = -1;
		m_tail = -1;
		m_capacity = capacity;
	}
	void push_back(T & item){
		m_data_size ++;
		if(m_data_size > m_capacity){
			//the handling here is easy, as we only add data into the ring buffer
			m_data_size = m_capacity;
		}
		if (m_tail == -1){
			//first time
			m_tail++;
			m_head++;
			m_vec[m_tail] = item ;
			return;
		}
		m_tail = (m_tail + 1) % m_capacity;
		m_vec[m_tail] = item;
		if(m_tail == m_head){
			m_head = (m_tail + 1) % m_capacity;
		}
		return;
	}
	T *tail_prev(){
		//return the item right before the tail node
		int idx = (m_tail + m_capacity - 1) % m_capacity;
		return &(m_vec[idx]);
	}
	T* end(){
		//end here refers to the addreas past the tail node
		if (m_tail == -1) return NULL;
		return &(m_vec[m_tail]) + 1;
	}
	int size(){
		return m_data_size;
	}

	virtual ~RingBuffer(){

	}
private:
	int m_data_size;
};

#endif /* SRC_RINGBUFFER_H_ */
