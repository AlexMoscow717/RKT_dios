#pragma once

#include <ArduRTOSHAL_AL.h>



typedef void (*TPTR)(void);
typedef TPTR Data;
void InitRTOS(void);
void Idle(void);
void SetTask(TPTR TS);
void SetTimerTask(TPTR TS, uint16_t NewTime);
void TimerService(void);

typedef struct TimerSlot
{
	TPTR GoToTask; 						// Указатель перехода
	uint16_t Time;						// Выдержка в мс
};

class Queue
{
	public:
	Queue();
	~Queue();
	void push_back(Data d);
	//int GetSize() { return size; }
	//T& operator[](const int index);
	void pop_front();

	private:

	class Node
	{
		public:

		Node* pNext;
		Data data;
		//Node(Data data = Idle, Node* pNext = nullptr)
		Node(Data data = nullptr, Node* pNext = nullptr)
		{
			this->data = data;
			this->pNext = pNext;	
		}
		
	};
	
	Node* head;
	Node* tail;
	//byte size;
};








