#include "ArduRTOS_AL.h"

//template<typename T> 



volatile static TimerSlot MainTimer[MainTimerQueueSize + 1];		// Очередь таймеров
//Queue QueueMain;
extern Queue QueueMain;
void Idle(void)
{

}

Queue::Queue()
{
	//size = 0;
	this->head = nullptr;
	this->tail = nullptr;

}

Queue::~Queue()
{
}

void Queue::push_back(Data d)
{
	uint8_t		nointerrupted = 0;
	
	if (STATUS_REG & (1<<Interrupt_Flag))  // Если прерывания разрешены, то запрещаем их.
	{
		Disable_Interrupt
		nointerrupted = 1;					// И ставим флаг, что мы не в прерывании.
	}

	Node* NewNode = new Node(d);

	if (tail != nullptr)
	{
		this->tail->pNext = NewNode;
		
	}
	
	this->tail = NewNode;

	if (head == nullptr)
	{
		this->head = NewNode;
	}

	//size++;
	if (nointerrupted)	Enable_Interrupt
}

// template<typename T>
// T& Queue<T>::operator[](const int index)
// {
// 	int counter = 0;
// 	Node<T>* current = this->head;
//
// 	while (current != nullptr)
// 	{
// 		if (counter == index)
// 		{
// 			return current->data;
// 		}
// 		current = current->pNext;
// 		counter++;
// 	}
// }


void Queue::pop_front()
{
	Disable_Interrupt
	
	if (this->head == nullptr)
	{
		Enable_Interrupt
		(Idle)();
		return;
	}
	Node* temp = this->head;
	Data data_temp = this->head->data;
	this->head = this->head->pNext;
	if (this->head == nullptr)
	{
		this->tail = nullptr;
	}
	delete temp;
	temp = nullptr;
	//size--;
	Enable_Interrupt;
	(data_temp)();
}


void InitRTOS(void)
{
	uint8_t	index;
	
	TimerSlot* ptr = MainTimer;
	
	for(index = 0; index != MainTimerQueueSize + 1; index++) // Обнуляем все таймеры.
	{
		ptr->GoToTask = Idle;
		ptr->Time = 0;
		ptr++;

	}
}


//Пустая процедура - простой ядра.

void SetTimerTask(TPTR TS, uint16_t NewTime)
{
	uint8_t		index = 0;
	//uint8_t		index_backup = 0;
	uint8_t		nointerrupted = 0;
	
	TimerSlot* ptr = MainTimer;
	//boolean flag_one_use_find_idle = 0;

	if (STATUS_REG & (1<<Interrupt_Flag)) 			// Проверка запрета прерывания, аналогично функции выше
	{
		Disable_Interrupt
		nointerrupted = 1;
	}


	for(index=0;index!=MainTimerQueueSize+1;index++)	//Прочесываем очередь таймеров
	{
		if (ptr->GoToTask == TS)
		{
			ptr->Time = NewTime;
			// Перезаписываем ей выдержку
			if (nointerrupted) 	Enable_Interrupt		// Разрешаем прерывания если не были запрещены.
			return;
		}
		ptr++;
		
	}
	
	
	ptr = MainTimer;
	for(index=0;index!=MainTimerQueueSize+1;index++)	// Если не находим похожий таймер, то ищем любой пустой
	{
		

		if (ptr->GoToTask == Idle)
		{
			ptr->GoToTask = TS;			// Заполняем поле перехода задачи
			ptr->Time = NewTime;		// И поле выдержки времени
			
			if (nointerrupted) 	Enable_Interrupt	// Разрешаем прерывания
			return;									// Выход.
		}
		ptr++;
	}												// тут можно сделать return c кодом ошибки - нет свободных таймеров
}



void TimerService(void)
{
	uint8_t index;
	
	TimerSlot* ptr = MainTimer;

	for(index = 0;index != MainTimerQueueSize + 1;index++)		// Прочесываем очередь таймеров
	{
		

		if(ptr->GoToTask == Idle) continue;		// Если нашли пустышку - щелкаем следующую итерацию

		if(ptr->Time !=1)						// Если таймер не выщелкал, то щелкаем еще раз.
		{												// To Do: Вычислить по тактам, что лучше !=1 или !=0.
			ptr->Time --;						// Уменьшаем число в ячейке если не конец.
		}
		else
		{
			// Дощелкали до нуля? Пихаем в очередь задачу
		QueueMain.push_back(ptr->GoToTask);
			ptr->GoToTask = Idle;				// А в ячейку пишем затычку
		}
		
		ptr++;
	}
}