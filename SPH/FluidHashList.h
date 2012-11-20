#ifndef FLUID_HASH_LIST_H
#define FLUID_HASH_LIST_H

// electrodruid TODO - sort the news/deletes by having a pool of nodes. Again, it's a memory hit
// but it should improve performance and cut down on memory fragmentation
class cFluidHashList
{
public:

	struct node
	{
		int data;
		node *pNext;
	};

	cFluidHashList() : mpHead(NULL), mpTail(NULL), mpIterator(NULL) {}

	~cFluidHashList() {}

	void PushBack(int num)
	{
		if(mpHead == NULL)
		{
			mpHead = new node;
			mpHead->data = num;
			mpHead->pNext = NULL;
			mpTail = mpHead;
		}
		else
		{
			node *t = new node;
			t->data = num;
			t->pNext = NULL;
			
			mpTail->pNext = t;
			mpTail = t;
		}
	}

	// Call this to delete grid cell lists at the end of a frame, but not on the neighbours
	// list partway through the frame
	void Clear()
	{
		node *q,*r;
		q = mpHead;

		while(q!=NULL)
		{
			r = q->pNext;

			delete q;

			q = r;
		}

		mpHead = mpTail = mpIterator = NULL;
	}

	node* pHead()
	{
		return mpHead;
	}

	node* pTail()
	{
		return mpTail;
	}

	void SetHead(node* pNode)
	{
		mpHead = pNode;
	}

	void SetTail(node* pNode)
	{
		mpTail = pNode;
	}

	// Call this on a neighbours list, NOT on a grid list
	void Splice(node* pList2Head, node* pList2Tail)
	{
		mpTail->pNext = pList2Head;
		mpTail = pList2Tail;
	}

	// Call this on the grid lists, NOT on the neighbours list
	void UnSplice()
	{
		if (mpTail)
		{
			mpTail->pNext = NULL;
		}
	}

	inline void ResetIterator()
	{
		mpIterator = mpHead;
	}

	int GetNext()
	{
		if(mpIterator)
		{
			int data = mpIterator->data;
			mpIterator = mpIterator->pNext;
			return data;
		}

		return -1;
	}

	// electrodruid TODO - Cache this rather than recalculating every time
	int GetSize()
	{
		node *q;
		int c=0;
		for(q=mpHead ; q != NULL ; q = q->pNext)
		{
			c++;
		}

		return c;
	}

	inline bool IsEmpty()
	{
		return mpHead == NULL;
	}

private:

	node *mpHead;
	node *mpTail;
	node *mpIterator;
};






#endif