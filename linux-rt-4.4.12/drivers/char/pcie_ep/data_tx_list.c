#include <linux/types.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/gfp.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include "data_tx_list.h"




typedef 	__kernel_size_t	SIZE_T;	

#define LIST_CONTAINOR(ptr, type, member) \
        ((type *)((char *)(ptr)-(SIZE_T)(&((type *)0)->member)))
//#define container_of(p,t,n) (t*)((p)-&(((t*)0)->n))


struct	__queue	{
	struct	list_head	queue;
	spinlock_t	lock;
	struct semaphore queue_sema;
};

typedef struct	__queue _queue;
typedef struct	list_head	_list;

struct trans_priv {
	_queue	transfer_queue;
};

struct data_obj {
	uint32_t dest_addr;
	uint32_t src_addr;
	uint16_t size;
	struct list_head list;
};



struct	trans_priv ptxpriv;

uint8_t* zmalloc(u32 sz)
{

	uint8_t 	*pbuf = kmalloc(sz,GFP_ATOMIC);

	if (pbuf != NULL) {
		memset(pbuf, 0, sz);
	}

	return pbuf;	
}


static _list *get_next(_list	*list)
{
	return list->next;
}	
static void list_delete(_list *plist)
{
	list_del_init(plist);
}


uint32_t	is_list_empty(struct list_head *phead)
{
	if (list_empty(phead))
		return _TRUE;
	else
		return _FALSE;	
}

uint32_t queue_empty(void)
{
	uint32_t retval;
	
	//spin_lock_bh(&(ptxpriv.transfer_queue.tx_lock));
	_down_sema(&(ptxpriv.transfer_queue.queue_sema));
	retval = list_empty(&(ptxpriv.transfer_queue.queue));
	up(&(ptxpriv.transfer_queue.queue_sema));

	//spin_unlock_bh(&(ptxpriv.transfer_queue.tx_lock));
	
	return retval;
}

void _init_listhead(_list *list)
{
	INIT_LIST_HEAD(list);
}

void	_init_queue(_queue	*pqueue)
{
	_init_listhead(&(pqueue->queue));
	spin_lock_init(&pqueue->lock);
	sema_init(&pqueue->queue_sema, 1);
}

void	init_tx_queue(void)
{
	_init_queue(&(ptxpriv.transfer_queue));
}



void list_insert_tail(_list *plist, _list *phead)
{
	list_add_tail(plist, phead);
}


int	enqueue_cmd(_queue *queue, struct data_obj *obj)
{
	//spin_lock_bh(&(ptxpriv.transfer_queue.tx_lock));
	_down_sema(&(ptxpriv.transfer_queue.queue_sema));
	list_insert_tail(&obj->list, &queue->queue);
	up(&(ptxpriv.transfer_queue.queue_sema));
	//spin_unlock_bh(&(ptxpriv.transfer_queue.tx_lock));

	return _SUCCESS;
}

struct	data_obj	*dequeue_cmd(_queue *queue)
{
	struct data_obj *obj;
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	
	//spin_lock_bh(&(ptxpriv.transfer_queue.tx_lock));
	_down_sema(&(ptxpriv.transfer_queue.queue_sema));
	if (list_empty(&(queue->queue)))
		obj = NULL;
	else
	{
		obj = LIST_CONTAINOR(get_next(&(queue->queue)), struct data_obj, list);
		list_delete(&obj->list);
	}
	up(&(ptxpriv.transfer_queue.queue_sema));
	//spin_unlock_bh(&(ptxpriv.transfer_queue.tx_lock));
	
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	return obj;
}
 
void enqueue_txData(uint32_t dest_addr,uint32_t src_addr,uint16_t out_size)
{
	struct data_obj *pdata_obj;
	
//	printk("%s[%d]\n",__FUNCTION__,__LINE__);
	if ((pdata_obj = (struct data_obj*)zmalloc(sizeof(struct data_obj))) == NULL)
	{
		return;
	}

	pdata_obj->dest_addr= dest_addr;
	pdata_obj->src_addr= src_addr;
	pdata_obj->size = out_size;
	
//	printk("pdata_obj->dest_addr = 0x%08x\n",pdata_obj->dest_addr);
//	printk("pdata_obj->src_addr = 0x%08x\n",pdata_obj->src_addr);
//	printk("pdata_obj->size = 0x%08x\n",pdata_obj->size);
	
	enqueue_cmd(&ptxpriv.transfer_queue, pdata_obj);
//	printk("%s[%d]\n",__FUNCTION__,__LINE__);
}

int dequeue_txData(raw_packet_data_t * p_output_image)
{
	struct data_obj *pdata;
	
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	if(!(pdata = dequeue_cmd(&ptxpriv.transfer_queue))) {
		return 0;
	}
	
	//printk("%s[%d]\n",__FUNCTION__,__LINE__);
	p_output_image->dest_addr = pdata->dest_addr;
	p_output_image->src_addr = pdata->src_addr;
	p_output_image->size = pdata->size;
	
	//printk("p_output_image->length = 0x%08x\n",p_output_image->length);
	//printk("p_output_image->data = 0x%08x\n",p_output_image->data);
	
	kfree(pdata);
	
	return 1;
}
