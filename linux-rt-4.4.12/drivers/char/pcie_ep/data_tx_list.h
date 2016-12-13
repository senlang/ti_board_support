#include <linux/types.h>




#ifndef TRUE
	#define _TRUE	1
#else
	#define _TRUE	TRUE	
#endif
		
#ifndef FALSE		
	#define _FALSE	0
#else
	#define _FALSE	FALSE	
#endif

#define _FAIL		0
#define _SUCCESS	1



/* Raw image data
 */

typedef struct raw_image_data {
	uint32_t dest_addr;
	uint32_t src_addr;
	uint16_t size;
	uint16_t flag;
} raw_packet_data_t;

static inline u32 _down_sema(struct semaphore *sema)
{
	if (down_interruptible(sema))
		return _FAIL;
	else
		return _SUCCESS;
}

void	init_tx_queue(void);
void 	enqueue_txData(uint32_t src,uint32_t dest,uint16_t out_size);
int 	dequeue_txData(raw_packet_data_t * p_output_image);
uint32_t queue_empty(void);
