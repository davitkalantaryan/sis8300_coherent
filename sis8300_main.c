/*
 *	File: sis8300_main.c
 *
 *	Created on: Jul 17, 2015
 *	Author: Davit Kalantaryan (Email: davit.kalantaryan@desy.de)
 *
 *
 */


#define		_DEFAULT_NUMBER_OF_SAMPLES2_		2048

//#define	MAY_BE_DELETED
#define	MAY_BE_DELETED	__attribute__((deprecated))

//#define TEST_STREAMING_DMA

#define	NUMBER_OF_BOARDS						16
#define SIS8300DEVNAME							"sis8300"	                    /* name of device */
#define SIS8300_VENDOR_ID						0x1796	/* FZJZEL vendor ID */
#define SIS8300_DEVICE_ID						0x0018	/* SIS8300 dev board device ID */
#define SIS8300L_DEVICE_ID						0x0019	/* SIS8300L dev board device ID */

#define SIS8300_MEM_MAX_SIZE					536870912 // 512MByte

#define SIS8300_ACQUISITION_CONTROL_STATUS_REG	0x10
//#define SIS8300_SAMPLE_CONTROL_REG				0x11
//#define SIS8300_CLOCK_DISTRIBUTION_MUX_REG		0x40
//#define SIS8300_SAMPLE_START_ADDRESS_CH1_REG	0x120
#define DMA_READ_DST_ADR_LO32					0x200
#define DMA_READ_DST_ADR_HI32					0x201
#define DMA_READ_SRC_ADR_LO32					0x202
#define DMA_READ_LEN							0x203
#define DMA_READ_CTRL							0x204
#define IRQ_ENABLE								0x220
#define IRQ_STATUS								0x221
#define IRQ_CLEAR								0x222

#define DMA_READ_DONE							0
#define DMA_READ_START							0

#include <linux/time.h>

#define GETTIMEMS(...)	\
({ \
	long __lnTime; \
	struct timeval __time; \
	\
	do_gettimeofday(&__time); \
	__lnTime = __time.tv_sec * 1000 + __time.tv_usec / 1000; \
	__lnTime; \
})


#define DEBUGNEW2(...)	\
{ \
	/*if(s_nDoDebug)*/ \
	{\
		long __lnMs=GETTIMEMS(); \
		printk(KERN_ALERT "%ld",__lnMs); \
		printk(KERN_CONT __VA_ARGS__); \
	}\
}

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include "mtcagen_exp.h"
#include "debug_functions.h"
#include "sis8300_io.h"
#include "version_dependence.h"
#include "adc_timer_interface_io.h"

static int s_vnSlotSamplePairs[NUMBER_OF_BOARDS] = { 0 };
//static int s_nDebugModules = 0;
static int s_nPrintSis8300DebugInfo = 0;

#ifndef WIN32
MODULE_AUTHOR("David Kalantaryan");
MODULE_DESCRIPTION("Driver for struck sis8300 board");
MODULE_VERSION("1.1.0");
MODULE_LICENSE("Dual BSD/GPL");

module_param_array_named(SAMPLES, s_vnSlotSamplePairs, int, NULL,S_IRUGO | S_IWUSR);
module_param_named(SAMPLES0, s_vnSlotSamplePairs[0], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES1, s_vnSlotSamplePairs[1], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES2, s_vnSlotSamplePairs[2], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES3, s_vnSlotSamplePairs[3], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES4, s_vnSlotSamplePairs[4], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES5, s_vnSlotSamplePairs[5], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES6, s_vnSlotSamplePairs[6], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES7, s_vnSlotSamplePairs[7], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES8, s_vnSlotSamplePairs[8], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES9, s_vnSlotSamplePairs[9], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES10, s_vnSlotSamplePairs[10], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES11, s_vnSlotSamplePairs[11], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES12, s_vnSlotSamplePairs[12], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES13, s_vnSlotSamplePairs[13], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES14, s_vnSlotSamplePairs[14], int, S_IRUGO | S_IWUSR);
module_param_named(SAMPLES15, s_vnSlotSamplePairs[15], int, S_IRUGO | S_IWUSR);
module_param_named(sis8300debug, s_nPrintSis8300DebugInfo, int, S_IRUGO | S_IWUSR);
#endif

//#define DEBUSIS8300(...)	if(s_nPrintSis8300DebugInfo){ALERTCT(__VA_ARGS__);}
#define DEBUSIS8300(...)	if(s_nPrintSis8300DebugInfo){printk(KERN_ALERT __VA_ARGS__);}


struct SSIS8300Zn
{
	struct pciedev_dev*			dev;
	dma_addr_t					sharedBusAddress;
	void*						sharedAddress;
	u_int32_t					numberOfSamples;
	u_int32_t					dmaTransferLen; 
	u_int32_t					oneBufferLen;		// dmaTransferLen + header
	int							mmap_usage;			// Only when this is 0 shared memory size can be increased
	wait_queue_head_t			waitDMAcomplete;
	long						numberOfdmaDone;	// after eache dma transfer increment by one

	struct task_struct*			dmaTaskStruct;	// task that run threads for starting dma after timer interrupt
	long*						gen_event_ptr;		// Pointer to gen event number from timer driver
	struct SIrqWaiterStruct*	timerWaitStruct;	// Structure for waiting and responding timer interrupts
	char						vcWaiterNm[16];
	char						vcEvNumNm[16];
	int							dma_active;
	struct mutex				management_mutex;

};

static struct file_operations	s_sis833FileOps /*= g_default_mtcagen_fops_exp*/;

static void sis8300_vma_open(struct vm_area_struct *a_vma)
{
	struct SSIS8300Zn* pSis8300 = a_vma->vm_private_data;
	mutex_lock(&(pSis8300->management_mutex));
	++pSis8300->mmap_usage;
	mutex_unlock(&(pSis8300->management_mutex));
}


static void sis8300_vma_close(struct vm_area_struct *a_vma)
{
	struct SSIS8300Zn* pSis8300 = a_vma->vm_private_data;
	mutex_lock(&(pSis8300->management_mutex));
	--pSis8300->mmap_usage;
	mutex_unlock(&(pSis8300->management_mutex));
}


static struct vm_operations_struct sis8300_mmap_vm_ops =
{
	.open = sis8300_vma_open,
	.close = sis8300_vma_close,
	//.fault = daq_vma_fault,	// Finally page fault exception should be used to do 
								// page size changing really dynamoc
};

static struct SSIS8300Zn s_vDevices[NUMBER_OF_BOARDS];
static int s_nOtherDeviceInterrupt = 0;

static void FreeDmaPages(struct SSIS8300Zn* a_pSis8300);
static int SetNumberOfSamplings(struct SSIS8300Zn* a_pSis8300, u_int32_t a_unNumberOfSamplings);


#if LINUX_VERSION_CODE < 0x20613 // irq_handler_t has changed in 2.6.19
static irqreturn_t sis8300_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#else
static irqreturn_t sis8300_interrupt(int irq, void *dev_id)
#endif
{
	struct SSIS8300Zn*	dev = dev_id;
	struct pciedev_dev*	pciedev = dev->dev;
	char*				deviceAddress = pciedev->memmory_base[0];
	uint32_t			intreg;

	// Because interrupt line is shared between diferent devices
	// First we are checking interrupt register to see if interrupt is done by 
	// sis8300 board
	intreg = ioread32(deviceAddress + IRQ_STATUS * 4);
	smp_rmb();
	// If not exiting with code, that interrupt is not handled
	if (intreg == 0){ ++s_nOtherDeviceInterrupt; return IRQ_NONE; }

	// Switching to next buffer in our ring buffers, for 
	// next dma transfer
	*((int*)dev->sharedAddress) = (*((int*)dev->sharedAddress) + 1) % _NUMBER_OF_RING_BUFFERS_;

	// Waking up all processes, waiting for data from board
	++dev->numberOfdmaDone;
	dev->dma_active = 0;
	wake_up(&dev->waitDMAcomplete);

	// Piti nayvi
	// Preparing device for next trigger
	iowrite32(0x00000002, (deviceAddress + 0x40));
	smp_wmb();

	// Interrupt acknowledgment
	// Informing device, that interrupt handled!
	iowrite32(intreg, (deviceAddress + IRQ_CLEAR * 4));
	smp_wmb();

	return IRQ_HANDLED;
}



static int dma_thread_func(void *a_data);

/*
 * This probe function will be called, when
 * parent MTCA general driver finds device
 */
static int ProbeFunction(struct pciedev_dev* a_dev, void* a_pData)
{
	char*	deviceAddress;

	int nReturn = 0;
	int device = a_dev->pciedev_pci_dev->device;
	int brdNum = a_dev->brd_num % NUMBER_OF_BOARDS;

	// Just checking if this entry is free or not
	if (unlikely(s_vDevices[brdNum].dev)) return -1; // Already in use
	s_vDevices[brdNum].dev = a_dev;

	// Some output
	ALERTCT("device:\"%s\"(ID=0x%x);a_dev=%p; a_pData=%p\n",
		device == SIS8300_DEVICE_ID ? "SIS8300_DEVICE_ID" : "SIS8300L_DEVICE_ID", device,a_dev, a_pData);

	// Initialization of device
	nReturn = Mtcagen_GainAccess_exp(a_dev, 0, NULL, &s_sis833FileOps,
		SIS8300DEVNAME, "%ss%d", SIS8300DEVNAME, a_dev->brd_num);
	if (nReturn)
	{
		ERRCT("nReturn = %d\n",nReturn);
		s_vDevices[brdNum].dev = NULL;
		return nReturn;
	}

	memset(&s_vDevices[brdNum], 0, sizeof(struct SSIS8300Zn));
	s_vDevices[brdNum].dev = a_dev;
	a_dev->parent = &s_vDevices[brdNum];
	mutex_init(&(s_vDevices[brdNum].management_mutex)); /// new

	//return 0;
	// setting some initial number of samples
	nReturn = SetNumberOfSamplings(&(s_vDevices[brdNum]), s_vnSlotSamplePairs[brdNum]);
	if (nReturn)
	{
		Mtcagen_remove_exp(a_dev, 0, NULL);
		s_vDevices[brdNum].dev = NULL;
		ERRCT("nReturn = %d\n", nReturn);
		return nReturn;
	}

	init_waitqueue_head(&s_vDevices[brdNum].waitDMAcomplete);
	nReturn = request_irq(a_dev->pci_dev_irq, &sis8300_interrupt, IRQF_SHARED | IRQF_DISABLED, SIS8300DEVNAME, &(s_vDevices[brdNum]));
	if (nReturn)
	{
		FreeDmaPages(&(s_vDevices[brdNum]));
		Mtcagen_remove_exp(a_dev, 0, NULL);
		s_vDevices[brdNum].dev = NULL;
		return nReturn;
	}

	deviceAddress = a_dev->memmory_base[0];

	/// disable all IRQs  // erevi sa petq request_irq -ic araj anel
	iowrite32(0xffff0000, ((void*)(deviceAddress + IRQ_ENABLE * 4)));
	smp_wmb();

	/// enable dma read interrupt
	iowrite32((1 << DMA_READ_DONE), ((void*)(deviceAddress + IRQ_ENABLE * 4)));
	smp_wmb();

	// Piti nayvi
	// Preparing device for next trigger
	iowrite32(0x00000002, (deviceAddress + 0x40));
	smp_wmb();

	//#define	_ENTRY_NAME_WAITERS_	"timer_waiters_for"
	//#define	_ENTRY_NAME_GEN_EVENT_	"gen_event_entry_name"
	s_vDevices[brdNum].timerWaitStruct = NULL;
	s_vDevices[brdNum].gen_event_ptr = &s_vDevices[brdNum].numberOfdmaDone;
	s_vDevices[brdNum].dmaTaskStruct = kthread_run(dma_thread_func, &s_vDevices[brdNum], "dmathreadforADC%d", brdNum);

	return nReturn;
}


static void disconnect_from_timer(struct SSIS8300Zn* a_pSis8300, int a_type)
{
	switch (a_type)
	{
	case 0:
		if (a_pSis8300->timerWaitStruct){ PutEntryToGlobalContainer(a_pSis8300->vcWaiterNm); }
		a_pSis8300->timerWaitStruct = NULL;
		break;
	case 1:
		if (a_pSis8300->gen_event_ptr != &a_pSis8300->numberOfdmaDone){ PutEntryToGlobalContainer(a_pSis8300->vcEvNumNm); }
		a_pSis8300->gen_event_ptr = &a_pSis8300->numberOfdmaDone;
		break;
	case 2:
		if (a_pSis8300->timerWaitStruct){ PutEntryToGlobalContainer(a_pSis8300->vcWaiterNm); }
		if (a_pSis8300->gen_event_ptr != &a_pSis8300->numberOfdmaDone){ PutEntryToGlobalContainer(a_pSis8300->vcEvNumNm); }
		a_pSis8300->timerWaitStruct = NULL;
		a_pSis8300->gen_event_ptr = &a_pSis8300->numberOfdmaDone;
		break;
	default:
		break;
	}
}


static void RemoveFunction(struct pciedev_dev* a_dev, void* a_pData)
{
	struct SSIS8300Zn*		pSis8300 = a_dev->parent;
	char*	deviceAddress = pSis8300->dev->memmory_base[0];

	DEBUGCT("\n");

	kthread_stop(pSis8300->dmaTaskStruct);
	pSis8300->dmaTaskStruct = NULL;

	/// Disabling all interrupts
	iowrite32(0xFFFF0000, (void*)(deviceAddress + IRQ_ENABLE * 4));
	smp_wmb();

	free_irq(pSis8300->dev->pci_dev_irq, pSis8300);

	disconnect_from_timer(pSis8300,2);
	//mutex_lock_interruptible(&(a_pSis8300->management_mutex));
	FreeDmaPages(pSis8300);
	pSis8300->dev = NULL;
	//mutex_unlock(&(a_pSis8300->management_mutex));
	Mtcagen_remove_exp(a_dev, 0, NULL);
}


/*
 * 
 * 
 */
static void FreeDmaPages(struct SSIS8300Zn* a_pSis8300)
{
	u_int32_t unBytes = WHOLE_MEMORY_SIZE(a_pSis8300->oneBufferLen);

	ALERTCT("!!!!!!!!!!!!!!!!! unBytess=%u, a_pSis8300->sharedAddress=%p, a_pSis8300->sharedBusAddress=%d\n", 
		unBytes, a_pSis8300->sharedAddress, (int)a_pSis8300->sharedBusAddress);

	if (a_pSis8300->sharedBusAddress && a_pSis8300->sharedAddress && a_pSis8300->numberOfSamples)
	{
		pci_free_consistent(a_pSis8300->dev->pciedev_pci_dev, unBytes, a_pSis8300->sharedAddress, a_pSis8300->sharedBusAddress);
	}
	a_pSis8300->sharedBusAddress = 0;
	a_pSis8300->sharedAddress = NULL;
	a_pSis8300->numberOfSamples = 0;
	a_pSis8300->dmaTransferLen = 0;
	a_pSis8300->oneBufferLen = 0;
}


/*
 *
 */
static int SetNumberOfSamplings(struct SSIS8300Zn* a_pSis8300, u_int32_t a_unNumberOfSamplings)
{
	u_int32_t	unNumberOfSamples0 = a_pSis8300->numberOfSamples;
	u_int32_t unBytes;
	int* ringIndexPtr;

	ALERTCT("\n");
	if (a_unNumberOfSamplings <= 0 || a_unNumberOfSamplings>(SIS8300_MEM_MAX_SIZE / 2 / _NUMBER_OF_CHANNELS_))return 0;
	a_unNumberOfSamplings = (a_unNumberOfSamplings % 16) ? (16 + a_unNumberOfSamplings - (a_unNumberOfSamplings % 16)) : a_unNumberOfSamplings;
	if (unNumberOfSamples0 != a_unNumberOfSamplings)
	{
		if (mutex_lock_interruptible(&(a_pSis8300->management_mutex))){return ERESTARTSYS;}
		while (a_pSis8300->dma_active) { msleep(1); }
		if (a_pSis8300->mmap_usage != 0)
		{
			mutex_unlock(&(a_pSis8300->management_mutex));
			return -EBUSY; 
		}
		FreeDmaPages(a_pSis8300);
		unBytes = WHOLE_MEMORY_SIZE(ONE_BUFFER_SIZE(DMA_TRANSFER_LEN(a_unNumberOfSamplings)));
		a_pSis8300->sharedAddress = pci_alloc_consistent(a_pSis8300->dev->pciedev_pci_dev, unBytes, &a_pSis8300->sharedBusAddress);

		ALERTCT("!!!!!!!!!!!!! unBytess=%u, a_pSis8300->destAddress=%p, a_pSis8300->sharedBusAddress=%d, __pa(addr)=%d\n",
			unBytes, a_pSis8300->sharedAddress, (int)a_pSis8300->sharedBusAddress, (int)__pa(a_pSis8300->sharedAddress));

		if (!a_pSis8300->sharedAddress || !a_pSis8300->sharedBusAddress)
		{
			FreeDmaPages(a_pSis8300);
			mutex_unlock(&(a_pSis8300->management_mutex));
			ERRCT("Unable to locate memory!\n");
			return ENOMEM;
		}

		a_pSis8300->numberOfSamples = a_unNumberOfSamplings;
		a_pSis8300->dmaTransferLen = DMA_TRANSFER_LEN(a_unNumberOfSamplings);
		a_pSis8300->oneBufferLen = ONE_BUFFER_SIZE(a_pSis8300->dmaTransferLen);
		mutex_unlock(&(a_pSis8300->management_mutex));
	}
	ringIndexPtr = a_pSis8300->sharedAddress;
	*ringIndexPtr = (_NUMBER_OF_RING_BUFFERS_ - 1);

	return 0;
}


static int prepare_dma_read_prvt(struct SSIS8300Zn* a_pSis8300, dma_addr_t a_sysMemAddress,
	u_int32_t a_device_offset, u_int32_t a_count);


static int StartDma2(struct SSIS8300Zn* a_pSis8300, long* a_gen_event_ptr)
{
	int nBufferIndex = (1 + *((int*)a_pSis8300->sharedAddress)) % _NUMBER_OF_RING_BUFFERS_;
	size_t unOffsetToBuffer = OFFSET_TO_BUFFER(nBufferIndex, a_pSis8300->oneBufferLen);
	int* pnEventNumber = (int*)EVENT_NUMBER_PTR(unOffsetToBuffer, a_pSis8300->sharedAddress);
	dma_addr_t sysMemorForDma = a_pSis8300->sharedBusAddress + unOffsetToBuffer + __HEADER_SIZE__;

	*((int*)((char*)a_pSis8300->sharedAddress + unOffsetToBuffer + __OFSET_TO_BUFFER_INDEX__)) = nBufferIndex;
	//*pnEventNumber = (int)(*a_pSis8300->gen_event_int_irq_ptr);
	*pnEventNumber = (int)(*a_gen_event_ptr);

	return prepare_dma_read_prvt(a_pSis8300, sysMemorForDma, 0, a_pSis8300->dmaTransferLen);
}



static int sis8300_mmap(struct file *a_filp, struct vm_area_struct *a_vma)
{
	struct pciedev_dev*		dev = a_filp->private_data;
	struct SSIS8300Zn*		pSis8300 = dev->parent;
	unsigned long sizeFrUser = a_vma->vm_end - a_vma->vm_start;
	unsigned long sizeOrig = WHOLE_MEMORY_SIZE(pSis8300->oneBufferLen);
	unsigned int size = sizeFrUser>sizeOrig ? sizeOrig : sizeFrUser;

	if (mutex_lock_interruptible(&(pSis8300->management_mutex))) return ERESTARTSYS;
	
	if (remap_pfn_range(a_vma, a_vma->vm_start, virt_to_phys((void *)pSis8300->sharedAddress) >> PAGE_SHIFT, size, a_vma->vm_page_prot) < 0)
	{
		mutex_unlock(&(pSis8300->management_mutex));
		ERRCT("remap_pfn_range failed\n");
		return -EIO;
	}

	a_vma->vm_private_data = pSis8300;
	a_vma->vm_ops = &sis8300_mmap_vm_ops;
	mutex_unlock(&(pSis8300->management_mutex));
	sis8300_vma_open(a_vma);

	return 0;
}


#define SIS8300REGWRITE(a_device,a_register_num,a_value) \
{ \
	char*		deviceAddress = (a_device)->dev->memmory_base[0]; \
	iowrite32((a_value),((void*)(deviceAddress + (a_register_num) * 4))); \
	smp_wmb(); \
}



static int prepare_dma_read_prvt(struct SSIS8300Zn* a_pSis8300, dma_addr_t a_sysMemAddress,
	u_int32_t a_device_offset, u_int32_t a_count)
{
	uint64_t			dmaspace_addr;
	uint32_t			temp, aquision_state;
	struct SSIS8300Zn*	sisdevice = a_pSis8300;
	char*				deviceAddress = a_pSis8300->dev->memmory_base[0];

	DEBUGNEW("!!!!!!!!!! count=%u\n", a_count);
	if (mutex_lock_interruptible(&(a_pSis8300->management_mutex))) { return ERESTARTSYS; }

	aquision_state = ioread32((void*)(deviceAddress + SIS8300_ACQUISITION_CONTROL_STATUS_REG * 4));//deviceAddress
	smp_rmb();
	if (aquision_state & 0x00000001)
	{
		// some more reporting
		mutex_unlock(&(a_pSis8300->management_mutex));
		printk(KERN_WARNING "!!!!!!!!!!!!!!!!!!+++++++++++ Sampling busy!!!\n");
		return -EBUSY;
	}

	SIS8300REGWRITE(sisdevice, DMA_READ_SRC_ADR_LO32, a_device_offset);

	// set destination address
	dmaspace_addr = (uint64_t)a_sysMemAddress;
	temp = (uint32_t)dmaspace_addr;
	SIS8300REGWRITE(sisdevice, DMA_READ_DST_ADR_LO32, temp);

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	temp = (uint32_t)(dmaspace_addr >> 32);
#else
	temp = 0;
#endif
	SIS8300REGWRITE(sisdevice, DMA_READ_DST_ADR_HI32, temp);

	// set transfer len
	SIS8300REGWRITE(sisdevice, DMA_READ_LEN, a_count);

	// disable interrupt
	SIS8300REGWRITE(sisdevice, IRQ_ENABLE, 0xFFFF0000);

	// enable interrupt
	SIS8300REGWRITE(sisdevice, IRQ_ENABLE, (1 << DMA_READ_DONE));

	//sisdevice->intr_flag = 0;
	// start
	sisdevice->dma_active = 1;
	SIS8300REGWRITE(sisdevice, DMA_READ_CTRL, (1 << DMA_READ_START));
	mutex_unlock(&(a_pSis8300->management_mutex));

	return 0;
}


#include <linux/delay.h>

static int dma_thread_func(void *a_data)
{
	struct SSIS8300Zn* pSisDevice = (struct SSIS8300Zn*)a_data;
	struct SIrqWaiterStruct* pWaitStr = pSisDevice->timerWaitStruct;

	long lnIRQindex;
	long lnCurTime, lnOldTime = 0;
	int nSamplingBusy = 0;

	while (!kthread_should_stop())
	{
		pWaitStr = pSisDevice->timerWaitStruct;

		if (pWaitStr)
		{
			lnIRQindex = pWaitStr->numberOfIRQs;
			if (wait_event_interruptible_timeout(pWaitStr->waitIRQ, lnIRQindex < pWaitStr->numberOfIRQs, 2000)>0)
			{
				lnCurTime = GETTIMEMS();
				lnOldTime = lnCurTime;

				if (StartDma2(pSisDevice, pSisDevice->gen_event_ptr))
				{
					if (nSamplingBusy == 0)
					{
						printk(KERN_ALERT "Sampling busy!\n");
						nSamplingBusy = 1;
					}
				}
				else
				{
					if (nSamplingBusy == 1)
					{
						printk(KERN_ALERT "Sampling not busy!\n");
						nSamplingBusy = 0;
					}
				}
			}// if (wait_event_interruptible_timeout(pWaitStr->waitIRQ, lnIRQindex < pWaitStr->numberOfIRQs, 2000)>0)
		} // if (pWaitStr)
		else
		{
			msleep(200);
		}


	} // end while (!kthread_should_stop())

	do_exit(0); //??
	return 0;
}



static long  sis8300_ioctl(struct file *a_filp, unsigned int a_cmd, unsigned long a_arg)
{
	struct pciedev_dev*		dev			= a_filp->private_data;
	struct SSIS8300Zn*		pSis8300	= dev->parent;
	u64						ulnJiffiesTmOut;
	int						nReturn		= 0;
	int32_t					nUserValue;
	long					lnNextNumberOfDmaDone = pSis8300->numberOfdmaDone + 1;

	if (unlikely(!dev->dev_sts))
	{
		WARNCT("device has been taken out!\n");
		return -ENODEV;
	}

	switch (a_cmd)
	{
	case SIS8300_TEST1:
		DEBUGNEW("SIS8300_TEST1\n");
		break;

	//case SIS8300_SHARED_MEMORY_SIZE:break;
//#define SIS8300_START_SAMPLING				_IOWR(SIS8300_IOC,52)
//#define SIS8300_START_DMA					_IOWR(SIS8300_IOC,53)
	//case SIS8300_START_SAMPLING:
	//	break;

	case SIS8300_START_DMA:
		StartDma2(pSis8300, &pSis8300->numberOfdmaDone);
		break;

	case SIS8300_NUMBER_OF_SAMPLES:
		DEBUGNEW("SIS8300_NUMBER_OF_SAMPLES\n");
		nReturn = pSis8300->numberOfSamples;
		break;

	case SIS8300_WAIT_FOR_DMA_INF:
		DEBUGNEW("SIS8300_WAIT_FOR_DMA_INF\n");
		nReturn = wait_event_interruptible(pSis8300->waitDMAcomplete, lnNextNumberOfDmaDone <= pSis8300->numberOfdmaDone);
		break;

	case SIS8300_WAIT_FOR_DMA_TIMEOUT:
		DEBUGNEW("SIS8300_WAIT_FOR_DMA_TIMEOUT\n");
		if (copy_from_user(&nUserValue, (int32_t*)a_arg, sizeof(int32_t)))
		{
			nReturn = -EFAULT;
			goto returnPoint;
		}
		ulnJiffiesTmOut = msecs_to_jiffies(nUserValue);
		nReturn = wait_event_interruptible_timeout(pSis8300->waitDMAcomplete, lnNextNumberOfDmaDone <= pSis8300->numberOfdmaDone, ulnJiffiesTmOut);
		break;

	case SIS8300_SET_NUMBER_OF_SAMPLES:
		DEBUGNEW("SIS8300_SET_NUMBER_OF_SAMPLES\n");

		if (copy_from_user(&nUserValue, (int32_t*)a_arg, sizeof(int32_t)))
		{
			nReturn = -EFAULT;
			goto returnPoint;
		}

		if (nUserValue > 0){ return SetNumberOfSamplings(pSis8300, nUserValue); }
		break;

	case MTCA_SET_ADC_TIMER:
		DEBUSIS8300("MTCA_SET_ADC_TIMER\n");
		disconnect_from_timer(pSis8300,0);
		pSis8300->vcWaiterNm[0] = 0;
		nReturn = strncpy_from_user(pSis8300->vcWaiterNm, (const char*)a_arg, 16);
		DEBUSIS8300("!!!!!!! ret=%d, wait_name=\"%s\"\n", nReturn, pSis8300->vcWaiterNm);
		if (nReturn<=0) break;
		pSis8300->timerWaitStruct = FindAndUseEntryFromGlobalContainer(pSis8300->vcWaiterNm);
		DEBUSIS8300("!!!!!!!!!!!! waiter_name=%s\n", pSis8300->vcWaiterNm);
		break;

	case MTCA_SET_ADC_GEN_EVNT_SRC:
		DEBUSIS8300("MTCA_SET_ADC_GEN_EVNT_SRC\n");
		disconnect_from_timer(pSis8300, 1);
		nReturn = strncpy_from_user(pSis8300->vcEvNumNm, (const char*)a_arg, 16);
		DEBUSIS8300("!!!!!!! ret=%d\n", nReturn);
		if (nReturn<=0) break;
		pSis8300->gen_event_ptr = FindAndUseEntryFromGlobalContainer(pSis8300->vcEvNumNm);
		if (!pSis8300->gen_event_ptr){ pSis8300->gen_event_ptr = &pSis8300->numberOfdmaDone; }
		DEBUSIS8300("!!!!!!!!!!!! gen_event_name=%s\n", pSis8300->vcEvNumNm);
		break;

	default:
		DEBUGNEW("default\n");
		return mtcagen_ioctl_exp(a_filp, a_cmd, a_arg);
	}

returnPoint:
	return nReturn;
}


static void __exit sis8300_cleanup_module(void)
{
	SDeviceParams aDevParam;
	
	ALERTCT("\n");

	DEVICE_PARAM_RESET(&aDevParam);
	aDevParam.vendor = SIS8300_VENDOR_ID;

	aDevParam.device = SIS8300_DEVICE_ID;
	removeIDfromMainDriverByParam(&aDevParam);

	aDevParam.device = SIS8300L_DEVICE_ID;
	removeIDfromMainDriverByParam(&aDevParam);

}



static int __init sis8300_init_module(void)
{
	int i;
	unsigned int unRemnant, bit;
	SDeviceParams aDevParam;

	printk(KERN_ALERT "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! __USER_CS=%d(0x%x), __USER_DS=%d(0x%x), __KERNEL_CS=%d(%d), __KERNEL_DS=%d(%d)\n", 
		(int)__USER_CS, (int)__USER_CS, (int)__USER_DS, (int)__USER_DS, (int)__KERNEL_CS, (int)__KERNEL_CS, (int)__KERNEL_DS, (int)__KERNEL_DS);

	ALERTCT("============= version 9:\n");

	////////////////
	printk(KERN_ALERT "!!! vector={");
	for (i = 0; i < NUMBER_OF_BOARDS; ++i)
	{
		if (s_vnSlotSamplePairs[i] == 0)
		{
			s_vnSlotSamplePairs[i] = _DEFAULT_NUMBER_OF_SAMPLES2_;
		}
		else
		{
			unRemnant = s_vnSlotSamplePairs[i];
			for (bit = 0; unRemnant; unRemnant = (s_vnSlotSamplePairs[i] >> ++bit));
			unRemnant = 1 << (bit-1);
			if (unRemnant != s_vnSlotSamplePairs[i]){ s_vnSlotSamplePairs[i] = unRemnant << 1; }
		}
		printk(KERN_CONT "%d, ", s_vnSlotSamplePairs[i]);
	}
	printk(KERN_CONT "};\n");

	DEVICE_PARAM_RESET(&aDevParam);
	aDevParam.vendor = SIS8300_VENDOR_ID;

	memcpy(&s_sis833FileOps, &g_default_mtcagen_fops_exp, sizeof(struct file_operations));
	s_sis833FileOps.compat_ioctl = s_sis833FileOps.unlocked_ioctl = &sis8300_ioctl;
	s_sis833FileOps.mmap = &sis8300_mmap;
	s_sis833FileOps.owner = THIS_MODULE;

	aDevParam.device = SIS8300_DEVICE_ID;
	registerDeviceToMainDriver(&aDevParam, &ProbeFunction, &RemoveFunction, NULL);

	aDevParam.device = SIS8300L_DEVICE_ID;
	registerDeviceToMainDriver(&aDevParam, &ProbeFunction, &RemoveFunction, NULL);
	
	return 0; /* succeed */
}

module_init(sis8300_init_module);
module_exit(sis8300_cleanup_module);
