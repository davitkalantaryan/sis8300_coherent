/*
 *	File: sis8300_io.h
 *
 *	Created on: Jul 17, 2015
 *	Author: Davit Kalantaryan (Email: davit.kalantaryan@desy.de)
 *
 *
 */
#ifndef __sis8300_io_h__
#define __sis8300_io_h__

#define	_NUMBER_OF_CHANNELS_					10

#define SIS8300_SAMPLE_CONTROL_REG				0x11
#define SIS8300_CLOCK_DISTRIBUTION_MUX_REG		0x40
#define SIS8300_SAMPLE_START_ADDRESS_CH1_REG	0x120
#define SIS8300_SAMPLE_LENGTH_REG				0x12A

#if 0
#define	_NUMBER_OF_RING_BUFFERS_				4
#define	__HEADER_SIZE__							256
#define	__OFSET_TO_BUFFER_INDEX__				4
#define	__OFSET_TO_EVENT_NUMBER__				8

#define	ONE_CHANNEL_SIZE(_a_number_of_samples)	((_a_number_of_samples)*2)
#define	DMA_TRANSFER_LEN(_a_number_of_samples)	(ONE_CHANNEL_SIZE(_a_number_of_samples)*_NUMBER_OF_CHANNELS_)
#define	ONE_BUFFER_SIZE(_a_dma_transfer_len)	((_a_dma_transfer_len) + __HEADER_SIZE__)
#define	WHOLE_MEMORY_SIZE(_a_one_buffer_size)	((_a_one_buffer_size)*_NUMBER_OF_RING_BUFFERS_ )

#define OFFSET_TO_BUFFER(_a_buffer_num, _a_one_buffer_size)				((_a_one_buffer_size)*(_a_buffer_num))
#define	_BUFFER_PTR_(_a_offset_to_buffer,_a_p_shared)					((char*)(_a_p_shared)+(_a_offset_to_buffer))
#define	EVENT_NUMBER_PTR(_a_offset_to_buffer,_a_p_shared)				(_BUFFER_PTR_(_a_offset_to_buffer,_a_p_shared)+__OFSET_TO_EVENT_NUMBER__)
#define	DMA_BUFFER_PTR(_a_offset_to_buffer,_a_p_shared)					(_BUFFER_PTR_(_a_offset_to_buffer,_a_p_shared)+__HEADER_SIZE__)
#define	CHANNEL_BUFFER_PTR(_a_offset_to_buffer,\
						_a_n_channel,_a_channel_size,_a_p_shared)		(DMA_BUFFER_PTR(_a_offset_to_buffer,_a_p_shared)+(a_n_channel)*(a_channel_size))
#endif

#include "mtcagen_io.h"


#define	SIS8300_IOC							's'

//////////////////////////////////////////////////////////////////////////
#define SIS8300_TEST1						_IOWR(SIS8300_IOC,51, int)
//#define SIS8300_START_SAMPLING				_IOWR(SIS8300_IOC,52)
#define SIS8300_START_DMA					_IO2(SIS8300_IOC,53)
#define SIS8300_NUMBER_OF_SAMPLES			_IO2(SIS8300_IOC,56)
#define SIS8300_WAIT_FOR_DMA_INF			_IO2(SIS8300_IOC,57)
#define SIS8300_WAIT_FOR_DMA_TIMEOUT		_IOW2(SIS8300_IOC,58, int)
#define SIS8300_SET_NUMBER_OF_SAMPLES		_IOWR(SIS8300_IOC,59, int)


#endif  /* #ifndef __sis8300_io_h__ */
