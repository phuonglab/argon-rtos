/*
 * Copyright (c) 2007-2014 Immo Software
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*!
 * @file
 * @brief Source for Ar microkernel channels.
 */

#include "ar_internal.h"
#include <string.h>
#include <assert.h>

using namespace Ar;

//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

static ar_status_t ar_channel_block(ar_channel_t * channel, ar_list_t & myDirList, void * value, uint32_t timeout);
static ar_status_t ar_channel_send_receive(ar_channel_t * channel, bool isSending, ar_list_t & myDirList, ar_list_t & otherDirList, void * value, uint32_t timeout);

//------------------------------------------------------------------------------
// Code
//------------------------------------------------------------------------------

// See ar_kernel.h for documentation of this function.
ar_status_t ar_channel_create(ar_channel_t * channel, const char * name, uint32_t width)
{
    if (!channel)
    {
        return kArInvalidParameterError;
    }

    memset(channel, 0, sizeof(ar_channel_t));
    channel->m_name = name ? name : AR_ANONYMOUS_OBJECT_NAME;
    channel->m_width = (width == 0) ? sizeof(void *) : width;

#if AR_GLOBAL_OBJECT_LISTS
    channel->m_createdNode.m_obj = channel;
    g_ar.allObjects.channels.add(&channel->m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS

    return kArSuccess;
}

// See ar_kernel.h for documentation of this function.
ar_status_t ar_channel_delete(ar_channel_t * channel)
{
    if (!channel)
    {
        return kArInvalidParameterError;
    }

    // Unblock all threads blocked on this channel.
    ar_thread_t * thread;
    while (channel->m_blockedSenders.m_head)
    {
        thread = channel->m_blockedSenders.m_head->getObject<ar_thread_t>();
        thread->unblockWithStatus(channel->m_blockedSenders, kArObjectDeletedError);
    }

    while (channel->m_blockedReceivers.m_head)
    {
        thread = channel->m_blockedReceivers.m_head->getObject<ar_thread_t>();
        thread->unblockWithStatus(channel->m_blockedReceivers, kArObjectDeletedError);
    }

#if AR_GLOBAL_OBJECT_LISTS
    g_ar.allObjects.channels.remove(&channel->m_createdNode);
#endif // AR_GLOBAL_OBJECT_LISTS

    return kArSuccess;
}

//! @brief Handles blocking a thread on a channel.
//!
//! The kernel must be locked prior to entry of this function.
ar_status_t ar_channel_block(ar_channel_t * channel, ar_list_t & myDirList, void * value, uint32_t timeout)
{
    // Nobody waiting, so we must block. Return immediately if the timeout is 0.
    if (timeout == kArNoTimeout)
    {
        return kArTimeoutError;
    }

    // Block this thread on the channel. Save the value pointer into the thread
    // object so the other side of this channel can access it.
    ar_thread_t * thread = g_ar.currentThread;
    thread->m_channelData = value;
    thread->block(myDirList, timeout);

    // Enable interrupts for this block, so we can enter the scheduler.
    {
        KernelUnlock guard;

        // Yield to the scheduler. We'll return when a call for the other direction, or
        // a timeout, wakes this thread.
        ar_kernel_enter_scheduler();
    }

    // We're back from the scheduler. Interrupts are still disabled.
    // Check for errors and exit early if there was one.
    if (thread->m_unblockStatus != kArSuccess)
    {
        myDirList.remove(&thread->m_blockedNode);
        return thread->m_unblockStatus;
    }

    return kArSuccess;
}

//! @brief Common channel send/receive code.
ar_status_t ar_channel_send_receive(ar_channel_t * channel, bool isSending, ar_list_t & myDirList, ar_list_t & otherDirList, void * value, uint32_t timeout)
{
    // Ensure that only 0 timeouts are specified when called from an IRQ handler.
    if (ar_port_get_irq_state() && timeout != 0)
    {
        return kArNotFromInterruptError;
    }

    KernelLock guard;

    // Are there any blocked threads for the opposite direction of this call?
    if (otherDirList.isEmpty())
    {
        return ar_channel_block(channel, myDirList, value, timeout);
    }
    else
    {
        // Get the first thread blocked on this channel.
        ar_thread_t * thread = otherDirList.m_head->getObject<ar_thread_t>();

        // Figure out the direction of the data transfer.
        void * src;
        void * dest;
        if (isSending)
        {
            src = value;
            dest = thread->m_channelData;
        }
        else
        {
            src = thread->m_channelData;
            dest = value;
        }

        // Do the transfer. Optimize word-sized channels so we don't have to call into memcpy().
        if (channel->m_width == sizeof(uint32_t))
        {
            *(uint32_t *)dest = *(uint32_t *)src;
        }
        else
        {
            memcpy(dest, src, channel->m_width);
        }

        // Unblock the other side.
        thread->unblockWithStatus(otherDirList, kArSuccess);

        // Invoke the scheduler if the unblocked thread is higher priority than the current one.
        if (thread->m_priority > g_ar.currentThread->m_priority)
        {
            guard.enable();

            ar_kernel_enter_scheduler();
        }
    }

    return kArSuccess;
}


// See ar_kernel.h for documentation of this function.
ar_status_t ar_channel_receive(ar_channel_t * channel, void * value, uint32_t timeout)
{
    if (!channel)
    {
        return kArInvalidParameterError;
    }

    return ar_channel_send_receive(channel, false, channel->m_blockedReceivers, channel->m_blockedSenders, value, timeout);
}

// See ar_kernel.h for documentation of this function.
ar_status_t ar_channel_send(ar_channel_t * channel, const void * value, uint32_t timeout)
{
    if (!channel)
    {
        return kArInvalidParameterError;
    }

    return ar_channel_send_receive(channel, true, channel->m_blockedSenders, channel->m_blockedReceivers, const_cast<void *>(value), timeout);
}

// See ar_kernel.h for documentation of this function.
const char * ar_channel_get_name(ar_channel_t * channel)
{
    return channel ? channel->m_name : NULL;
}

//------------------------------------------------------------------------------
// EOF
//------------------------------------------------------------------------------
