/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

/**
 * @file
 * Implementation of the OS abstraction layer for the kernel device driver
 */

#ifndef _OSK_ARCH_LOCKS_H_
#define _OSK_ARCH_LOCKS_H_

#ifndef _OSK_H_
#error "Include mali_osk.h directly"
#endif

OSK_STATIC_INLINE osk_error osk_rwlock_init(osk_rwlock * const lock, int order)
{
	init_rwsem(lock);
	return OSK_ERR_NONE;
}

OSK_STATIC_INLINE void osk_rwlock_term(osk_rwlock * lock)
{
	/* nop */
}

OSK_STATIC_INLINE void osk_rwlock_read_lock(osk_rwlock * lock)
{
	down_read(lock);
}

OSK_STATIC_INLINE void osk_rwlock_read_unlock(osk_rwlock * lock)
{
	up_read(lock);
}

OSK_STATIC_INLINE void osk_rwlock_write_lock(osk_rwlock * lock)
{
	down_write(lock);
}

OSK_STATIC_INLINE void osk_rwlock_write_unlock(osk_rwlock * lock)
{
	up_write(lock);
}

OSK_STATIC_INLINE osk_error osk_mutex_init(osk_mutex * const lock, int order)
{
	mutex_init(lock);
	return OSK_ERR_NONE;
}

OSK_STATIC_INLINE void osk_mutex_term(osk_mutex * lock)
{ 
	return; /* nop */
}

OSK_STATIC_INLINE void osk_mutex_lock(osk_mutex * lock)
{
	mutex_lock(lock);
}

OSK_STATIC_INLINE void osk_mutex_unlock(osk_mutex * lock)
{
	mutex_unlock(lock);
}

OSK_STATIC_INLINE osk_error osk_spinlock_init(osk_spinlock * const lock, int order)
{
	spin_lock_init(lock);
	return OSK_ERR_NONE;
}

OSK_STATIC_INLINE void osk_spinlock_term(osk_spinlock * lock)
{
	/* nop */
}

OSK_STATIC_INLINE void osk_spinlock_lock(osk_spinlock * lock)
{
	spin_lock(lock);
}

OSK_STATIC_INLINE void osk_spinlock_unlock(osk_spinlock * lock)
{
	spin_unlock(lock);
}

OSK_STATIC_INLINE osk_error osk_spinlock_irq_init(osk_spinlock_irq * const lock, int order)
{
	spin_lock_init(&lock->lock);
	return OSK_ERR_NONE;
}

OSK_STATIC_INLINE void osk_spinlock_irq_term(osk_spinlock_irq * lock)
{
}

OSK_STATIC_INLINE void osk_spinlock_irq_lock(osk_spinlock_irq * lock)
{
	spin_lock_irqsave(&lock->lock, lock->flags);
}

OSK_STATIC_INLINE void osk_spinlock_irq_unlock(osk_spinlock_irq * lock)
{
	spin_unlock_irqrestore(&lock->lock, lock->flags);
}

#endif /* _OSK_ARCH_LOCKS_H_ */
