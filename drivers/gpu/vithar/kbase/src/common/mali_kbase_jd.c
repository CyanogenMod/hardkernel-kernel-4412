/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2010-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

#include <kbase/src/common/mali_kbase.h>
#include <kbase/src/common/mali_kbase_mem.h>
#include <kbase/src/common/mali_kbase_uku.h>
#include <kbase/src/common/mali_kbase_jm.h>

#define beenthere(f, a...)  OSK_PRINT_INFO(OSK_BASE_JD, "%s:" f, __func__, ##a)

/* Structure for workaround for MIDBASE-791 */
typedef struct jd_flush_info
{
	osk_workq_work work;
	osk_waitq done_waitq;
} jd_flush_info;

/*
 * This is the kernel side of the API. Only entry points are:
 * - kbase_jd_submit(): Called from userspace to submit a single bag
 * - kbase_jd_done(): Called from interrupt context to track the
 *   completion of a job.
 * Callouts:
 * - to the job manager (enqueue a job)
 * - to the event subsystem (signals the completion/failure of bag/job-chains).
 */

static INLINE void dep_raise_sem(u32 *sem, u8 dep)
{
	if (!dep)
		return;

	sem[BASEP_JD_SEM_WORD_NR(dep)] |= BASEP_JD_SEM_MASK_IN_WORD(dep);
}

static INLINE void dep_clear_sem(u32 *sem, u8 dep)
{
	if (!dep)
		return;

	sem[BASEP_JD_SEM_WORD_NR(dep)] &= ~BASEP_JD_SEM_MASK_IN_WORD(dep);
}

static INLINE int dep_get_sem(u32 *sem, u8 dep)
{
	if (!dep)
		return 0;

	return !!(sem[BASEP_JD_SEM_WORD_NR(dep)] & BASEP_JD_SEM_MASK_IN_WORD(dep));
}

static INLINE mali_bool jd_add_dep(kbase_jd_context *ctx,
				   kbase_jd_atom *katom, u8 d)
{
	kbase_jd_dep_queue *dq = &ctx->dep_queue;
	u8 s = katom->pre_dep.dep[d];

	if (!dep_get_sem(ctx->dep_queue.sem, s))
		return MALI_FALSE;

	/*
	 * The slot must be free already. If not, then something went
	 * wrong in the validate path.
	 */
	OSK_ASSERT(!dq->queue[s]);

	dq->queue[s] = katom;
	beenthere("queued %p slot %d", (void *)katom, s);

	return MALI_TRUE;
}

/*
 * This function only computes the address of the first possible
 * atom. It doesn't mean it's actually valid (jd_validate_atom takes
 * care of that).
 */
static INLINE base_jd_atom *jd_get_first_atom(kbase_jd_context *ctx,
					      kbase_jd_bag *bag)
{
	/* Check that offset is within pool */
	if ((bag->offset + sizeof(base_jd_atom)) > ctx->pool_size)
		return NULL;

	return  (base_jd_atom *)((char *)ctx->pool + bag->offset);
}

/*
 * Same as with jd_get_first_atom, but for any subsequent atom.
 */
static INLINE base_jd_atom *jd_get_next_atom(kbase_jd_atom *katom)
{
	/* Think of adding extra padding for userspace */
	return (base_jd_atom *)base_jd_get_atom_syncset(katom->atom, katom->nr_syncsets);
}

static INLINE kbase_jd_atom *jd_validate_atom(struct kbase_context *kctx,
					      kbase_jd_bag *bag,
					      base_jd_atom *atom,
					      u32 *sem)
{
	kbase_jd_context *jctx = &kctx->jctx;
	kbase_jd_atom *katom;
	u32 nr_syncsets = atom->nr_syncsets;
	base_jd_dep pre_dep = atom->pre_dep;

	/* Check that the whole atom fits within the pool.
	 * syncsets integrity will be performed as we execute them */
	if ((char *)base_jd_get_atom_syncset(atom, nr_syncsets) > ((char *)jctx->pool + jctx->pool_size))
		return NULL;

	/*
	 * Check that dependencies are sensible: the atom cannot have
	 * pre-dependencies that are already in use by another atom.
	 */
	if (jctx->dep_queue.queue[pre_dep.dep[0]] ||
	    jctx->dep_queue.queue[pre_dep.dep[1]])
		return NULL;

	/* Check for conflicting dependencies inside the bag */
	if (dep_get_sem(sem, pre_dep.dep[0]) ||
	    dep_get_sem(sem, pre_dep.dep[1]))
		return NULL;

	dep_raise_sem(sem, pre_dep.dep[0]);
	dep_raise_sem(sem, pre_dep.dep[1]);

	/* We surely want to preallocate a pool of those, or have some
	 * kind of slab allocator around */
	katom = osk_calloc(sizeof(*katom));
	if (!katom)
		return NULL;	/* Ideally we should handle OOM more gracefully */

	katom->atom		= atom;
	katom->pre_dep		= atom->pre_dep;
	katom->post_dep		= atom->post_dep;
	katom->bag		= bag;
	katom->kctx		= kctx;
	katom->nr_syncsets	= nr_syncsets;

	/* pre-fill the event */
	katom->event.event_code	= BASE_JD_EVENT_DONE;
	katom->event.data	= katom;

	return katom;
}

static void kbase_jd_cancel_bag(kbase_context *kctx, kbase_jd_bag *bag,
				base_jd_event_code code)
{
	bag->event.event_code = code;
	kbase_event_post(kctx, &bag->event);
}

static void kbase_jd_katom_dtor(kbase_event *event)
{
	kbase_jd_atom *katom = CONTAINER_OF(event, kbase_jd_atom, event);
	osk_free(katom);
}

static mali_error kbase_jd_validate_bag(kbase_context *kctx,
					kbase_jd_bag *bag,
					osk_dlist *klistp)
{
	kbase_jd_context *jctx = &kctx->jctx;
	kbase_jd_atom *katom;
	base_jd_atom *atom;
	mali_error err = MALI_ERROR_NONE;
	u32 sem[BASEP_JD_SEM_ARRAY_SIZE] = { 0 };
	int i;

	atom = jd_get_first_atom(jctx, bag);
	if (!atom)
	{
		/* Bad start... */
		kbase_jd_cancel_bag(kctx, bag, BASE_JD_EVENT_BAG_INVALID);
		err = MALI_ERROR_FUNCTION_FAILED;
		goto out;
	}

	for (i = 0; i < bag->nr_atoms; i++)
	{
		katom = jd_validate_atom(kctx, bag, atom, sem);
		if (!katom)
		{
			OSK_DLIST_EMPTY_LIST(klistp, kbase_event,
						entry, kbase_jd_katom_dtor);
			kbase_jd_cancel_bag(kctx, bag, BASE_JD_EVENT_BAG_INVALID);
			err = MALI_ERROR_FUNCTION_FAILED;
			goto out;
		}

		OSK_DLIST_PUSH_BACK(klistp, &katom->event,
				       kbase_event, entry);
		atom = jd_get_next_atom(katom);
	}
out:
	return err;
}

static INLINE kbase_jd_atom *jd_resolve_dep(kbase_jd_atom *katom, u8 d,
					    int zapping)
{
	u8 other_dep;
	u8 dep;
	kbase_jd_atom *dep_katom;
	kbase_jd_context *ctx = &katom->kctx->jctx;

	dep = katom->post_dep.dep[d];

	if (!dep)
		return NULL;

	dep_clear_sem(ctx->dep_queue.sem, dep);

	/* Get the atom that's waiting for us (if any), and remove it
	 * from this particular dependency queue */
	dep_katom = ctx->dep_queue.queue[dep];

	/* Case of a dangling dependency */
	if (!dep_katom)
		return NULL;
	
	ctx->dep_queue.queue[dep] = NULL;

	beenthere("removed %p from slot %d",
		  (void *)dep_katom, dep);

	/* Find out if this atom is waiting for another job to be done.
	 * If it's not waiting anymore, put it on the run queue. */
	if (dep_katom->pre_dep.dep[0] == dep)
		other_dep = dep_katom->pre_dep.dep[1];
	else
		other_dep = dep_katom->pre_dep.dep[0];

	/*
	 * The following line seem to confuse people, so here's the
	 * rational behind it:
	 *
	 * The queue hold pointers to atoms waiting for a single
	 * pre-dependency to be satisfied. Above, we've already
	 * satisfied a pre-dep for an atom (dep_katom). The next step
	 * is to check whether this atom is now free to run, or has to
	 * wait for another pre-dep to be satisfied.
	 *
	 * For a single entry, 3 possibilities:
	 *
	 * - It's a pointer to dep_katom -> the pre-dep has not been
	 *   satisfied yet, and it cannot run immediately.
	 *
	 * - It's NULL -> the atom can be scheduled immediately, as
	 *   the dependency has already been satisfied.
	 *
	 * - Neither of the above: this is the case of a dependency
	 *   that has already been satisfied, and the slot reused by
	 *   an incoming atom -> dep_katom can be run immediately.
	 */
	if (ctx->dep_queue.queue[other_dep] != dep_katom)
		return dep_katom;

	/*
	 * We're on a killing spree. Cancel the additionnal
	 * dependency, and return the atom anyway. An unfortunate
	 * consequence is that userpace may receive notifications out
	 * of order WRT the dependency tree.
	 */
	if (zapping)
	{
		ctx->dep_queue.queue[other_dep] = NULL;
		return dep_katom;
	}

	beenthere("katom %p waiting for slot %d",
		  (void *)dep_katom, other_dep);
	return NULL;
}

/*
 * Perform the necessary handling of an atom that has finished running
 * on the GPU. The @a zapping parameter instruct the function to
 * propagate the state of the completed atom to all the atoms that
 * depend on it, directly or indirectly.
 *
 * This flag is used for error propagation in the "failed job", or
 * when destroying a context.
 */
static mali_bool jd_done_nolock(kbase_jd_atom *katom, int zapping)
{
	kbase_jd_atom *dep_katom;
	struct kbase_context *kctx = katom->kctx;
	osk_dlist ts;	/* traversal stack */
	osk_dlist *tsp = &ts;
	osk_dlist vl;	/* visited list */
	osk_dlist *vlp = &vl;
	kbase_jd_atom *node;
	base_jd_event_code event_code = katom->event.event_code;
	mali_bool need_to_try_schedule_context = MALI_FALSE;

	/*
	 * We're trying to achieve two goals here:
	 * - Eliminate dependency atoms very early so we can push real
         *    jobs to the HW
	 * - Avoid recursion which could result in a nice DoS from
         *    user-space.
	 *
	 * We use two lists here:
	 * - One as a stack (ts) to get rid of the recursion
	 * - The other to queue jobs that are either done or ready to
	 *   run.
	 */
	OSK_DLIST_INIT(tsp);
	OSK_DLIST_INIT(vlp);

	/* push */
	OSK_DLIST_PUSH_BACK(tsp, &katom->event, kbase_event, entry);

	while(!OSK_DLIST_IS_EMPTY(tsp))
	{
		/* pop */
		node = OSK_DLIST_POP_BACK(tsp, kbase_jd_atom, event.entry);

		if (node == katom ||
		    node->atom->core_req == BASE_JD_REQ_DEP ||
		    zapping)
		{
			int i;
			for (i = 0; i < 2; i++)
			{
				dep_katom = jd_resolve_dep(node, i, zapping);
				if (dep_katom) /* push */
					OSK_DLIST_PUSH_BACK(tsp,
							    &dep_katom->event,
							    kbase_event,
							    entry);
			}
		}

		OSK_DLIST_PUSH_BACK(vlp, &node->event,
				       kbase_event, entry);
	}

	while(!OSK_DLIST_IS_EMPTY(vlp))
	{
		node = OSK_DLIST_POP_FRONT(vlp, kbase_jd_atom, event.entry);

		if (node == katom ||
		    node->atom->core_req == BASE_JD_REQ_DEP ||
		    zapping)
		{
			kbase_jd_bag *bag = node->bag;

			/* If we're zapping stuff, propagate the event code */
			if (zapping)
			{
				/* Prevents issue raised in MIDBASE-878 */
				bag->event.event_code = event_code;
				node->event.event_code = BASE_JD_EVENT_DONE;
				/* node->event.event_code = event_code; */
			}
			/* This will signal our per-context worker
			 * thread that we're done with this katom. Any
			 * use of this katom after that point IS A
			 * ERROR!!! */
			kbase_event_post(kctx, &node->event);
			beenthere("done atom %p\n", (void*)node);

			if (--bag->nr_atoms == 0)
			{
				/* This atom was the last, signal userspace */
				kbase_event_post(kctx, &bag->event);
				beenthere("done bag %p\n", (void*)bag);
			}

			/* Decrement and check the TOTAL number of jobs. This includes
			 * those not tracked by the scheduler: 'not ready to run' and
			 * 'dependency-only' jobs. */
			if (--kctx->jctx.job_nr == 0)
			{
				/* All objects are safely queued now, and we
				 * can mark the context as idle if
				 * necessary */
				osk_waitq_set(&kctx->jctx.idle_waitq);
			}
		}
		else
		{
			/* Queue an action about whether we should try scheduling a context */
			need_to_try_schedule_context |= kbasep_js_add_job( kctx, node );
		}
	}

	return need_to_try_schedule_context;
}

mali_error kbase_jd_submit(kbase_context *kctx, const kbase_uk_job_submit *user_bag)
{
	osk_dlist klist;
	osk_dlist *klistp = &klist;
	kbase_jd_context *jctx = &kctx->jctx;
	kbase_jd_atom *katom;
	kbase_jd_bag *bag;
	mali_error err = MALI_ERROR_NONE;
	int i = -1;
	mali_bool need_to_try_schedule_context = MALI_FALSE;
	kbase_device *kbdev;

	kbdev = kctx->kbdev;

	beenthere("%s", "Enter");
	bag = osk_malloc(sizeof(*bag));
	if (NULL == bag)
	{
		err = MALI_ERROR_OUT_OF_MEMORY;
		goto out_bag;
	}

	bag->core_restriction	= user_bag->core_restriction;
	bag->offset		= user_bag->offset;
	bag->nr_atoms		= user_bag->nr_atoms;
	bag->event.event_code	= BASE_JD_EVENT_BAG_DONE;
	bag->event.data		= (void *)(uintptr_t)user_bag->bag_uaddr;

	osk_mutex_lock(&jctx->lock);

	/*
	 * Use a temporary list to store all the validated atoms.
	 * Once we're sure nothing is wrong, there's no going back.
	 */
	OSK_DLIST_INIT(klistp);

	if (kbase_jd_validate_bag(kctx, bag, klistp))
	{
		err = MALI_ERROR_FUNCTION_FAILED;
		goto out;
	}

	while(!OSK_DLIST_IS_EMPTY(klistp))
	{

		katom = OSK_DLIST_POP_FRONT(klistp,
					    kbase_jd_atom, event.entry);
		i++;

		/* This is crucial. As jobs are processed in-order, we must
		 * indicate that any job with a pre-dep on this particular job
		 * must wait for its completion (indicated as a post-dep).
		 */
		dep_raise_sem(jctx->dep_queue.sem, katom->post_dep.dep[0]);
		dep_raise_sem(jctx->dep_queue.sem, katom->post_dep.dep[1]);

		/* Process pre-exec syncsets before queueing */
		kbase_pre_job_sync(kctx,
				   base_jd_get_atom_syncset(katom->atom, 0),
				   katom->nr_syncsets);

		/* Update the TOTAL number of jobs. This includes those not tracked by
		 * the scheduler: 'not ready to run' and 'dependency-only' jobs. */
		jctx->job_nr++;
		osk_waitq_clear(&jctx->idle_waitq);

		/* If no pre-dep has been set, then we're free to run
		 * the job immediately */
		if ((jd_add_dep(jctx, katom, 0) | jd_add_dep(jctx, katom, 1)))
		{
			beenthere("queuing atom #%d(%p %p)", i,
				  (void *)katom, (void *)katom->atom);
			continue;
		}

		beenthere("running atom #%d(%p %p)", i,
			  (void *)katom, (void *)katom->atom);

		/* Lock the JS Context, for submitting jobs, and queue an action about
		 * whether we need to try scheduling the context */
		osk_mutex_lock( &kctx->jctx.sched_info.ctx.jsctx_lock );
		if (katom->atom->core_req != BASE_JD_REQ_DEP)
		{
			need_to_try_schedule_context |= kbasep_js_add_job( kctx, katom );
		}
		else
		{
			/* This is a pure dependency. Resolve it immediately */
			need_to_try_schedule_context |= jd_done_nolock(katom, 0);
		}
		osk_mutex_unlock( &kctx->jctx.sched_info.ctx.jsctx_lock );

	}

	/* Only whilst we've dropped the JS context lock can we schedule a new
	 * context.
	 *
	 * As an optimization, we only need to do this after processing all jobs
	 * resolved from this context. */
	if ( need_to_try_schedule_context != MALI_FALSE )
	{
		kbasep_js_try_schedule_head_ctx( kbdev );
	}

out:
	osk_mutex_unlock(&jctx->lock);
out_bag:
	beenthere("%s", "Exit");
	return err;
}

static void jd_done_worker(osk_workq_work *data)
{
	kbase_jd_atom *katom = CONTAINER_OF(data, kbase_jd_atom, work);
	kbase_jd_context *jctx;
	kbase_context *kctx;
	kbasep_js_kctx_info *js_kctx_info;
	kbasep_js_policy *js_policy;
	kbase_device *kbdev;
	kbasep_js_device_data *js_devdata;
	int zapping;

	mali_bool need_to_try_schedule_context = MALI_FALSE;
	mali_bool retry_submit;
	int retry_jobslot;

	kctx = katom->kctx;
	jctx = &kctx->jctx;
	kbdev = kctx->kbdev;
	js_kctx_info = &kctx->jctx.sched_info;

	js_devdata = &kbdev->js_data;
	js_policy = &kbdev->js_data.policy;

	/* Atomically perform the following:
	 * - Update the runpool atomically
	 * - Resolve dependencies (potentially enqueuing more jobs)
	 * - Schedule out the context if appropriate
	 *
	 * NOTE : Should separate out scheduler stuff */
	osk_mutex_lock( &jctx->lock );
	osk_mutex_lock( &js_kctx_info->ctx.jsctx_lock );

	/* Atomically perform the following:
	 * - Update running jobs
	 * - Requeue a soft-stopped job
	 * - Remove jobs from the system
	 *
	 * This can leave the Run Pool in a state where it has contexts scheduled
	 * in, but no jobs at all (running or ready to run)
	 */
	osk_spinlock_irq_lock( &js_devdata->runpool_lock );

	/* This only gets called on contexts that are scheduled *in*. This is
	 * because it only happens in response to an IRQ from a job that was
	 * running.
	 */
	OSK_ASSERT( js_kctx_info->ctx.is_scheduled != MALI_FALSE );

	OSK_ASSERT( js_kctx_info->runpool.nr_running_jobs > 0 );
	--(js_kctx_info->runpool.nr_running_jobs);

	/* Grab the retry_submit state before the katom disappears */
	retry_submit = kbasep_js_get_job_retry_submit_slot( katom, &retry_jobslot );

	if (katom->event.event_code == BASE_JD_EVENT_STOPPED)
	{
		OSK_PRINT_INFO(OSK_BASE_JM, "JS: Soft Stopped %p on Ctx %p; Requeuing", kctx );
		/* Only requeue the atom on soft-stop */
		kbasep_js_clear_job_retry_submit( katom );
		kbasep_js_policy_requeue_job( js_policy, katom );

		/* Drop the Run Pool lock for now */
		osk_spinlock_irq_unlock( &js_devdata->runpool_lock );
	}
	else if (katom->event.event_code == BASE_JD_EVENT_REMOVED_FROM_NEXT)
	{
		kbasep_js_policy_requeue_job( js_policy, katom );

		/* Drop the Run Pool lock for now */
		osk_spinlock_irq_unlock( &js_devdata->runpool_lock );
	}
	else
	{
		/* Remove the job from the system */
		kbasep_js_remove_job_nolock( kctx, katom );

		/* Drop the Run Pool lock for now - the completed job is not present in it, but
		 * we still need to resolve its dependencies outside of the lock */
		osk_spinlock_irq_unlock( &js_devdata->runpool_lock );

		zapping = (katom->event.event_code != BASE_JD_EVENT_DONE);

		/* Queue an action for whether to try scheduling in a new context */
		need_to_try_schedule_context |= jd_done_nolock(katom, zapping);
		/* katom may have been freed now, do not use! */
	}
	/* Run Pool lock was dropped by this point */


	/* This checks whether the context should be descheduled, and takes a lot
	 * of other action if descheduling.
	 *
	 * We queue an action to occur later too: whether to try scheduling in a new context. */
	need_to_try_schedule_context |= kbasep_js_check_and_deschedule_ctx( kbdev, kctx );

	/* We've finished with this context for now, so drop the lock for it. */
	osk_mutex_unlock( &js_kctx_info->ctx.jsctx_lock );
	/* We've finished any actions on the JD context too */
	osk_mutex_unlock( &jctx->lock );


	/* Have we got any actions queued up from when we held the context lock? */
	if ( need_to_try_schedule_context != MALI_FALSE )
	{
		/* We've freed up an address space, so let's try to schedule in another
		 * context
		 *
		 * Note: if there's a context to schedule in, then it also tries to run
		 * another job, in case the new context has jobs satisfying requirements
		 * that no other context/job in the runpool does */
		kbasep_js_try_schedule_head_ctx( kbdev );
	}

	/* If the IRQ handler failed to get a job from the policy, try again from
	 * outside the IRQ handler */
	if ( retry_submit != MALI_FALSE )
	{
		kbasep_js_try_run_next_job_on_slot( kbdev, retry_jobslot );
	}
}

/*
 * Work queue job cancel function
 * Only called as part of 'Zapping' a context (which occurs on termination)
 * Operates serially with the jd_done_worker() on the work queue
 */
static void jd_cancel_worker(osk_workq_work *data)
{
	kbase_jd_atom *katom = CONTAINER_OF(data, kbase_jd_atom, work);
	kbase_jd_context *jctx;
	kbase_context *kctx;
	kbasep_js_kctx_info *js_kctx_info;
	kbasep_js_policy *js_policy;
	kbase_device *kbdev;
	int zapping;

	kctx = katom->kctx;
	jctx = &kctx->jctx;
	kbdev = kctx->kbdev;
	js_kctx_info = &kctx->jctx.sched_info;

	js_policy = &kbdev->js_data.policy;

	/* This only gets called on contexts that are scheduled out. Hence, we must
	 * make sure we don't de-ref the number of running* jobs (there aren't
	 * any), nor must we try to schedule out the context (it's already
	 * scheduled out).
	 */
	OSK_ASSERT( js_kctx_info->ctx.is_scheduled == MALI_FALSE );

	/* Scheduler: Remove the job from the system */
	kbasep_js_remove_job_nolock( kctx, katom );

	osk_mutex_lock(&jctx->lock);
	zapping = (katom->event.event_code != BASE_JD_EVENT_DONE);
	jd_done_nolock(katom, zapping);
	/* katom may have been freed now, do not use! */
	osk_mutex_unlock(&jctx->lock);
}


void kbase_jd_done(kbase_jd_atom *katom)
{
	osk_workq_submit(&katom->kctx->jctx.job_done_wq,
			jd_done_worker, &katom->work);
}

void kbase_jd_cancel(kbase_jd_atom *katom)
{
	kbase_context *kctx;
	kbasep_js_kctx_info *js_kctx_info;

	kctx = katom->kctx;
	js_kctx_info = &kctx->jctx.sched_info;

	/* This should only be done from a context that is not scheduled */
	OSK_ASSERT( js_kctx_info->ctx.is_scheduled == MALI_FALSE );
	OSK_ASSERT( js_kctx_info->runpool.nr_running_jobs == 0 );

	katom->event.event_code = BASE_JD_EVENT_JOB_CANCELLED;

	osk_workq_submit(&kctx->jctx.job_done_wq,
			jd_cancel_worker, &katom->work);
}

/* Workaround for MIDBASE-791 */
static void jd_flush_worker(osk_workq_work *data)
{
	jd_flush_info *flusher = CONTAINER_OF(data, jd_flush_info, work);
	/* Just signal the attached wait-queue, to make use of the FIFO ordering */
	osk_waitq_set(&flusher->done_waitq);
}

void kbase_jd_flush_workqueues(kbase_context *kctx)
{
	jd_flush_info flusher;
	osk_error osk_err;
	kbase_device *kbdev;
	int i;

	OSK_ASSERT( kctx );

	kbdev = kctx->kbdev;
	OSK_ASSERT( kbdev );

	osk_err = osk_waitq_init(&flusher.done_waitq);
	if (OSK_ERR_NONE != osk_err)
	{
		OSK_PRINT_ERROR(OSK_BASE_JD, "Failed to initialize a wait queue for the job dispatch flusher!\n");
		return;
	}

	osk_waitq_clear(&flusher.done_waitq);
	osk_workq_submit( &kctx->jctx.job_done_wq, jd_flush_worker, &flusher.work );
	osk_waitq_wait( &flusher.done_waitq );

	/* Flush all workqueues, for simplicity */
	for (i = 0; i < kbdev->nr_address_spaces; i++)
	{
		osk_waitq_clear(&flusher.done_waitq);
		osk_workq_submit(&kbdev->as[i].pf_wq, jd_flush_worker, &flusher.work);
		osk_waitq_wait( &flusher.done_waitq );
	}

	osk_waitq_term( &flusher.done_waitq );
}

typedef struct zap_reset_data
{
	/* The stages are:
	 * 1. The timer has never been called
	 * 2. The zap has timed out, all slots are soft-stopped - the GPU reset will happen
	 * 3. The GPU has been reset
	 */
	int stage;
	kbase_device *kbdev;
	osk_timer *timer;
	osk_waitq waitq;
} zap_reset_data;

static void zap_timeout_worker(osk_workq_work *data)
{
	kbase_device *kbdev = CONTAINER_OF(data, kbase_device, reset_work);
	int i;
	kbasep_js_tick end_timestamp = kbasep_js_get_js_ticks();

	/* All slot have been soft-stopped and we've waited ZAP_TIMEOUT for the slots to clear, at this point we assume
	 * that anything that is still left on the GPU is stuck there and we'll kill it when we reset the GPU */

	OSK_PRINT_ERROR(OSK_BASE_JD, "Resetting GPU");

	/* Reset the GPU */
	kbase_pm_power_transitioning(kbdev);
	kbase_pm_init_hw(kbdev);

	/* Pretend there is a context active - this is to satisify the precondition of KBASE_PM_EVENT_INIT */
	kbase_pm_context_active(kbdev);

	/* Re-init the power policy */
	kbase_pm_send_event(kbdev, KBASE_PM_EVENT_INIT);

	/* Idle the fake context */
	kbase_pm_context_idle(kbdev);

	/* Complete any jobs that were still on the GPU */
	for (i = 0; i < kbdev->nr_job_slots; i++)
	{
		int nr_done;
		kbase_jm_slot *slot = &kbdev->jm_slots[i];

		osk_spinlock_irq_lock(&slot->lock);

		nr_done = kbasep_jm_nr_jobs_submitted( slot );
		while (nr_done) {
			kbase_job_done_slot(kbdev, i, BASE_JD_EVENT_JOB_CANCELLED, 0, &end_timestamp);
			nr_done--;
		}
#ifdef BASE_HW_ISSUE_5713
		/* We've reset the GPU so the slot is no longer soft-stopped */
		slot->submission_blocked_for_soft_stop = MALI_FALSE;
#endif

		osk_spinlock_irq_unlock(&slot->lock);
	}

	osk_atomic_set(&kbdev->reset_gpu, 0);
	
	OSK_PRINT_ERROR(OSK_BASE_JD, "Reset complete");
	
	/* Try submitting some jobs to restart processing */
	for (i = 0; i < kbdev->nr_job_slots; i++)
	{
		kbasep_js_try_run_next_job_on_slot(kbdev, i);
	}
}

static void zap_timeout_callback(void *data)
{
	zap_reset_data *reset_data = (zap_reset_data*)data;
	kbase_device *kbdev = reset_data->kbdev;
	int i;
	osk_error ret;
	osk_timer *timer = reset_data->timer;

	if (reset_data->stage == 2)
	{
		osk_workq_submit(&kbdev->reset_workq, zap_timeout_worker, &kbdev->reset_work);

		reset_data->stage = 3;

		osk_waitq_set(&reset_data->waitq);
		return;
	}

	if (osk_atomic_compare_and_swap(&kbdev->reset_gpu, 0, 1) != 0)
	{
		/* Some other thread is already resetting the GPU */
		osk_waitq_set(&reset_data->waitq);
		return;
	}

	reset_data->stage = 2;

	OSK_PRINT_ERROR(OSK_BASE_JD, "Context failed to idle: preparing to soft-reset GPU");

	for (i = 0; i < kbdev->nr_job_slots; i++)
	{
		kbase_jm_slot *slot = &kbdev->jm_slots[i];
		osk_spinlock_irq_lock(&slot->lock);
		kbase_job_slot_softstop(kbdev, i);
		osk_spinlock_irq_unlock(&slot->lock);
	}

	ret = osk_timer_start(timer, ZAP_TIMEOUT);
	if (ret != OSK_ERR_NONE)
	{
		OSK_PRINT_ERROR(OSK_BASE_JD, "Failed to start timer for soft-resetting GPU");
	}
}

void kbase_jd_zap_context(kbase_context *kctx)
{
	kbase_device *kbdev = kctx->kbdev;
	osk_timer zap_timeout;
	osk_error ret;
	zap_reset_data reset_data;

	kbase_job_zap_context(kctx);
#ifdef BASE_HW_ISSUE_6035
	osk_spinlock_irq_lock(&kbdev->js_data.runpool_lock);
	if (kctx->as_nr >= 0) {
		/* AS transaction begin */
		osk_spinlock_irq_lock(&kbdev->as[kctx->as_nr].transaction_lock);
		/* We need to map in a special set of MMU tables to work around a hardware issue */
		kbase_reg_write(kbdev, MMU_AS_REG(kctx->as_nr, ASn_TRANSTAB_LO), kbdev->mmu_fault_pages[0], kctx);
		kbase_reg_write(kbdev, MMU_AS_REG(kctx->as_nr, ASn_TRANSTAB_HI), 0, kctx);

		kbase_reg_write(kbdev, MMU_AS_REG(kctx->as_nr, ASn_COMMAND), 1, kctx);
		osk_spinlock_irq_unlock(&kbdev->as[kctx->as_nr].transaction_lock);
		/* AS transaction end */
	}
	osk_spinlock_irq_unlock(&kbdev->js_data.runpool_lock);
#endif
	ret = osk_timer_init(&zap_timeout);
	if (ret != OSK_ERR_NONE)
	{
		goto skip_timeout;
	}
	ret = osk_waitq_init(&reset_data.waitq);
	if (ret != OSK_ERR_NONE)
	{
		osk_timer_term(&zap_timeout);
		goto skip_timeout;
	}
	reset_data.kbdev = kbdev;
	reset_data.timer = &zap_timeout;
	reset_data.stage = 1;
	osk_timer_callback_set(&zap_timeout, zap_timeout_callback, &reset_data);
	ret = osk_timer_start(&zap_timeout, ZAP_TIMEOUT);

	if (ret != OSK_ERR_NONE)
	{
		osk_timer_term(&zap_timeout);
		osk_waitq_term(&reset_data.waitq);
		goto skip_timeout;
	}

	/* If we jump to here then the zap timeout will not be active,
	 * so if the GPU hangs the driver will also hang. This will only
	 * happen if the driver is very resource starved.
	 */
skip_timeout:
	osk_waitq_wait(&kctx->jctx.idle_waitq);

	if (ret == OSK_ERR_NONE)
	{
		osk_timer_stop(&zap_timeout);
		if (reset_data.stage == 2)
		{
			/* Whoops - we stopped the timer in the middle of the GPU reset sequence, we restart the timer and
			 * continue with the GPU reset. And this time wait for the reset to complete */
			ret = osk_timer_start(&zap_timeout, ZAP_TIMEOUT);
			if (ret != OSK_ERR_NONE)
			{
				/* We don't have enough resources to re-enable the timer. This shouldn't happen, the best we
				 * can do is to re-enable submission and hope that everything turns out alright */
				OSK_PRINT_WARN(OSK_BASE_JD, "Zap: Failed to restart timer for soft-resetting GPU");
				osk_atomic_set(&kbdev->reset_gpu, 0);
			}
			else
			{
				osk_waitq_wait(&reset_data.waitq);
			}
		}
		osk_timer_term(&zap_timeout);
		osk_waitq_term(&reset_data.waitq);
	}

	OSK_PRINT_INFO(OSK_BASE_JM, "Zap: Finished Context %p", kctx );

	/* Synchronize with jd_done_worker!!! */
	osk_mutex_lock(&kctx->jctx.lock);
	osk_mutex_unlock(&kctx->jctx.lock);
}

mali_error kbase_jd_init(struct kbase_context *kctx)
{
	void *kaddr;
	int i;
	mali_error mali_err;
	osk_error osk_err;

	kaddr = osk_vmalloc(BASEP_JCTX_RB_NRPAGES * OSK_PAGE_SIZE);
	if (!kaddr)
	{
		mali_err = MALI_ERROR_OUT_OF_MEMORY;
		goto out;
	}
	osk_err = osk_workq_init(&kctx->jctx.job_done_wq, "mali_jd", 0);
	if (OSK_ERR_NONE != osk_err)
	{
		mali_err = MALI_ERROR_OUT_OF_MEMORY;
		goto out1;
	}

	for (i = 0; i < 256; i++)
		kctx->jctx.dep_queue.queue[i] = NULL;

	for (i = 0; i < BASEP_JD_SEM_ARRAY_SIZE; i++)
		kctx->jctx.dep_queue.sem[i] = 0;

	osk_err = osk_mutex_init(&kctx->jctx.lock, OSK_LOCK_ORDER_JCTX);
	if (OSK_ERR_NONE != osk_err)
	{
		mali_err = MALI_ERROR_FUNCTION_FAILED;
		goto out2;
	}

	osk_err = osk_waitq_init(&kctx->jctx.idle_waitq);
	if (OSK_ERR_NONE != osk_err)
	{
		mali_err = MALI_ERROR_FUNCTION_FAILED;
		goto out3;
	}

	osk_err = osk_spinlock_irq_init(&kctx->jctx.tb_lock, OSK_LOCK_ORDER_TB);
	if (OSK_ERR_NONE != osk_err)
	{
		mali_err = MALI_ERROR_FUNCTION_FAILED;
		goto out4;
	}

	osk_waitq_set(&kctx->jctx.idle_waitq);

	kctx->jctx.pool		= kaddr;
	kctx->jctx.pool_size	= BASEP_JCTX_RB_NRPAGES * OSK_PAGE_SIZE;
	kctx->jctx.job_nr	= 0;

	return MALI_ERROR_NONE;

out4:
	osk_waitq_term(&kctx->jctx.idle_waitq);
out3:
	osk_mutex_term(&kctx->jctx.lock);
out2:
	osk_workq_term(&kctx->jctx.job_done_wq);
out1:
	osk_vfree(kaddr);
out:
	return mali_err;
}

void kbase_jd_exit(struct kbase_context *kctx)
{
	osk_mutex_lock(&kctx->jctx.lock);
	osk_spinlock_irq_term(&kctx->jctx.tb_lock);
	osk_workq_term(&kctx->jctx.job_done_wq);
	osk_vfree(kctx->jctx.pool);
	osk_mutex_unlock(&kctx->jctx.lock);
	osk_mutex_term(&kctx->jctx.lock);
}
