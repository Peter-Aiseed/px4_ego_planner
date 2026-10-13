"""Shared APScheduler wiring for every periodic job in the host.

Two kinds of work run on a timer: the AFBR-S50 Start keepalive (re-sends the
Start Remote Frame so the sensor keeps streaming) and the host's periodic
status block.  Both are APScheduler interval jobs rather than hand-rolled
``sleep`` loops so interval drift, misfires and shutdown are handled in one
place.

A :class:`BackgroundScheduler` runs jobs on a thread pool, so a job never
blocks the CAN reader thread or the main thread.

Usage::

    scheduler = create_scheduler()
    schedule_interval(scheduler, print_status, job_id="host-status", seconds=5)
    scheduler.start()
    ...
    scheduler.shutdown(wait=False)
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any

from apscheduler.events import EVENT_JOB_ERROR, JobExecutionEvent
from apscheduler.executors.pool import ThreadPoolExecutor
from apscheduler.jobstores.base import JobLookupError
from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.schedulers.base import BaseScheduler

logger = logging.getLogger(__name__)

#: Threads available to run jobs.  Enough for a couple of devices plus the
#: host's status job; jobs are short, so a bigger pool only adds contention.
JOB_POOL_SIZE = 4

#: A tick that fires late is still worth running (e.g. the thread was busy
#: with a slow UART write), but a very stale one is dropped so the job cannot
#: stampede after a long stall.
MISFIRE_GRACE_SEC = 5.0


def _log_job_error(event: JobExecutionEvent) -> None:
    """Log a job exception, which APScheduler would otherwise only report itself."""
    logger.error("scheduled job %r raised: %r", event.job_id, event.exception)


def create_scheduler(thread_name_prefix: str = "can_host") -> BackgroundScheduler:
    """Create a background scheduler tuned for short, frequent host jobs.

    ``coalesce`` collapses missed runs into one, ``max_instances=1`` stops a
    slow tick from stacking on top of itself, and :data:`MISFIRE_GRACE_SEC`
    keeps a slightly late tick alive without allowing a burst after a stall.

    The returned scheduler is **not started**; jobs added before ``start()``
    are held and begin running once it comes up.
    """
    scheduler = BackgroundScheduler(
        executor=ThreadPoolExecutor(JOB_POOL_SIZE),
        job_defaults={
            "coalesce": True,
            "max_instances": 1,
            "misfire_grace_time": MISFIRE_GRACE_SEC,
        },
        thread_name_prefix=thread_name_prefix,
    )
    scheduler.add_listener(_log_job_error, EVENT_JOB_ERROR)
    return scheduler


def schedule_interval(
    scheduler: BaseScheduler,
    action: Callable[[], None],
    *,
    job_id: str,
    seconds: float,
) -> Any:
    """Add (or replace) an interval job that runs *action* every *seconds*.

    The first tick happens one *seconds* from now, so a caller that needs the
    work done right away should run *action* once itself before scheduling it.

    An existing job with the same *job_id* is removed first: APScheduler's
    ``replace_existing`` only deduplicates against a running job store, and
    jobs added before ``start()`` are still pending — removing explicitly makes
    re-registering a job idempotent in both states.

    Raises :class:`ValueError` if *seconds* is not positive — a non-positive
    interval is either a permanent busy loop or a job that never runs.
    """
    if seconds <= 0:
        raise ValueError(f"job {job_id!r}: interval must be > 0, got {seconds}")
    try:
        scheduler.remove_job(job_id)
    except JobLookupError:
        pass
    return scheduler.add_job(
        action,
        "interval",
        seconds=seconds,
        id=job_id,
        replace_existing=True,
    )
