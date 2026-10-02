"""Bounded host-side JSONL persistence; the writer never accesses World."""
from copy import deepcopy
import json
from pathlib import Path
from queue import Empty, Full, Queue
import threading
import time


class ObservationLogWriter:
    def __init__(self, capacity=128):
        if not isinstance(capacity, int) or capacity < 1:
            raise ValueError('Observation queue capacity must be a positive integer')
        self._queue = Queue(maxsize=capacity)
        self._condition = threading.Condition()
        self._pending = self._dropped = self._failed = 0
        self._error = None
        self._closed = False
        self._thread = threading.Thread(target=self._run, name='sumo-observation-log', daemon=True)
        self._thread.start()

    def submit(self, path, records):
        batch = (Path(path), deepcopy(records))
        with self._condition:
            if self._closed:
                self._dropped += 1
                self._error = 'Observation writer is closed'
                return False
            self._pending += 1
            try:
                self._queue.put_nowait(batch)
            except Full:
                self._pending -= 1
                self._dropped += 1
                self._error = 'Observation log queue is full; complete records remain in replay memory'
                return False
        return True

    def _write(self, path, records):
        with path.open('a', encoding='utf-8') as stream:
            for record in records:
                stream.write(json.dumps(record)+'\n')

    def _run(self):
        while True:
            try:
                batch = self._queue.get(timeout=.1)
            except Empty:
                with self._condition:
                    if self._closed:
                        return
                continue
            try:
                self._write(*batch)
            except Exception as error:
                with self._condition:
                    self._failed += 1
                    self._error = str(error)
            finally:
                with self._condition:
                    self._pending -= 1
                    self._condition.notify_all()
                self._queue.task_done()

    def status(self):
        with self._condition:
            return {'pendingBatches': self._pending, 'droppedBatches': self._dropped,
                    'failedBatches': self._failed, 'error': self._error}

    def flush(self, timeout=5):
        deadline = time.monotonic()+timeout
        with self._condition:
            while self._pending:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
        return True

    def close(self, timeout=5):
        deadline = time.monotonic()+timeout
        with self._condition:
            self._closed = True
        if not self.flush(timeout):
            with self._condition:
                self._error = 'Observation log shutdown timed out; pending writes may be incomplete'
            return False
        if self._thread.is_alive():
            self._thread.join(max(0, deadline-time.monotonic()))
        return not self._thread.is_alive()
