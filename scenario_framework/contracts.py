"""Appendix A event/result serialization. Writes are sequential, then atomic."""
from datetime import datetime, timezone
import json
import fcntl
import os
from pathlib import Path
import time

STATUSES = {'COMPLETED', 'TIMED_OUT', 'ERROR', 'CANCELLED'}
REASONS = {'NONE', 'EXPECTATION_FAILED', 'TIMEOUT', 'CONFIG_INVALID',
           'INFRASTRUCTURE_ERROR', 'MONITOR_DATA_MISSING',
           'SCENARIO_PRECONDITION_FAILED', 'CANCELLED', 'CLEANUP_ERROR'}
KINDS = {'state', 'path', 'position', 'obstacle', 'service_response', 'process_exit', 'trigger'}


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def outcome(status='ERROR', reason='INFRASTRUCTURE_ERROR', phase='startup', message=None):
    value = {'verdict': 'FAIL', 'execution_status': status, 'reason': reason,
             'phase': phase, 'reasons': [] if reason == 'NONE' else [{'code': reason, 'phase': phase}],
             'expectation_checks': [], 'original_outcome': None}
    if message:
        value['message'] = message
    return value


def exit_code(result):
    if result['execution_status'] == 'TIMED_OUT':
        return 124
    if result['execution_status'] == 'ERROR':
        return 2
    if result['execution_status'] == 'CANCELLED':
        return 130
    return 0 if result['verdict'] == 'PASS' else 1


class EventStream:
    """One stream, locked across helper/host even if teardown leaves a helper alive."""
    def __init__(self, directory, manifest):
        self.path = Path(directory) / 'events.jsonl'
        self.manifest = manifest
        self.seq = 0
        self.stream = self.path.open('a+b')

    def last_sequence(self):
        # Read only the tail; full path archives can make an event line large.
        end = self.stream.seek(0, os.SEEK_END)
        data = b''
        while end > 0:
            start = max(0, end - 4096)
            self.stream.seek(start)
            data = self.stream.read(end - start) + data
            lines = data.splitlines()
            if len(lines) > 1 or start == 0:
                if not data.endswith(b'\n'):
                    raise ValueError('incomplete event line; evidence retained')
                return json.loads(lines[-1])['seq']
            end = start
        return 0

    def emit(self, kind, source, payload, ros_time_s=None):
        if kind not in KINDS:
            raise ValueError('unknown event kind: ' + kind)
        fcntl.flock(self.stream, fcntl.LOCK_EX)
        try:
            self.seq = self.last_sequence() + 1
            event = {'schema_version': 1, 'run_id': self.manifest['run_id'], 'seq': self.seq,
                     'elapsed_s': max(0, time.monotonic() - self.manifest['started_monotonic_s']),
                     'ros_time_s': ros_time_s, 'source': source, 'kind': kind, 'payload': payload}
            self.stream.write((json.dumps(event, allow_nan=False) + '\n').encode())
            self.stream.flush()
            return self.seq
        finally:
            fcntl.flock(self.stream, fcntl.LOCK_UN)

    def close(self):
        self.stream.close()


class ResultStore:
    @staticmethod
    def finalize(directory, result):
        atomic_json(Path(directory) / 'result.json', result)
