"""Phase 3 테스트 — FileIpcWorker 베이스.

구체 테스트 워커(EchoWorker)로 명령 디스패치/상태/staleness/생명주기를 검증.
"""

import time

from pickline.picking import FileIpcWorker
from pickline.infra.file_ipc import atomic_write_json, read_json


class EchoWorker(FileIpcWorker):
    """테스트용 구체 워커 — 사이클은 실행 기록만 남긴다."""
    WORKER_NAME = "echo_worker"
    CYCLE_COMMANDS = {"reject_cycle", "test_cycle"}

    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.setup_called = False
        self.executed = []          # 처리한 seq 기록
        self.estopped = []
        self.fail_next = False

    def setup(self):
        self.setup_called = True

    def execute_cycle(self, command):
        if self.fail_next:
            raise RuntimeError("의도된 실패")
        self.executed.append(int(command["seq"]))

    def on_emergency_stop(self, command):
        self.estopped.append(int(command["seq"]))


def make(tmp_path):
    return EchoWorker("r1", tmp_path / "ipc",
                      command_file="cmd.json", status_file="st.json")


def cmd(seq, command="reject_cycle", **extra):
    d = {"seq": seq, "command": command, "schema": "intel5.robot_command.v1"}
    d.update(extra)
    return d


# --- 상태 ---

def test_write_status(tmp_path):
    w = make(tmp_path)
    w.write_status("READY", "waiting")
    st = read_json(w.status_path)
    assert st["state"] == "READY" and st["channel_id"] == "r1"
    assert st["worker"] == "echo_worker"


# --- 명령 디스패치 ---

def test_cycle_command_executes(tmp_path):
    w = make(tmp_path)
    w.handle_command(cmd(1))
    assert w.executed == [1]
    assert w.last_processed_seq == 1
    assert read_json(w.status_path)["state"] == "DONE"

def test_duplicate_seq_ignored(tmp_path):
    w = make(tmp_path)
    w.handle_command(cmd(1))
    w.handle_command(cmd(1))      # 같은 seq — 무시
    assert w.executed == [1]

def test_unknown_command_ignored(tmp_path):
    w = make(tmp_path)
    w.handle_command(cmd(1, command="dance"))
    assert w.executed == []
    assert read_json(w.status_path)["state"] == "IGNORED"

def test_emergency_stop(tmp_path):
    w = make(tmp_path)
    w.handle_command(cmd(1, command="emergency_stop"))
    assert w.estopped == [1]
    assert read_json(w.status_path)["state"] == "STOPPED"

def test_stale_command_ignored(tmp_path):
    w = make(tmp_path)
    old = cmd(1, created_at=time.time() - 999, command_max_age_s=5)
    w.handle_command(old)
    assert w.executed == []
    assert read_json(w.status_path)["state"] == "IGNORED"

def test_expired_command_ignored(tmp_path):
    w = make(tmp_path)
    w.handle_command(cmd(1, expires_at=time.time() - 10))
    assert w.executed == []

def test_execute_cycle_failure_marks_error(tmp_path):
    w = make(tmp_path)
    w.fail_next = True
    w.handle_command(cmd(1))
    st = read_json(w.status_path)
    assert st["state"] == "ERROR"
    assert w.last_processed_seq == 1     # 실패해도 seq는 소비 (재실행 방지)


# --- 생명주기 ---

def test_loop_runs_and_stops(tmp_path):
    w = make(tmp_path)
    w.poll_s = 0.02
    w.start_background()
    time.sleep(0.1)
    assert w.setup_called is True
    # 파일로 명령 투입
    atomic_write_json(w.command_path, cmd(5))
    time.sleep(0.1)
    w.request_stop()
    time.sleep(0.1)
    assert 5 in w.executed
    assert read_json(w.status_path)["state"] == "STOPPED"
