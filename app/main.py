from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from uuid import uuid4

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

ROOT = Path(__file__).resolve().parents[1]
STATIC_DIR = ROOT / "app" / "static"
SIM_DIR = ROOT / "sim_engine"
RUNS_DIR = Path(
    os.getenv("PORTFOLIO_DRONE_RUNS_DIR", str(SIM_DIR / "runs"))
)
BINARY_PATH = SIM_DIR / "batch.bin"
JOB_TTL_SECONDS = 30 * 60

app = FastAPI(title="Drone Replay")
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


class ConditionsPayload(BaseModel):
    target_1_z: float | None = None


@dataclass
class JobState:
    job_id: str
    run_dir: Path
    conditions_path: Path
    telemetry_path: Path
    shape_path: Path
    meta_path: Path
    process: subprocess.Popen[str] | None = None
    started_at: float | None = None
    last_accessed_at: float = 0.0


JOBS: dict[str, JobState] = {}


@app.get("/")
def index(request: Request) -> RedirectResponse:
    return RedirectResponse(url=app_url(request, "/sim/batch"), status_code=307)


@app.get("/sim/batch")
def sim_batch() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.post("/api/jobs")
def create_job() -> dict[str, Any]:
    cleanup_expired_jobs()
    job = create_job_state()
    return job_payload(job)


@app.get("/api/jobs/{job_id}/telemetry")
def telemetry(job_id: str) -> list[dict[str, Any]]:
    job = get_job(job_id)
    return read_jsonl(job.telemetry_path)


@app.get("/api/jobs/{job_id}/shape")
def shape(job_id: str) -> dict[str, Any]:
    job = get_job(job_id)
    if not job.shape_path.exists():
        return {}
    try:
        return json.loads(job.shape_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise HTTPException(status_code=500, detail=f"Invalid shape JSON: {exc}") from exc


@app.get("/api/jobs/{job_id}/conditions")
def get_conditions(job_id: str) -> dict[str, Any]:
    job = get_job(job_id)
    return read_conditions(job)


@app.put("/api/jobs/{job_id}/conditions")
def put_conditions(job_id: str, payload: ConditionsPayload) -> dict[str, Any]:
    job = get_job(job_id)
    merged = read_conditions(job)
    merged.update(payload.model_dump(exclude_unset=True))
    sanitize_conditions(merged)
    write_conditions(job, merged)
    return read_conditions(job)


@app.get("/api/jobs/{job_id}/simulation/status")
def simulation_status(job_id: str) -> dict[str, object]:
    job = get_job(job_id)
    process = current_process(job)
    return {
        "job_id": job.job_id,
        "running": process is not None,
        "pid": process.pid if process else None,
        "started_at": job.started_at if process else None,
        "conditions": read_conditions(job),
        "telemetry_rows": count_jsonl_rows(job.telemetry_path),
    }


@app.post("/api/jobs/{job_id}/simulation/start")
def start_simulation(job_id: str) -> dict[str, object]:
    cleanup_expired_jobs()
    job = get_job(job_id)
    stop_existing_process(job)
    truncate_file(job.telemetry_path)
    truncate_file(job.shape_path)

    build_binary()

    conditions = read_conditions(job)
    conditions["kill"] = 0
    conditions["paused"] = 0
    write_conditions(job, conditions)

    job.process = subprocess.Popen(
        [
            str(BINARY_PATH),
            str(job.conditions_path),
            str(job.telemetry_path),
            str(job.shape_path),
        ],
        cwd=SIM_DIR,
        text=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    job.started_at = time.time()
    touch_job(job)

    return {
        "ok": True,
        "job_id": job.job_id,
        "running": True,
        "pid": job.process.pid,
        "started_at": job.started_at,
        "conditions": read_conditions(job),
        "telemetry_rows": 0,
    }


@app.post("/api/jobs/{job_id}/simulation/stop")
def stop_simulation(job_id: str) -> dict[str, object]:
    job = get_job(job_id)
    conditions = read_conditions(job)
    conditions["kill"] = 1
    write_conditions(job, conditions)
    stop_existing_process(job)
    touch_job(job)
    return {"ok": True, "job_id": job.job_id, "running": False, "pid": None}


@app.post("/api/run-simulation")
def run_simulation() -> dict[str, object]:
    job = create_job_state()
    return start_simulation(job.job_id)


def default_conditions() -> dict[str, Any]:
    return {
        "kill": 0,
        "paused": 0,
        "nav_mode": "gps_hold",
        "target_1_z": 0.0,
        "target_yaw": 0.0,
    }


def sanitize_conditions(payload: dict[str, Any]) -> dict[str, Any]:
    payload.pop("runtime", None)
    payload.pop("target_1_x", None)
    payload.pop("target_1_y", None)
    payload.pop("wind_x", None)
    payload.pop("wind_y", None)
    payload.pop("wind_z", None)
    return payload


def create_job_state() -> JobState:
    while True:
        job_id = uuid4().hex
        run_dir = RUNS_DIR / job_id
        if not run_dir.exists():
            break

    job = JobState(
        job_id=job_id,
        run_dir=run_dir,
        conditions_path=run_dir / "conditions.json",
        telemetry_path=run_dir / "telemetry.jsonl",
        shape_path=run_dir / "shape.json",
        meta_path=run_dir / "meta.json",
        last_accessed_at=time.time(),
    )
    run_dir.mkdir(parents=True, exist_ok=True)
    write_conditions(job, default_conditions())
    touch_job(job)
    JOBS[job_id] = job
    return job


def load_job(job_id: str) -> JobState | None:
    if not job_id or any(char not in "0123456789abcdef" for char in job_id):
        return None

    job = JOBS.get(job_id)
    if job is not None:
        touch_job(job)
        return job

    run_dir = RUNS_DIR / job_id
    if not run_dir.exists() or not run_dir.is_dir():
        return None

    job = JobState(
        job_id=job_id,
        run_dir=run_dir,
        conditions_path=run_dir / "conditions.json",
        telemetry_path=run_dir / "telemetry.jsonl",
        shape_path=run_dir / "shape.json",
        meta_path=run_dir / "meta.json",
        last_accessed_at=read_job_last_access(run_dir / "meta.json"),
    )
    JOBS[job_id] = job
    touch_job(job)
    return job


def get_job(job_id: str) -> JobState:
    cleanup_expired_jobs()
    job = load_job(job_id)
    if job is None:
        raise HTTPException(status_code=404, detail="Job not found")
    return job


def read_conditions(job: JobState) -> dict[str, Any]:
    ensure_conditions_file(job)
    defaults = default_conditions()
    try:
        current = json.loads(job.conditions_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        write_conditions(job, defaults)
        return dict(defaults)

    if not isinstance(current, dict):
        write_conditions(job, defaults)
        return dict(defaults)

    merged = dict(defaults)
    merged.update(current)
    sanitize_conditions(merged)
    touch_job(job)
    return merged


def write_conditions(job: JobState, payload: dict[str, Any]) -> None:
    job.run_dir.mkdir(parents=True, exist_ok=True)
    sanitize_conditions(payload)
    temp_path = job.conditions_path.with_suffix(job.conditions_path.suffix + ".tmp")
    temp_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    temp_path.replace(job.conditions_path)
    touch_job(job)


def ensure_conditions_file(job: JobState) -> None:
    if job.conditions_path.exists():
        return
    write_conditions(job, default_conditions())


def job_payload(job: JobState) -> dict[str, Any]:
    process = current_process(job)
    return {
        "job_id": job.job_id,
        "running": process is not None,
        "pid": process.pid if process else None,
        "started_at": job.started_at if process else None,
        "conditions": read_conditions(job),
        "telemetry_rows": count_jsonl_rows(job.telemetry_path),
    }


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []

    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                rows.append(json.loads(stripped))
            except json.JSONDecodeError as exc:
                raise HTTPException(
                    status_code=500,
                    detail=f"Invalid telemetry JSONL at line {line_number}: {exc}",
                ) from exc
    return rows


def current_process(job: JobState) -> subprocess.Popen[str] | None:
    if job.process is None:
        return None
    if job.process.poll() is not None:
        job.process = None
        job.started_at = None
        return None
    return job.process


def stop_existing_process(job: JobState) -> None:
    process = current_process(job)
    if process is None:
        return
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3)
    job.process = None
    job.started_at = None


def build_binary() -> None:
    if not BINARY_PATH.is_file() or not os.access(BINARY_PATH, os.X_OK):
        raise HTTPException(
            status_code=503,
            detail="Drone simulator binary is missing; rebuild it during deployment.",
        )


def count_jsonl_rows(path: Path) -> int:
    if not path.exists():
        return 0
    with path.open("r", encoding="utf-8") as file:
        return sum(1 for line in file if line.strip())


def truncate_file(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("", encoding="utf-8")


def touch_job(job: JobState) -> None:
    job.last_accessed_at = time.time()
    job.run_dir.mkdir(parents=True, exist_ok=True)
    job.meta_path.write_text(
        json.dumps({"last_accessed_at": job.last_accessed_at}, indent=2) + "\n",
        encoding="utf-8",
    )


def read_job_last_access(meta_path: Path) -> float:
    if not meta_path.exists():
        return time.time()
    try:
        payload = json.loads(meta_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return time.time()
    if not isinstance(payload, dict):
        return time.time()
    value = payload.get("last_accessed_at")
    return float(value) if isinstance(value, (int, float)) else time.time()


def cleanup_expired_jobs() -> None:
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    now = time.time()
    known_job_ids = set(JOBS)

    for job_id in list(known_job_ids):
        job = JOBS.get(job_id)
        if job is None:
            continue
        current_process(job)
        if now - job.last_accessed_at <= JOB_TTL_SECONDS:
            continue
        stop_existing_process(job)
        delete_job(job)

    for run_dir in RUNS_DIR.iterdir():
        if not run_dir.is_dir():
            continue
        job_id = run_dir.name
        if job_id in JOBS:
            continue
        last_accessed_at = read_job_last_access(run_dir / "meta.json")
        if now - last_accessed_at <= JOB_TTL_SECONDS:
            continue
        shutil.rmtree(run_dir, ignore_errors=True)


def delete_job(job: JobState) -> None:
    JOBS.pop(job.job_id, None)
    shutil.rmtree(job.run_dir, ignore_errors=True)


def app_root_path(request: Request) -> str:
    return request.scope.get("root_path", "").rstrip("/")


def app_url(request: Request, path: str) -> str:
    return f"{app_root_path(request)}{path}"
