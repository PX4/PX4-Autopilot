#!/usr/bin/env python3
"""Minimal REST server for ML experiment orchestration."""
from __future__ import annotations

import argparse, http.server, json, os, sqlite3, urllib.parse
from datetime import datetime, timezone
from typing import Callable, TypeVar

DB_FILE = os.environ.get("JOB_SERVER_DB", "jobs.db")


def init_db(path: str = DB_FILE) -> None:
    conn = sqlite3.connect(path)
    conn.executescript(
        "CREATE TABLE IF NOT EXISTS jobs(id INTEGER PRIMARY KEY,name TEXT UNIQUE NOT NULL);"
        "CREATE TABLE IF NOT EXISTS tasks(id INTEGER PRIMARY KEY,job_id INTEGER NOT NULL REFERENCES jobs(id) ON DELETE CASCADE,"
        "spec TEXT NOT NULL,status TEXT NOT NULL DEFAULT 'pending',result TEXT,updated TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP);"
    )
    conn.commit(); conn.close()


F = TypeVar("F", bound=Callable[..., object])

def with_connection(fn: F) -> F:  # type: ignore[override]
    def inner(self, *args, **kwargs):  # type: ignore[override]
        conn = sqlite3.connect(DB_FILE, timeout=30)
        try:
            return fn(self, conn, *args, **kwargs)
        finally:
            conn.close()
    return inner  # type: ignore[return-value]


class RequestHandler(http.server.BaseHTTPRequestHandler):
    server_version = "RLJobServer/0.5"

    # ------------------------------------------------------------- utilities
    def _send(self, status: int, body=None):
        self.send_response(status)
        if body is not None:
            payload = json.dumps(body).encode()
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers(); self.wfile.write(payload)
        else:
            self.end_headers()

    def _read_body(self) -> str:
        l = int(self.headers.get("Content-Length", "0"))
        return self.rfile.read(l).decode() if l else ""

    def _split(self):
        return urllib.parse.urlparse(self.path).path.strip("/").split("/")

    # --------------------------------------------------------------- routing
    def do_GET(self):  # noqa: N802
        parts = self._split()
        if not parts or parts[0] != "jobs":
            return self._send(404, {"error": "Unknown endpoint"})
        if len(parts) == 1:
            return self._handle_list_jobs()
        if len(parts) == 2:
            return self._handle_list_tasks(parts[1])
        return self._send(404, {"error": "Unknown endpoint"})

    def do_POST(self):  # noqa: N802
        parts = self._split()
        if not parts or parts[0] != "jobs":
            return self._send(404, {"error": "Unknown endpoint"})
        try:
            if len(parts) == 2:
                return self._handle_create_job(parts[1])
            if len(parts) == 3 and parts[2] == "tasks":
                return self._handle_take_task(parts[1])
            if len(parts) == 4 and parts[2] == "tasks":
                return self._handle_report_task(parts[1], int(parts[3]))
            if len(parts) == 3 and parts[2] == "reset":
                return self._handle_reset(parts[1])
        except ValueError:
            return self._send(400, {"error": "Malformed URL"})
        return self._send(404, {"error": "Unknown endpoint"})

    # ------------------------------------------------------ GET endpoints
    @with_connection
    def _handle_list_jobs(self, conn: sqlite3.Connection):
        rows = conn.execute("SELECT name FROM jobs ORDER BY name").fetchall()
        return self._send(200, [r[0] for r in rows])

    @with_connection
    def _handle_list_tasks(self, conn: sqlite3.Connection, job: str):
        row = conn.execute("SELECT id FROM jobs WHERE name=?", (job,)).fetchone()
        if not row:
            return self._send(404, {"error": "Job not found"})
        job_id = row[0]
        rows = conn.execute("SELECT id,status,spec,result FROM tasks WHERE job_id=? ORDER BY id", (job_id,)).fetchall()
        tasks = []
        for tid, status, spec, result in rows:
            tasks.append({
                "id": tid,
                "status": status,
                "spec": json.loads(spec),
                "result": json.loads(result) if result else None,
            })
        return self._send(200, tasks)

    # ---------------------------------------------------- POST endpoints
    @with_connection
    def _handle_create_job(self, conn: sqlite3.Connection, job: str):
        if conn.execute("SELECT 1 FROM jobs WHERE name=?", (job,)).fetchone():
            return self._send(409, {"error": "Job already exists"})
        body = self._read_body(); tasks = [l for l in body.splitlines() if l.strip()]
        if not tasks:
            return self._send(400, {"error": "No tasks provided"})
        try:
            tasks = [json.dumps(json.loads(t)) for t in tasks]
        except json.JSONDecodeError:
            tasks = [json.dumps(t) for t in tasks]

        cur = conn.cursor(); cur.execute("INSERT INTO jobs(name) VALUES (?)", (job,)); job_id = cur.lastrowid
        cur.executemany("INSERT INTO tasks(job_id,spec,status) VALUES (?,?,'pending')", ((job_id, t) for t in tasks))
        conn.commit(); return self._send(201, {"created_tasks": len(tasks)})

    @with_connection
    def _handle_take_task(self, conn: sqlite3.Connection, job: str):
        conn.isolation_level = None; cur = conn.cursor(); cur.execute("BEGIN IMMEDIATE")
        row = cur.execute("SELECT id FROM jobs WHERE name=?", (job,)).fetchone()
        if not row:
            conn.execute("ROLLBACK"); return self._send(404, {"error": "Job not found"})
        job_id = row[0]
        row = cur.execute("SELECT id,spec FROM tasks WHERE job_id=? AND status='pending' ORDER BY id ASC LIMIT 1", (job_id,)).fetchone()
        if not row:
            conn.execute("COMMIT"); return self._send(404, {"error": "no tasks available"})
        task_id, spec = row
        cur.execute("UPDATE tasks SET status='in_progress',updated=? WHERE id=?", (datetime.now(timezone.utc).isoformat(), task_id))
        conn.execute("COMMIT"); return self._send(200, {"task_id": task_id, "spec": json.loads(spec)})

    @with_connection
    def _handle_report_task(self, conn: sqlite3.Connection, job: str, task: int):
        try:
            result = json.loads(self._read_body() or "{}")
        except json.JSONDecodeError:
            return self._send(400, {"error": "Result is not valid JSON"})
        cur = conn.cursor()
        cur.execute("UPDATE tasks SET status='done',result=?,updated=? WHERE id=? AND status='in_progress'", (json.dumps(result), datetime.now(timezone.utc).isoformat(), task))
        if cur.rowcount == 0:
            return self._send(404, {"error": "Task not found or not in progress"})
        conn.commit(); return self._send(200, {"ok": True})

    @with_connection
    def _handle_reset(self, conn: sqlite3.Connection, job: str):
        row = conn.execute("SELECT id FROM jobs WHERE name=?", (job,)).fetchone()
        if not row:
            return self._send(404, {"error": "Job not found"})
        job_id = row[0]; cur = conn.cursor()
        cur.execute("UPDATE tasks SET status='pending',updated=? WHERE job_id=? AND status='in_progress'", (datetime.now(timezone.utc).isoformat(), job_id))
        conn.commit(); return self._send(200, {"reset_tasks": cur.rowcount})

    def log_message(self, fmt, *args):
        return


def main():
    p = argparse.ArgumentParser(description="Minimal REST server for ML experiment orchestration.")
    p.add_argument("--ip", default="0.0.0.0", help="Host to bind (default 0.0.0.0)")
    p.add_argument("--port", type=int, default=8000, help="Port to listen on (default 8000)")
    args = p.parse_args()

    init_db()
    print(f"DeepSweep Server listening on http://{args.ip}:{args.port}")
    http.server.ThreadingHTTPServer((args.ip, args.port), RequestHandler).serve_forever()

if __name__ == "__main__":
    main()
