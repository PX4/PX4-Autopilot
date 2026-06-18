#!/usr/bin/env python3
"""Client utilities for fetching a task and invoking a local shell runner.

This script fetches a single task from a DeepSweep server and then executes a
shell script in the current working directory (`./deepsweep-client.sh`). The
script receives the following environment variables:

- DEEPSWEEP_JOB: The job name
- DEEPSWEEP_TASK_ID: The numeric task identifier
- DEEPSWEEP_SPEC: The JSON-encoded task spec (compact string)
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import urllib.error
import urllib.request
from typing import Any, Dict, Optional, Tuple


def fetch_task(base_url: str, job_name: str) -> Optional[Tuple[int, Dict[str, Any]]]:
    """Fetch a single task from the server.

    Sends a POST to `{base_url}/jobs/{job_name}/tasks` and expects a JSON
    response of the shape `{ "task_id": int, "spec": object }`.

    Returns
    -------
    Optional[Tuple[int, Dict[str, Any]]]
        `(task_id, spec)` if a task is available, otherwise `None` if the
        server reports no tasks are available (HTTP 404 with proper error).

    Raises
    ------
    RuntimeError
        If an unexpected HTTP error occurs.
    """
    url = f"{base_url.rstrip('/')}/jobs/{job_name}/tasks"
    req = urllib.request.Request(url=url, method="POST")
    try:
        with urllib.request.urlopen(req) as resp:
            payload = json.loads(resp.read().decode("utf-8"))
            task_id = int(payload["task_id"])  # may raise KeyError/ValueError
            spec = payload["spec"]
            if not isinstance(spec, dict):
                # Normalize non-dict specs into a dict for consistency
                spec = {"value": spec}
            return task_id, spec
    except urllib.error.HTTPError as e:
        # 404 when job not found or when no tasks are available.
        if e.code == 404:
            try:
                body = e.read().decode("utf-8")
                info = json.loads(body)
                # Treat "no tasks available" as None; other 404s as hard errors
                if info.get("error") == "no tasks available":
                    return None
                raise RuntimeError(f"Server error: {info.get('error', 'not found')}")
            except Exception:
                return None
        raise RuntimeError(f"HTTP error {e.code}: {e.reason}") from e
    except urllib.error.URLError as e:
        raise RuntimeError(f"Failed to connect to {url}: {e.reason}") from e


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="DeepSweep task client")
    parser.add_argument("job", help="Job name to fetch tasks from")
    parser.add_argument(
        "--url",
        default=os.environ.get("DEEPSWEEP_SERVER", "localhost:13338"),
        help="Base URL of the DeepSweep server (default: %(default)s)",
    )
    parser.add_argument(
        "--script",
        default=os.environ.get("DEEPSWEEP_SCRIPT", "./deepsweep-client.sh"),
        help="Path to the shell script to execute (default: %(default)s)",
    )
    parser.add_argument(
        "--shell",
        action="store_true",
        help="Drop into an interactive shell with DEEPSWEEP_* env vars set (exec).",
    )
    parser.add_argument(
        "--emit-env",
        action="store_true",
        help="Print export lines for DEEPSWEEP_* env vars (use with: source <(…)).",
    )
    args = parser.parse_args(argv)

    task = fetch_task(args.url, args.job)
    if task is None:
        print("No tasks available.")
        return 2

    task_id, spec = task
    env = os.environ.copy()
    env["DEEPSWEEP_SERVER"] = args.url
    env["DEEPSWEEP_TASK_ID"] = str(task_id)
    env["DEEPSWEEP_SPEC"] = json.dumps(spec, separators=(",", ":"))
    env["DEEPSWEEP_JOB"] = args.job

    if args.shell:
        shell = env.get("SHELL", "/bin/bash")
        # Replace current process with an interactive shell that inherits env
        os.execvpe(shell, [shell, "-i"], env)
        return 0  # not reached

    if args.emit_env:
        def _sq(val: str) -> str:
            return "'" + val.replace("'", "'\"'\"'") + "'"

        print(f"export DEEPSWEEP_SERVER={_sq(env['DEEPSWEEP_SERVER'])}")
        print(f"export DEEPSWEEP_TASK_ID={_sq(env['DEEPSWEEP_TASK_ID'])}")
        print(f"export DEEPSWEEP_SPEC={_sq(env['DEEPSWEEP_SPEC'])}")
        print(f"export DEEPSWEEP_JOB={_sq(env['DEEPSWEEP_JOB'])}")
        return 0

    if not os.path.exists(args.script):
        print(f"Runner script not found: {args.script}", file=sys.stderr)
        return 3

    try:
        result = subprocess.run([args.script], env=env, check=False)
        return int(result.returncode)
    except OSError as e:
        print(f"Failed to execute {args.script}: {e}", file=sys.stderr)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())


