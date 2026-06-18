#!/usr/bin/env python3
import os
import sys
import json
from urllib.request import Request, urlopen
from urllib.error import URLError, HTTPError

def _with_http(url: str) -> str:
    return url if url.startswith(("http://", "https://")) else f"http://{url}"

def _get_json(url: str):
    try:
        req = Request(url, headers={"Accept": "application/json"})
        with urlopen(req, timeout=10) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
        return raw, json.loads(raw)
    except (HTTPError, URLError) as e:
        print(f"Error fetching {url}: {e}", file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Invalid JSON from {url}: {e}", file=sys.stderr)
        sys.exit(1)

def sweeptop(server: str):
    base = _with_http(server.rstrip("/"))
    jobs_txt, jobs = _get_json(f"{base}/jobs")
    if not isinstance(jobs, list):
        print("Jobs endpoint did not return a list", file=sys.stderr)
        sys.exit(1)

    jobs = sorted(jobs)

    for job in jobs:
        data_txt, data = _get_json(f"{base}/jobs/{job}")

        if isinstance(data, list):
            total = len(data)
            done = sum(1 for x in data if isinstance(x, dict) and x.get("status") == "done")
            in_prog = sum(1 for x in data if isinstance(x, dict) and x.get("status") == "in_progress")
        else:
            total = done = in_prog = 0

        print(f"{job} {done}/{in_prog}/{total}")

def main():
    server = os.environ.get("DEEPSWEEP_SERVER", "localhost:13338")
    sweeptop(server)

if __name__ == "__main__":
    main()