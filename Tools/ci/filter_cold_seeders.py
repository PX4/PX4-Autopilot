#!/usr/bin/env python3
"""Filter the seeder matrix to chip families with no warm ccache.

Reads the full seeder matrix JSON on stdin and prints a matrix containing
only the families with no cache entry under their ccache namespace in the
RunsOn magic-cache S3 bucket. Exits non-zero when probing is impossible
(missing bucket env, aws cli error) so the caller can fail open and seed
every family, which is the pre-probe behavior.
"""
import json
import os
import subprocess
import sys


def has_cache(bucket, prefix):
    result = subprocess.run(
        ["aws", "s3api", "list-objects-v2", "--bucket", bucket,
         "--prefix", prefix, "--max-items", "1", "--output", "json"],
        capture_output=True, text=True, timeout=30)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip())
    return bool(result.stdout.strip()) and "Contents" in json.loads(result.stdout)


def main():
    bucket = os.environ.get("RUNS_ON_S3_BUCKET_CACHE")
    if not bucket:
        print("RUNS_ON_S3_BUCKET_CACHE not set, cannot probe", file=sys.stderr)
        return 1
    matrix = json.load(sys.stdin)
    cold = []
    for entry in matrix["include"]:
        # The same namespace the build jobs search as their last
        # restore-keys fallback; the magic cache stores GitHub cache
        # entries under the cache/ prefix of the bucket.
        prefix = f"cache/ccache-{entry['chip_family']}-{entry['runner']}-"
        try:
            warm = has_cache(bucket, prefix)
        except Exception as exc:
            print(f"probe failed for {prefix}: {exc}", file=sys.stderr)
            return 1
        state = "warm" if warm else "COLD"
        print(f"::notice title=seeder probe::{entry['chip_family']}/"
              f"{entry['runner']}: {state}", file=sys.stderr)
        if not warm:
            cold.append(entry)
    json.dump({"include": cold}, sys.stdout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
