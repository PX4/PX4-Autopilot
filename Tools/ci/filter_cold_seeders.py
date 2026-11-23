#!/usr/bin/env python3
"""Emit the seeder matrix filtered to chip families with no warm ccache.

Reads the full seeder matrix JSON on stdin and writes two GitHub Actions
outputs (to $GITHUB_OUTPUT, or stdout when unset for local runs):

  cold_seeders  matrix containing only the families with no cache entry
                under their ccache namespace in the RunsOn magic-cache
                S3 bucket
  has_cold      "true"/"false", whether cold_seeders has any entries

Fails open, never non-zero: on any probe problem (missing bucket env,
aws cli error, unexpected input) the full input matrix is emitted with
has_cold=true, reproducing unconditional seeding. The probe can only
skip work, never leave a truly cold family unseeded.
"""
import json
import os
import subprocess
import sys


def scope_key_basenames(bucket, scope):
    """Cache-key basenames in one ref scope of the magic-cache bucket.

    The magic cache stores GitHub cache entries as
    cache/v1/{org}/{repo}/{ref}/{version-hash}/{cache-key}; the version
    hash is the actions/cache path+compression digest and not knowable
    here, so list the whole ref scope (auto-paginated) and match on the
    final path component.
    """
    prefix = f"cache/v1/{os.environ['GITHUB_REPOSITORY']}/{scope}/"
    result = subprocess.run(
        ["aws", "s3", "ls", f"s3://{bucket}/{prefix}", "--recursive"],
        capture_output=True, text=True, timeout=120)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip())
    return [line.split()[-1].rsplit("/", 1)[-1]
            for line in result.stdout.splitlines() if line.strip()]


def filter_cold(matrix):
    bucket = os.environ["RUNS_ON_S3_BUCKET_CACHE"]
    # Default-branch caches are visible to every ref, so main-scope
    # warmth is what the build jobs' last restore-keys fallback can
    # always reach regardless of the branch being built.
    scope = os.environ.get("SEEDER_PROBE_SCOPE", "refs/heads/main")
    basenames = scope_key_basenames(bucket, scope)
    cold = []
    for entry in matrix["include"]:
        # The namespace root is minted by generate_board_targets_json.py
        # and arrives on every matrix entry; no key format knowledge here.
        prefix = entry["cache_prefix"] + "-"
        warm = any(name.startswith(prefix) for name in basenames)
        print(f"::notice title=seeder probe::{entry['chip_family']}/"
              f"{entry['runner']}: {'warm' if warm else 'COLD'}", file=sys.stderr)
        if not warm:
            cold.append(entry)
    return {"include": cold}


def write_outputs(cold_json, has_cold):
    payload = (f"cold_seeders<<EOF\n{cold_json}\nEOF\n"
               f"has_cold={'true' if has_cold else 'false'}\n")
    path = os.environ.get("GITHUB_OUTPUT")
    if path:
        with open(path, "a") as out:
            out.write(payload)
    else:
        sys.stdout.write(payload)


def main():
    raw = sys.stdin.read()
    try:
        matrix = json.loads(raw)
    except ValueError as exc:
        # Unparseable input passes through verbatim so the seed matrix
        # fails loudly downstream instead of being silently skipped.
        print(f"::notice title=seeder probe::fail-open, bad input: {exc}",
              file=sys.stderr)
        write_outputs(raw, True)
        return 0
    try:
        cold = filter_cold(matrix)
    except Exception as exc:
        print(f"::notice title=seeder probe::fail-open, seeding all "
              f"families: {exc}", file=sys.stderr)
        cold = matrix
    write_outputs(json.dumps(cold), bool(cold["include"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
