#!/usr/bin/env python3
#
# clang-tidy-fixes-to-review.py
#
# Producer-side helper that converts a clang-tidy fixes.yml file into a
# pr-review artifact (manifest.json + comments.json) suitable for
# Tools/ci/pr-review-poster.py.
#
# This script runs inside the clang-tidy job's px4-dev container so it can
# read the source tree directly and look up byte offsets in the original
# files. The output it writes is a fully-baked array of review comments;
# the poster never reads source files or fixes.yml.
#
# ----------------------------------------------------------------------------
# ATTRIBUTION
# ----------------------------------------------------------------------------
# This script reuses the diagnostic-to-review-comment translation logic
# from platisd/clang-tidy-pr-comments. The original work is:
#
#   MIT License
#
#   Copyright (c) 2021 Dimitris Platis
#
#   Permission is hereby granted, free of charge, to any person obtaining a
#   copy of this software and associated documentation files (the
#   "Software"), to deal in the Software without restriction, including
#   without limitation the rights to use, copy, modify, merge, publish,
#   distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject to
#   the following conditions:
#
#   The above copyright notice and this permission notice shall be included
#   in all copies or substantial portions of the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
#   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
#   MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
#   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
#   CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
#   TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
#   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# Adapted parts:
#   - get_diff_line_ranges_per_file() and its inner change_to_line_range()
#   - generate_review_comments() and its nested helpers
#       (get_line_by_offset, validate_warning_applicability,
#        calculate_replacements_diff, markdown, markdown_url,
#        diagnostic_name_visual, generate_single_comment)
#   - reorder_diagnostics()
#
# Removed parts (handled by Tools/ci/pr-review-poster.py instead):
#   - post_review_comments / dismiss_change_requests / resolve_conversations
#   - the original argparse main and the requests-based HTTP layer
#
# Adaptation notes:
#   - The HTTP layer is rewritten on top of Tools/ci/_github_helpers.py so
#     this script does not depend on the third-party `requests` package.
#   - Conversation resolution (the GraphQL path) is intentionally dropped
#     for v1; revisit if it turns out to be missed.
#   - Clang-Tidy 8 upconvert is preserved verbatim.
#
# ----------------------------------------------------------------------------
# Bounded assumptions (documented for future maintainers):
#   - Source files are UTF-8 (we read them as latin_1, matching clang-tidy's
#     own byte-offset model, and the offsets we surface are line counts)
#   - Source files use LF line endings
#   - Malformed entries in fixes.yml are skipped with a warning rather than
#     crashing the job
#
# Dependencies: pyyaml + Tools/ci/_github_helpers.py.
# pyyaml is preinstalled in the px4-dev container; this script is intended
# to run there, not on bare ubuntu-latest.
"""Convert a clang-tidy fixes.yml into a pr-review artifact."""

import argparse
import difflib
import json
import os
import posixpath
import re
import sys
import urllib.parse

import yaml

import _github_helpers
from _github_helpers import fail as _fail


# Markers used inside the per-comment body to call out severity. Plain
# strings rather than emojis to keep the file emoji-free per project
# preferences; the rendered Markdown is unaffected.
SINGLE_COMMENT_MARKERS = {
    'Error': '**[error]**',
    'Warning': '**[warning]**',
    'Remark': '**[remark]**',
    'fallback': '**[note]**',
}


# ---------------------------------------------------------------------------
# Diff-range parsing (adapted from platisd)
# ---------------------------------------------------------------------------

def get_diff_line_ranges_per_file(pr_files):
    """Return a dict mapping each PR file path to a list of line ranges
    (the +new-side hunks) parsed from its patch."""

    def change_to_line_range(change):
        split_change = change.split(',')
        start = int(split_change[0])
        size = int(split_change[1]) if len(split_change) > 1 else 1
        return range(start, start + size)

    result = {}
    for pr_file in pr_files:
        # Removed binary files etc. have no patch section.
        if 'patch' not in pr_file:
            continue
        file_name = pr_file['filename']
        # Match lines like '@@ -101,8 +102,11 @@'
        git_line_tags = re.findall(
            r'^@@ -.*? +.*? @@', pr_file['patch'], re.MULTILINE)
        changes = [
            tag.replace('@@', '').strip().split()[1].replace('+', '')
            for tag in git_line_tags
        ]
        result[file_name] = [
            change_to_line_range(change) for change in changes
        ]
    return result


def fetch_pull_request_files(client, repo, pr_number):
    """Yield file metadata objects for each file modified by the PR."""
    path = 'repos/{}/pulls/{}/files'.format(repo, pr_number)
    for entry in client.paginated(path):
        yield entry


# ---------------------------------------------------------------------------
# Diagnostic ordering (adapted from platisd)
# ---------------------------------------------------------------------------

def reorder_diagnostics(diags):
    """Return diagnostics ordered Error -> Warning -> Remark -> other."""
    errors = [d for d in diags if d.get('Level') == 'Error']
    warnings = [d for d in diags if d.get('Level') == 'Warning']
    remarks = [d for d in diags if d.get('Level') == 'Remark']
    others = [
        d for d in diags
        if d.get('Level') not in {'Error', 'Warning', 'Remark'}
    ]
    if others:
        print(
            'warning: some fixes have an unexpected Level (not Error, '
            'Warning, or Remark)', file=sys.stderr)
    return errors + warnings + remarks + others


# ---------------------------------------------------------------------------
# Comment generation (adapted from platisd)
# ---------------------------------------------------------------------------

def generate_review_comments(clang_tidy_fixes, repository_root,
                              diff_line_ranges_per_file,
                              single_comment_markers):
    """Yield review comment dicts for each clang-tidy diagnostic that
    intersects the PR diff."""

    def get_line_by_offset(file_path, offset):
        # Clang-Tidy doesn't support multibyte encodings and measures
        # offsets in bytes; latin_1 makes byte offsets and string offsets
        # equivalent.
        with open(repository_root + file_path, encoding='latin_1') as fh:
            source = fh.read()
        return source[:offset].count('\n') + 1

    def validate_warning_applicability(file_path, start_line_num, end_line_num):
        assert end_line_num >= start_line_num
        for line_range in diff_line_ranges_per_file[file_path]:
            assert line_range.step == 1
            if (line_range.start <= start_line_num
                    and end_line_num < line_range.stop):
                return True
        return False

    def calculate_replacements_diff(file_path, replacements):
        # Apply replacements in reverse order so subsequent offsets do not
        # shift.
        replacements.sort(key=lambda item: (-item['Offset']))
        with open(repository_root + file_path, encoding='latin_1') as fh:
            source = fh.read()
        changed = source
        for replacement in replacements:
            changed = (
                changed[:replacement['Offset']]
                + replacement['ReplacementText']
                + changed[replacement['Offset'] + replacement['Length']:]
            )
        return difflib.Differ().compare(
            source.splitlines(keepends=True),
            changed.splitlines(keepends=True),
        )

    def markdown(s):
        md_chars = '\\`*_{}[]<>()#+-.!|'

        def escape_chars(s):
            for ch in md_chars:
                s = s.replace(ch, '\\' + ch)
            return s

        def unescape_chars(s):
            for ch in md_chars:
                s = s.replace('\\' + ch, ch)
            return s

        s = escape_chars(s)
        s = re.sub(
            "'([^']*)'",
            lambda m: '`` ' + unescape_chars(m.group(1)) + ' ``',
            s,
        )
        return s

    def markdown_url(label, url):
        return '[{}]({})'.format(label, url)

    def diagnostic_name_visual(diagnostic_name):
        visual = '**{}**'.format(markdown(diagnostic_name))
        try:
            first_dash_idx = diagnostic_name.index('-')
        except ValueError:
            return visual
        namespace = urllib.parse.quote_plus(diagnostic_name[:first_dash_idx])
        check_name = urllib.parse.quote_plus(
            diagnostic_name[first_dash_idx + 1:])
        return markdown_url(
            visual,
            'https://clang.llvm.org/extra/clang-tidy/checks/{}/{}.html'.format(
                namespace, check_name),
        )

    def generate_single_comment(file_path, start_line_num, end_line_num,
                                 name, message, single_comment_marker,
                                 replacement_text=None):
        result = {
            'path': file_path,
            'line': end_line_num,
            'side': 'RIGHT',
            'body': '{} {} {}\n{}'.format(
                single_comment_marker,
                diagnostic_name_visual(name),
                single_comment_marker,
                markdown(message),
            ),
        }
        if start_line_num != end_line_num:
            result['start_line'] = start_line_num
            result['start_side'] = 'RIGHT'
        if replacement_text is not None:
            if not replacement_text or replacement_text[-1] != '\n':
                replacement_text += '\n'
            result['body'] += '\n```suggestion\n{}```'.format(replacement_text)
        return result

    for diag in clang_tidy_fixes['Diagnostics']:
        # Upconvert clang-tidy 8 format to 9+
        if 'DiagnosticMessage' not in diag:
            diag['DiagnosticMessage'] = {
                'FileOffset': diag.get('FileOffset'),
                'FilePath': diag.get('FilePath'),
                'Message': diag.get('Message'),
                'Replacements': diag.get('Replacements', []),
            }

        diag_message = diag['DiagnosticMessage']
        diag_message['FilePath'] = posixpath.normpath(
            (diag_message.get('FilePath') or '').replace(repository_root, ''))
        for replacement in diag_message.get('Replacements') or []:
            replacement['FilePath'] = posixpath.normpath(
                replacement['FilePath'].replace(repository_root, ''))

        diag_name = diag.get('DiagnosticName', '<unknown>')
        diag_message_msg = diag_message.get('Message', '')
        level = diag.get('Level', 'Warning')
        single_comment_marker = single_comment_markers.get(
            level, single_comment_markers['fallback'])

        replacements = diag_message.get('Replacements') or []
        if not replacements:
            file_path = diag_message['FilePath']
            offset = diag_message.get('FileOffset')
            if offset is None:
                print('warning: skipping {!r}: missing FileOffset'.format(
                    diag_name), file=sys.stderr)
                continue
            if file_path not in diff_line_ranges_per_file:
                print(
                    "'{}' for {} does not apply to the files changed in "
                    'this PR'.format(diag_name, file_path))
                continue
            try:
                line_num = get_line_by_offset(file_path, offset)
            except (OSError, ValueError) as e:
                print('warning: skipping {!r} on {}: {}'.format(
                    diag_name, file_path, e), file=sys.stderr)
                continue

            print("Processing '{}' at line {} of {}...".format(
                diag_name, line_num, file_path))
            if validate_warning_applicability(file_path, line_num, line_num):
                yield generate_single_comment(
                    file_path,
                    line_num,
                    line_num,
                    diag_name,
                    diag_message_msg,
                    single_comment_marker=single_comment_marker,
                )
            else:
                print('This warning does not apply to the lines changed '
                      'in this PR')
        else:
            for file_path in {item['FilePath'] for item in replacements}:
                if file_path not in diff_line_ranges_per_file:
                    print(
                        "'{}' for {} does not apply to the files changed "
                        'in this PR'.format(diag_name, file_path))
                    continue

                line_num = 1
                start_line_num = None
                end_line_num = None
                replacement_text = None

                try:
                    diff_iter = calculate_replacements_diff(
                        file_path,
                        [r for r in replacements if r['FilePath'] == file_path],
                    )
                except (OSError, ValueError) as e:
                    print('warning: skipping {!r} on {}: {}'.format(
                        diag_name, file_path, e), file=sys.stderr)
                    continue

                for line in diff_iter:
                    # Comment line, ignore.
                    if line.startswith('? '):
                        continue
                    # A '-' line is the start or continuation of a region
                    # to replace.
                    if line.startswith('- '):
                        if start_line_num is None:
                            start_line_num = line_num
                            end_line_num = line_num
                        else:
                            end_line_num = line_num
                        if replacement_text is None:
                            replacement_text = ''
                        line_num += 1
                    # A '+' line is part of the replacement text.
                    elif line.startswith('+ '):
                        if replacement_text is None:
                            replacement_text = line[2:]
                        else:
                            replacement_text += line[2:]
                    # A context line marks the end of a replacement region.
                    elif line.startswith('  '):
                        if replacement_text is not None:
                            if start_line_num is None:
                                # Pure addition: synthesize a one-line
                                # range and append the context line to
                                # the replacement.
                                start_line_num = line_num
                                end_line_num = line_num
                                replacement_text += line[2:]

                            print("Processing '{}' at lines {}-{} of {}...".format(
                                diag_name, start_line_num, end_line_num, file_path))

                            if validate_warning_applicability(
                                    file_path, start_line_num, end_line_num):
                                yield generate_single_comment(
                                    file_path,
                                    start_line_num,
                                    end_line_num,
                                    diag_name,
                                    diag_message_msg,
                                    single_comment_marker=single_comment_marker,
                                    replacement_text=replacement_text,
                                )
                            else:
                                print(
                                    'This warning does not apply to the '
                                    'lines changed in this PR')

                            start_line_num = None
                            end_line_num = None
                            replacement_text = None

                        line_num += 1
                    else:
                        # Unknown difflib prefix; skip rather than abort.
                        print('warning: unexpected diff prefix {!r}; '
                              'skipping diagnostic'.format(line[:2]),
                              file=sys.stderr)
                        break

                # End of file with a pending replacement region.
                if replacement_text is not None and start_line_num is not None:
                    print("Processing '{}' at lines {}-{} of {}...".format(
                        diag_name, start_line_num, end_line_num, file_path))
                    if validate_warning_applicability(
                            file_path, start_line_num, end_line_num):
                        yield generate_single_comment(
                            file_path,
                            start_line_num,
                            end_line_num,
                            diag_name,
                            diag_message_msg,
                            single_comment_marker=single_comment_marker,
                            replacement_text=replacement_text,
                        )
                    else:
                        print('This warning does not apply to the lines '
                              'changed in this PR')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Convert a clang-tidy fixes.yml into a pr-review '
                    'artifact (manifest.json + comments.json).',
    )
    parser.add_argument('--fixes', required=True,
                        help='Path to fixes.yml from clang-tidy')
    parser.add_argument('--repo-root', required=True,
                        help='Path to the repository root containing the '
                             'source files referenced by fixes.yml')
    parser.add_argument('--repo', required=True,
                        help='owner/name of the repository')
    parser.add_argument('--pr-number', required=True, type=int,
                        help='Pull request number')
    parser.add_argument('--commit-sha', required=True,
                        help='40-char hex commit SHA the review will pin to')
    parser.add_argument('--out-dir', required=True,
                        help='Directory to write manifest.json and '
                             'comments.json')
    parser.add_argument(
        '--marker',
        default='<!-- pr-review-poster:clang-tidy -->',
        help='Marker string embedded in the review body so the poster '
             'can find and dismiss stale runs')
    parser.add_argument(
        '--event',
        default='REQUEST_CHANGES',
        choices=('COMMENT', 'REQUEST_CHANGES'),
        help='GitHub review event type')
    parser.add_argument(
        '--summary', default='',
        help='Optional review summary text appended to the review body')
    args = parser.parse_args(argv)

    if args.pr_number <= 0:
        _fail('--pr-number must be > 0')
    if not re.match(r'^[0-9a-f]{40}$', args.commit_sha):
        _fail('--commit-sha must be a 40-char lowercase hex string')

    token = os.environ.get('GITHUB_TOKEN')
    if not token:
        _fail('GITHUB_TOKEN is not set')

    # Normalize the repo root with a trailing slash so the platisd-style
    # str.replace() trick still strips it cleanly.
    repo_root = args.repo_root
    if not repo_root.endswith(os.sep):
        repo_root = repo_root + os.sep

    os.makedirs(args.out_dir, exist_ok=True)

    client = _github_helpers.GitHubClient(token, user_agent='px4-clang-tidy-fixes-to-review')

    print('Fetching PR file list from GitHub...')
    pr_files = list(fetch_pull_request_files(client, args.repo, args.pr_number))
    diff_line_ranges_per_file = get_diff_line_ranges_per_file(pr_files)

    print('Loading clang-tidy fixes from {}...'.format(args.fixes))
    if not os.path.isfile(args.fixes):
        # No fixes file means clang-tidy ran cleanly. Emit an empty
        # comments.json so the poster can short-circuit.
        comments = []
    else:
        with open(args.fixes, encoding='utf-8') as fh:
            clang_tidy_fixes = yaml.safe_load(fh)
        if (not clang_tidy_fixes
                or 'Diagnostics' not in clang_tidy_fixes
                or not clang_tidy_fixes['Diagnostics']):
            comments = []
        else:
            clang_tidy_fixes['Diagnostics'] = reorder_diagnostics(
                clang_tidy_fixes['Diagnostics'])
            comments = list(generate_review_comments(
                clang_tidy_fixes,
                repo_root,
                diff_line_ranges_per_file,
                single_comment_markers=SINGLE_COMMENT_MARKERS,
            ))

    print('Generated {} review comment(s)'.format(len(comments)))

    manifest = {
        'pr_number': args.pr_number,
        'marker': args.marker,
        'event': args.event,
        'commit_sha': args.commit_sha,
    }
    if args.summary:
        manifest['summary'] = args.summary

    manifest_path = os.path.join(args.out_dir, 'manifest.json')
    comments_path = os.path.join(args.out_dir, 'comments.json')
    with open(manifest_path, 'w', encoding='utf-8') as fh:
        json.dump(manifest, fh, indent=2)
        fh.write('\n')
    with open(comments_path, 'w', encoding='utf-8') as fh:
        json.dump(comments, fh, indent=2)
        fh.write('\n')

    print('Wrote {} and {}'.format(manifest_path, comments_path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
