#!/usr/bin/env python3
"""
Build and publish the weekly PX4 Dev Call agenda to Discourse.

Run from a scheduled GitHub Actions workflow (see
.github/workflows/dev_call_post.yml). All the logic that used to live as
inline shell in that workflow lives here instead, so it can be read,
reviewed, and run locally:

    DRY_RUN=1 GH_TOKEN=... ./Tools/ci/dev_call_post.py

What it does:
  1. Computes the next call date (Wednesday) and the one-week lookback.
  2. Queries the GitHub Search API for the week's merged PRs, the open
     review backlog, new bug reports, other new issues, and every open
     issue/PR labeled "Dev Call".
  3. Renders .github/dev_call_template.md by substituting {{PLACEHOLDER}}
     tokens (literal string replace; no shell-escaping pitfalls).
  4. Checks Discourse for an existing topic with the same title and skips
     if found.
  5. Posts the rendered agenda to the configured category.
  6. Removes the "Dev Call" label from each item and leaves a backlink
     comment. Uses the issues labels/comments API, which treats PRs and
     issues uniformly, so PR items are handled correctly.

DRY_RUN=1 (or 'true') runs every read/build step and prints the rendered
body, but performs no Discourse post and no label mutation.

Policy knobs come from the environment (set in the workflow, not baked
into this script): CALL_DAY, CALL_TIME, CALL_TIMEZONE, DEV_CALL_LABEL,
LOOKBACK_DAYS, plus CATEGORY_ID/DISCOURSE_URL. Each has a sensible
default so the script also runs standalone.

Python stdlib only, to match the other Tools/ci scripts and avoid a
pip install step in CI.
"""

import datetime
import json
import os
import urllib.error
import urllib.parse
import urllib.request

from _github_helpers import GitHubClient, fail

REPO = 'PX4/PX4-Autopilot'
REPO_URL = 'https://github.com/' + REPO
TEMPLATE = '.github/dev_call_template.md'

# Weekday names accepted by CALL_DAY, mapped to datetime.weekday() (Mon=0).
WEEKDAYS = {
    'monday': 0, 'tuesday': 1, 'wednesday': 2, 'thursday': 3,
    'friday': 4, 'saturday': 5, 'sunday': 6,
}


def env(name, default=None, required=False):
    value = os.environ.get(name, default)
    if required and not value:
        fail('missing required environment variable: {}'.format(name))
    return value


def env_int(name, default):
    value = env(name, str(default)) or str(default)
    try:
        return int(value)
    except ValueError:
        fail('{} must be an integer, got: {!r}'.format(name, value))


def is_truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def compute_dates(today, call_weekday, lookback_days):
    """Return (call_date, display_date, since) as strings.

    call_date is the next occurrence of `call_weekday` strictly after
    `today` (so a run on the call day itself rolls to the following week).
    `since` is `lookback_days` before `today`, the lower bound for the
    "this week" GitHub queries.
    """
    days_ahead = (call_weekday - today.weekday() + 7) % 7
    if days_ahead == 0:
        days_ahead = 7
    call = today + datetime.timedelta(days=days_ahead)
    since = today - datetime.timedelta(days=lookback_days)
    return (
        call.strftime('%Y-%m-%d'),
        call.strftime('%b %d, %Y'),
        since.strftime('%Y-%m-%d'),
    )


def search_count(client, query):
    """Return total_count for a GitHub issues-search query."""
    path = 'search/issues?q={}'.format(urllib.parse.quote(query))
    body, _ = client.request('GET', path)
    return int(body.get('total_count', 0))


def search_items(client, query, limit=50):
    """Return up to `limit` items (number/title/html_url) for a query."""
    path = 'search/issues?q={}&per_page={}'.format(
        urllib.parse.quote(query), limit)
    body, _ = client.request('GET', path)
    return body.get('items', [])


def link_or_placeholder(count, label, search_path, empty):
    """One-line markdown: a link to the GitHub search, or `empty` if zero."""
    if count == 0:
        return empty
    return '[{} {} →]({}/{})'.format(count, label, REPO_URL, search_path)


def render(template_text, replacements):
    out = template_text
    for key, value in replacements.items():
        out = out.replace('{{' + key + '}}', value)
    return out


def build_dev_call_section(items):
    if not items:
        return '- None'
    lines = []
    for it in items:
        lines.append('- [#{} {}]({})'.format(
            it['number'], it['title'], it['html_url']))
    return '\n'.join(lines)


class Discourse:
    def __init__(self, base_url, api_key, api_user):
        self._base = base_url.rstrip('/')
        self._key = api_key
        self._user = api_user

    def _request(self, method, path, json_body=None):
        url = '{}/{}'.format(self._base, path.lstrip('/'))
        data = None
        headers = {
            'Api-Key': self._key,
            'Api-Username': self._user,
        }
        if json_body is not None:
            data = json.dumps(json_body).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        req = urllib.request.Request(url, data=data, method=method,
                                     headers=headers)
        try:
            with urllib.request.urlopen(req) as resp:
                raw = resp.read()
                return json.loads(raw.decode('utf-8')) if raw else None
        except urllib.error.HTTPError as e:
            body = e.read().decode('utf-8', errors='replace')
            raise RuntimeError('Discourse {} {} failed: HTTP {} {}\n{}'.format(
                method, url, e.code, e.reason, body))

    def topic_exists(self, title):
        path = 'search.json?q={}&in=title'.format(
            urllib.parse.quote(title))
        result = self._request('GET', path) or {}
        for topic in result.get('topics', []):
            if topic.get('title') == title:
                return topic.get('id')
        return None

    def create_topic(self, title, raw, category_id):
        result = self._request('POST', 'posts.json', {
            'title': title,
            'raw': raw,
            'category': category_id,
        }) or {}
        topic_id = result.get('topic_id')
        if not topic_id:
            raise RuntimeError(
                'Discourse did not return a topic_id: {}'.format(result))
        return '{}/t/{}'.format(self._base, topic_id)


def clear_dev_call_labels(client, items, topic_url, label):
    """Remove the Dev Call label and leave a backlink comment on each item.

    Uses the issues labels/comments endpoints, which accept both issue and
    PR numbers, so PR items are handled the same as issues.
    """
    comment = 'Added to the [Dev Call agenda]({}).'.format(topic_url)
    label_path = urllib.parse.quote(label)
    for it in items:
        number = it['number']
        client.request(
            'DELETE',
            'repos/{}/issues/{}/labels/{}'.format(REPO, number, label_path))
        client.request(
            'POST',
            'repos/{}/issues/{}/comments'.format(REPO, number),
            {'body': comment})


def main():
    dry_run = is_truthy(env('DRY_RUN', 'false'))
    token = env('GH_TOKEN', required=True)
    client = GitHubClient(token, user_agent='px4-dev-call-post')

    call_day_name = (env('CALL_DAY', 'wednesday') or '').strip().lower()
    if call_day_name not in WEEKDAYS:
        fail('CALL_DAY must be a weekday name, got: {!r}'.format(
            env('CALL_DAY')))
    lookback_days = env_int('LOOKBACK_DAYS', 7)
    dev_call_label = env('DEV_CALL_LABEL', 'Dev Call')
    timezone = env('CALL_TIMEZONE', 'Europe/Berlin')
    call_time = env('CALL_TIME', '17:00:00')

    today = datetime.date.today()
    call_date, display_date, since = compute_dates(
        today, WEEKDAYS[call_day_name], lookback_days)

    merged_q = 'repo:{} is:pr is:merged merged:>={}'.format(REPO, since)
    # Open review backlog: every open non-draft PR with no review, not just
    # those opened this week. The old workflow filtered on created:>=, which
    # hid the older backlog that makes up most of the queue.
    review_q = 'repo:{} is:pr is:open draft:false review:none'.format(REPO)
    bug_q = 'repo:{} is:issue created:>={} label:bug'.format(REPO, since)
    other_q = 'repo:{} is:issue created:>={} -label:bug'.format(REPO, since)
    dev_call_q = 'repo:{} is:open label:"{}"'.format(REPO, dev_call_label)

    merged_count = search_count(client, merged_q)
    review_count = search_count(client, review_q)
    bug_count = search_count(client, bug_q)
    other_count = search_count(client, other_q)
    dev_call_items = search_items(client, dev_call_q)

    replacements = {
        'CALL_DATE': call_date,
        'CALL_TIME': call_time,
        'CALL_TIMEZONE': timezone,
        'MERGED_COUNT': str(merged_count),
        'REVIEW_COUNT': str(review_count),
        'BUG_COUNT': str(bug_count),
        'MERGED_PRS': link_or_placeholder(
            merged_count, 'PRs merged this week',
            'pulls?q=is:pr+is:merged+merged:>={}'.format(since),
            '- None this week'),
        'REVIEW_PRS': link_or_placeholder(
            review_count, 'PRs awaiting review',
            'pulls?q=is:pr+is:open+draft:false+review:none',
            '- None pending'),
        'BUG_ISSUES': link_or_placeholder(
            bug_count, 'new bug reports',
            'issues?q=is:issue+created:>={}+label:bug'.format(since),
            '- No new bugs'),
        'OTHER_ISSUES': link_or_placeholder(
            other_count, 'new issues',
            'issues?q=is:issue+created:>={}+-label:bug'.format(since),
            '- None'),
        'DEV_CALL_ITEMS': build_dev_call_section(dev_call_items),
    }

    with open(TEMPLATE, encoding='utf-8') as f:
        body = render(f.read(), replacements)

    title = 'PX4 Dev Call: {} (Team sync, and Community Q&A)'.format(
        display_date)

    if dry_run:
        print('=== DRY RUN — title ===')
        print(title)
        print('=== DRY RUN — post body ===')
        print(body)
        print('=== {} Dev Call item(s) would be cleared ==='.format(
            len(dev_call_items)))
        return

    discourse = Discourse(
        env('DISCOURSE_URL', 'https://discuss.px4.io'),
        env('DISCOURSE_API_KEY', required=True),
        env('DISCOURSE_USER', required=True))

    existing = discourse.topic_exists(title)
    if existing:
        print('Topic already exists ({}/t/{}); skipping post.'.format(
            discourse._base, existing))
        return

    topic_url = discourse.create_topic(title, body, int(env('CATEGORY_ID', '39')))
    print('Created: {}'.format(topic_url))

    clear_dev_call_labels(client, dev_call_items, topic_url, dev_call_label)
    print('Cleared Dev Call label from {} item(s).'.format(
        len(dev_call_items)))


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as exc:
        fail(str(exc))
