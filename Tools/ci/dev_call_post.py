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

DEV_CALL_LABEL = 'Dev Call'
# Discourse date bbcode timezone. IANA name, not "CET", so the rendered
# call time tracks CET/CEST daylight-saving transitions automatically.
CALL_TIMEZONE = 'Europe/Berlin'


def env(name, default=None, required=False):
    value = os.environ.get(name, default)
    if required and not value:
        fail('missing required environment variable: {}'.format(name))
    return value


def is_truthy(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def compute_dates(today):
    """Return (call_date, display_date, week_ago) as strings.

    call_date is the next Wednesday strictly after `today` (matches the
    Monday-scheduled run; if dispatched on a Wednesday it rolls to the
    following week, same as the old `date -d "next Wednesday"`).
    """
    # weekday(): Mon=0 .. Sun=6; Wednesday is 2.
    days_ahead = (2 - today.weekday() + 7) % 7
    if days_ahead == 0:
        days_ahead = 7
    call = today + datetime.timedelta(days=days_ahead)
    week_ago = today - datetime.timedelta(days=7)
    return (
        call.strftime('%Y-%m-%d'),
        call.strftime('%b %d, %Y'),
        week_ago.strftime('%Y-%m-%d'),
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


def clear_dev_call_labels(client, items, topic_url):
    """Remove the Dev Call label and leave a backlink comment on each item.

    Uses the issues labels/comments endpoints, which accept both issue and
    PR numbers, so PR items are handled the same as issues.
    """
    comment = 'Added to the [Dev Call agenda]({}).'.format(topic_url)
    label_path = urllib.parse.quote(DEV_CALL_LABEL)
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

    today = datetime.date.today()
    call_date, display_date, week_ago = compute_dates(today)

    merged_q = 'repo:{} is:pr is:merged merged:>={}'.format(REPO, week_ago)
    # Open review backlog: every open non-draft PR with no review, not just
    # those opened this week. The old workflow filtered on created:>=, which
    # hid the older backlog that makes up most of the queue.
    review_q = 'repo:{} is:pr is:open draft:false review:none'.format(REPO)
    bug_q = 'repo:{} is:issue created:>={} label:bug'.format(REPO, week_ago)
    other_q = 'repo:{} is:issue created:>={} -label:bug'.format(REPO, week_ago)
    dev_call_q = 'repo:{} is:open label:"{}"'.format(REPO, DEV_CALL_LABEL)

    merged_count = search_count(client, merged_q)
    review_count = search_count(client, review_q)
    bug_count = search_count(client, bug_q)
    other_count = search_count(client, other_q)
    dev_call_items = search_items(client, dev_call_q)

    replacements = {
        'CALL_DATE': call_date,
        'CALL_TIMEZONE': CALL_TIMEZONE,
        'MERGED_COUNT': str(merged_count),
        'REVIEW_COUNT': str(review_count),
        'BUG_COUNT': str(bug_count),
        'MERGED_PRS': link_or_placeholder(
            merged_count, 'PRs merged this week',
            'pulls?q=is:pr+is:merged+merged:>={}'.format(week_ago),
            '- None this week'),
        'REVIEW_PRS': link_or_placeholder(
            review_count, 'PRs awaiting review',
            'pulls?q=is:pr+is:open+draft:false+review:none',
            '- None pending'),
        'BUG_ISSUES': link_or_placeholder(
            bug_count, 'new bug reports',
            'issues?q=is:issue+created:>={}+label:bug'.format(week_ago),
            '- No new bugs'),
        'OTHER_ISSUES': link_or_placeholder(
            other_count, 'new issues',
            'issues?q=is:issue+created:>={}+-label:bug'.format(week_ago),
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

    clear_dev_call_labels(client, dev_call_items, topic_url)
    print('Cleared Dev Call label from {} item(s).'.format(
        len(dev_call_items)))


if __name__ == '__main__':
    try:
        main()
    except RuntimeError as exc:
        fail(str(exc))
