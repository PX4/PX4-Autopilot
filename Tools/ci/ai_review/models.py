"""The model registry: which Bedrock models the review uses, in one file.

models.json maps stable aliases ("opus", "sonnet") to a Bedrock inference
profile and a display label, and sets the default model and effort per
role. Upgrading to a new model version is a one-line change there; the
workflow, the CLI and the prompts only ever use the aliases.
"""

import json
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

REGISTRY = Path(__file__).resolve().parent / 'models.json'
EFFORTS = ('low', 'medium', 'high', 'xhigh', 'max')
ROLES = ('reviewer', 'validator')
# The px4-ai-review IAM role allows these families (see README.md); a
# profile outside them would fail at runtime with AccessDenied.
PROFILE_RE = re.compile(
    r'^global\.anthropic\.claude-(opus|sonnet)-[0-9a-z-]+$')


class RegistryError(ValueError):
    pass


@dataclass(frozen=True)
class Model:
    alias: str
    bedrock_profile: str
    label: str


@dataclass(frozen=True)
class Default:
    model: str
    effort: str


@dataclass(frozen=True)
class Registry:
    models: Dict[str, Model]
    defaults: Dict[str, Default]

    def get(self, alias: str) -> Model:
        try:
            return self.models[alias]
        except KeyError:
            raise RegistryError(f'unknown model alias {alias!r}; known: '
                                f'{", ".join(sorted(self.models))}') from None


def parse(data: Any) -> Registry:
    if not isinstance(data, dict):
        raise RegistryError('registry: expected an object')
    raw_models = data.get('models')
    if not isinstance(raw_models, dict) or not raw_models:
        raise RegistryError('models: expected a non-empty object')
    models = {}
    for alias, m in raw_models.items():
        if not isinstance(m, dict):
            raise RegistryError(f'models.{alias}: expected an object')
        profile = m.get('bedrock_profile')
        label = m.get('label')
        if not isinstance(profile, str) or not PROFILE_RE.match(profile):
            raise RegistryError(
                f'models.{alias}.bedrock_profile: {profile!r} is not a '
                'global Claude Opus or Sonnet inference profile')
        if not isinstance(label, str) or not label.strip():
            raise RegistryError(f'models.{alias}.label: expected text')
        models[alias] = Model(alias=alias, bedrock_profile=profile,
                              label=label)
    raw_defaults = data.get('defaults')
    if not isinstance(raw_defaults, dict):
        raise RegistryError('defaults: expected an object')
    defaults = {}
    for role in ROLES:
        d = raw_defaults.get(role)
        if not isinstance(d, dict):
            raise RegistryError(f'defaults.{role}: missing')
        if d.get('model') not in models:
            raise RegistryError(f'defaults.{role}.model: {d.get("model")!r} '
                                'is not a model alias')
        if d.get('effort') not in EFFORTS:
            raise RegistryError(f'defaults.{role}.effort: must be one of '
                                f'{EFFORTS}')
        defaults[role] = Default(model=d['model'], effort=d['effort'])
    return Registry(models=models, defaults=defaults)


def load(path: Optional[Path] = None) -> Registry:
    return parse(json.loads((path or REGISTRY).read_text(encoding='utf-8')))


Runner = Callable[[Sequence[str]], 'subprocess.CompletedProcess[str]']


def _run(cmd: Sequence[str]) -> 'subprocess.CompletedProcess[str]':
    return subprocess.run(list(cmd), capture_output=True, text=True,
                          check=False, timeout=120)


def check(registry: Registry, region: str,
          runner: Runner = _run) -> List[str]:
    """Send a one-token request to every model; return the failures.

    Run with the credentials the review will use. It catches the three
    things that break an upgrade: a profile that does not exist in the
    region, a model without an accepted Marketplace agreement, and an IAM
    policy that does not cover the new model.
    """
    failures = []
    for m in registry.models.values():
        proc = runner(['aws', 'bedrock-runtime', 'converse', '--region',
                       region, '--model-id', m.bedrock_profile,
                       '--messages',
                       '[{"role":"user","content":[{"text":"Reply OK"}]}]',
                       '--inference-config', '{"maxTokens":16}',
                       '--query', 'output.message.content[0].text',
                       '--output', 'text'])
        if proc.returncode != 0:
            failures.append(f'{m.alias} ({m.bedrock_profile}): '
                            f'{proc.stderr.strip()[-400:]}')
    return failures
