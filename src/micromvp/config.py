"""
One config file per deployment.

Every module — environment, controller, coordinator — reads the fields it
needs out of a single YAML file, instead of each carrying its own config
class with its own defaults. To reproduce a setup you copy one file.

Lookups are strict on purpose. A missing field raises immediately, naming
the field and who asked for it, rather than silently falling back to a
default that does not match your hardware:

    cfg = load_config("config/car_v4.yaml")
    size = cfg.require("car.marker_size_mm", float, who="ArucoObserver")

    ConfigError: missing required field 'car.marker_size_mm'
      needed by : ArucoObserver
      config    : config/car_v4.yaml
      add it to the 'car' section.

Usage:
    from micromvp.config import load_config
    cfg = load_config("config/car_v4.yaml")
    env = RealEnv(cfg)
"""
from __future__ import annotations

import os
from typing import Any, Dict, List, Optional, Type, TypeVar

import yaml

T = TypeVar("T")

_MISSING = object()


class ConfigError(Exception):
    """Raised when a required field is absent, or present with a bad type."""


class Config:
    """A read-only view over the parsed YAML, addressed by dotted paths.

    Tracks which fields were read so that typos in the file can be reported
    back to the user (see `unused_fields`).
    """

    def __init__(self, data: Dict[str, Any], source: str = "<dict>") -> None:
        if not isinstance(data, dict):
            raise ConfigError(f"{source}: top level must be a mapping, got {type(data).__name__}")
        self._data = data
        self._source = source
        self._used: set[str] = set()

    # ------------------------------------------------------------------
    # Constructors
    # ------------------------------------------------------------------

    @classmethod
    def from_dict(cls, data: Dict[str, Any], source: str = "<dict>") -> "Config":
        return cls(data, source)

    @property
    def source(self) -> str:
        return self._source

    # ------------------------------------------------------------------
    # Lookup
    # ------------------------------------------------------------------

    def require(self, path: str, kind: Type[T] = None, *, who: str = "") -> T:
        """Return the value at `path`, or raise ConfigError explaining what is missing."""
        value = self._lookup(path)
        if value is _MISSING:
            raise ConfigError(self._missing_message(path, who))
        return self._coerce(path, value, kind, who)

    def optional(self, path: str, default: T, kind: Type[T] = None, *, who: str = "") -> T:
        """Return the value at `path`, or `default` if it is absent.

        Reserved for fields that genuinely have a safe universal default
        (debug toggles and the like). Anything that depends on the physical
        setup should use `require` so a mismatch surfaces loudly.
        """
        value = self._lookup(path)
        if value is _MISSING:
            return default
        return self._coerce(path, value, kind, who)

    def require_pair(self, path: str, *, who: str = "") -> tuple[float, float]:
        """Read a two-element numeric list, e.g. an (x, y) offset in cm."""
        raw = self.require(path, list, who=who)
        if len(raw) != 2 or not all(
            isinstance(v, (int, float)) and not isinstance(v, bool) for v in raw
        ):
            raise ConfigError(self._type_message(path, raw, "a list of two numbers", who))
        return (float(raw[0]), float(raw[1]))

    def section(self, path: str, *, who: str = "") -> "Config":
        """Return a sub-config rooted at `path`, for iterating a whole block."""
        value = self.require(path, who=who)
        if not isinstance(value, dict):
            raise ConfigError(
                f"{self._source}: field '{path}' must be a mapping, "
                f"got {type(value).__name__}"
            )
        sub = Config(value, source=self._source)
        sub._used = self._used  # share tracking with the parent
        return sub

    def has(self, path: str) -> bool:
        return self._lookup(path) is not _MISSING

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _lookup(self, path: str) -> Any:
        node: Any = self._data
        for part in path.split("."):
            if not isinstance(node, dict) or part not in node:
                return _MISSING
            node = node[part]
        self._used.add(path)
        return node

    def _coerce(self, path: str, value: Any, kind: Optional[type], who: str) -> Any:
        if kind is None or value is None:
            return value
        # bool is a subclass of int; keep them distinct so `1` is not a bool
        if kind is bool:
            if isinstance(value, bool):
                return value
            raise ConfigError(self._type_message(path, value, "a boolean", who))
        if kind is float:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ConfigError(self._type_message(path, value, "a number", who))
            return float(value)
        if kind is int:
            if isinstance(value, bool) or not isinstance(value, int):
                raise ConfigError(self._type_message(path, value, "an integer", who))
            return value
        if not isinstance(value, kind):
            raise ConfigError(self._type_message(path, value, kind.__name__, who))
        return value

    def _missing_message(self, path: str, who: str) -> str:
        section = path.rsplit(".", 1)[0] if "." in path else "(top level)"
        lines = [
            f"missing required field '{path}'",
            f"  needed by : {who or 'micromvp'}",
            f"  config    : {self._source}",
            f"  add it to the '{section}' section.",
        ]
        near = self._nearby(path)
        if near:
            lines.append(f"  that section currently has: {', '.join(near)}")
        return "\n".join(lines)

    def _type_message(self, path: str, value: Any, expected: str, who: str) -> str:
        return (
            f"field '{path}' must be {expected}, got {value!r}\n"
            f"  needed by : {who or 'micromvp'}\n"
            f"  config    : {self._source}"
        )

    def _nearby(self, path: str) -> List[str]:
        """List the sibling keys of a missing field, to hint at typos."""
        if "." not in path:
            return sorted(self._data.keys())
        parent = self._lookup(path.rsplit(".", 1)[0])
        if isinstance(parent, dict):
            return sorted(parent.keys())
        return []

    # ------------------------------------------------------------------
    # Validation helpers
    # ------------------------------------------------------------------

    def unused_fields(self) -> List[str]:
        """Leaf fields present in the file that nothing ever read.

        Usually a typo, or a leftover from an older version of the file.
        Reading a whole block (say `obstacle.shapes`) counts as reading
        everything under it.
        """
        present: List[str] = []
        _collect_leaves(self._data, "", present)
        return sorted(p for p in present if not self._was_read(p))

    def _was_read(self, path: str) -> bool:
        """True if `path` or any block containing it was read."""
        parts = path.split(".")
        for i in range(1, len(parts) + 1):
            if ".".join(parts[:i]) in self._used:
                return True
        return False


def _collect_leaves(node: Any, prefix: str, out: List[str]) -> None:
    if isinstance(node, dict):
        for key, value in node.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            if isinstance(value, dict):
                _collect_leaves(value, path, out)
            else:
                out.append(path)
    elif prefix:
        out.append(prefix)


def load_config(path: str) -> Config:
    """Load a deployment config. `path` is resolved relative to the cwd."""
    if not os.path.isfile(path):
        raise ConfigError(
            f"config file not found: {path}\n"
            f"  cwd: {os.getcwd()}\n"
            f"  the repo ships ready-made configs under config/, "
            f"e.g. config/car_v4.yaml"
        )
    with open(path, "r", encoding="utf-8") as fh:
        try:
            data = yaml.safe_load(fh)
        except yaml.YAMLError as exc:
            raise ConfigError(f"{path}: could not parse YAML\n  {exc}") from exc
    if data is None:
        raise ConfigError(f"{path}: file is empty")
    return Config(data, source=path)
