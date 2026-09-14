"""Persistent UAVCAN node ID allocation store.

Writes/reads a JSON file so node IDs are never reused across sessions.
On startup the file is loaded and every node ID ever recorded is added
to ``_used_ids`` — those IDs are permanently skipped, and every device
gets a fresh allocation.

The file is written only when a *new* node ID is allocated, and again on
``flush()`` (called at shutdown) to record the final last-seen timestamp.
Repeated ``mark_seen`` calls only update the in-memory timestamp without
triggering disk I/O.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Optional

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "uavcan_node_ids.json"


class NodeIDStore:
    """JSON-backed store that never reuses a node ID across sessions."""

    def __init__(self, path: Path = DEFAULT_PATH) -> None:
        self._path = Path(path)
        self._data: dict[str, dict] = {}
        self._used_ids: set[int] = set()
        self._dirty: bool = False  # True when in-memory state needs flushing
        self._load()

    def _load(self) -> None:
        if not self._path.exists():
            return
        try:
            raw = json.loads(self._path.read_text(encoding="utf-8"))
            if isinstance(raw, dict):
                for uid, val in raw.items():
                    nid = None
                    last_seen = 0.0
                    if isinstance(val, dict):
                        nid = int(val.get("node_id", 0))
                        last_seen = float(val.get("last_seen", 0.0))
                    elif isinstance(val, int):
                        nid = val
                    if nid and nid > 0:
                        self._used_ids.add(nid)
                        self._data[str(uid)] = {
                            "node_id": nid,
                            "last_seen": last_seen,
                        }
        except (json.JSONDecodeError, ValueError, OSError):
            pass

    def _save(self) -> None:
        """Write current state to disk.  Called only on allocation or flush()."""
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            self._path.write_text(
                json.dumps(self._data, indent=2, sort_keys=True),
                encoding="utf-8",
            )
            self._dirty = False
        except OSError:
            pass

    def flush(self) -> None:
        """Persist the final last-seen timestamps.  Call at shutdown."""
        if self._dirty:
            self._save()

    def get(self, unique_id: str) -> Optional[int]:
        """Return the previously allocated node_id for this UID, or None."""
        entry = self._data.get(unique_id)
        if entry is None:
            return None
        # Skip placeholder entries (_node_*)
        if unique_id.startswith("_node_"):
            return None
        return entry["node_id"]

    def get_last_seen(self, unique_id: str) -> float:
        entry = self._data.get(unique_id)
        return entry["last_seen"] if entry else 0.0

    def set(self, unique_id: str, node_id: int) -> None:
        """Allocate a new node ID.  Writes to disk immediately."""
        self._data[unique_id] = {
            "node_id": node_id,
            "last_seen": time.time(),
        }
        self._used_ids.add(node_id)
        self._dirty = True
        self._save()

    def mark_seen(self, unique_id: str, node_id: int) -> None:
        """Update in-memory last-seen timestamp.  Does NOT write to disk."""
        entry = self._data.get(unique_id)
        if entry and entry["node_id"] == node_id:
            entry["last_seen"] = time.time()
            self._dirty = True
        elif entry is None:
            self._data[unique_id] = {
                "node_id": node_id,
                "last_seen": time.time(),
            }
            self._used_ids.add(node_id)
            self._dirty = True

    def mark_seen_by_node_id(self, node_id: int) -> None:
        """Record that ``node_id`` is currently active.  In-memory only."""
        for uid, entry in self._data.items():
            if entry["node_id"] == node_id:
                entry["last_seen"] = time.time()
                self._dirty = True
                return
        placeholder_uid = f"_node_{node_id}"
        self._data[placeholder_uid] = {
            "node_id": node_id,
            "last_seen": time.time(),
        }
        self._used_ids.add(node_id)
        self._dirty = True

    def is_node_id_used(self, node_id: int) -> bool:
        return node_id in self._used_ids

    def find_unused_node_id(self, start: int = 1, end: int = 127) -> int:
        for nid in range(start, end + 1):
            if nid not in self._used_ids:
                return nid
        raise RuntimeError(f"No free node ID in range {start}-{end}")

    @property
    def entries(self) -> dict[str, dict]:
        return dict(self._data)

    @property
    def used_ids(self) -> set[int]:
        return set(self._used_ids)

    @property
    def dirty(self) -> bool:
        return self._dirty