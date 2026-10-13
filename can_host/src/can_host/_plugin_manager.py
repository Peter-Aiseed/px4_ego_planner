"""Generic self-registration + lazy-discovery plugin manager.

Provides the shared mechanism used by device and bus factories:

- :meth:`register` — class decorator that stores the class by name.
- :meth:`_discover` — scans a package via :func:`pkgutil.iter_modules`,
  importing every submodule so self-registrations run.  Infrastructure modules
  import cleanly but register nothing, so they are silently skipped.  Modules
  with missing optional dependencies are warned and skipped.
- :meth:`create` / :meth:`create_all` — instantiate registered classes,
  filtering kwargs per-class via :func:`inspect.signature`.

Both factories instantiate a ``PluginManager`` parametrised by their base class::

    _devices = PluginManager("can_host.devices", "device")
    register_device = _devices.register
"""

from __future__ import annotations

import importlib
import inspect
import logging
import pkgutil
from collections.abc import Callable
from typing import Any, Generic, TypeVar

logger = logging.getLogger(__name__)

T = TypeVar("T")


class PluginManager(Generic[T]):
    """Generic plugin manager with lazy discovery via package scanning."""

    def __init__(self, package: str, kind: str = "backend") -> None:
        self._package = package
        self._kind = kind
        self._entries: dict[str, type[T]] = {}
        self._discovered = False

    def register(self, name: str) -> Callable[[type[T]], type[T]]:
        """Class decorator that registers *cls* under *name*.

        Usage::

            @register("socketcan")
            class SocketCANBus(CANBus):
                ...
        """

        def decorator(cls: type[T]) -> type[T]:
            self._entries[name] = cls
            return cls

        return decorator

    def _discover(self) -> None:
        """Import every submodule so self-registrations run.

        There is no hardcoded list — any module that calls :meth:`register`
        becomes available automatically.  Infrastructure modules import
        cleanly but register nothing, so they are silently skipped.  Modules
        that fail to import (missing optional dep) are warned and skipped.
        Idempotent.
        """
        if self._discovered:
            return
        self._discovered = True

        pkg = importlib.import_module(self._package)
        for _, name, _ in pkgutil.iter_modules(
            pkg.__path__, prefix=f"{self._package}."
        ):
            short = name.rsplit(".", 1)[-1]
            before = set(self._entries)
            try:
                importlib.import_module(name)
            except ImportError as exc:
                logger.warning("Skipping %s module %s: %s", self._kind, name, exc)
                continue
            if not set(self._entries) - before:
                logger.debug(
                    "Module %s imported but registered no %s", short, self._kind
                )

    def list_registered(self) -> list[str]:
        """Return all registered names."""
        self._discover()
        return list(self._entries)

    def resolve(self, name: str) -> type[T]:
        """Return the registered class for *name*.

        Raises :class:`ValueError` if *name* is not registered.
        """
        self._discover()
        cls = self._entries.get(name)
        if cls is None:
            raise ValueError(
                f"unknown {self._kind}: {name!r}. Known: {sorted(self._entries)}"
            )
        return cls

    def create(self, name: str, event_bus: Any, **kwargs: Any) -> T:
        """Instantiate a registered component by name.

        *kwargs* are filtered to the class's accepted parameters via
        :func:`inspect.signature`.
        """
        cls = self.resolve(name)
        sig = inspect.signature(cls.__init__)
        valid = {k: v for k, v in kwargs.items() if k in sig.parameters}
        return cls(event_bus=event_bus, **valid)  # type: ignore[call-arg]

    def create_all(
        self,
        event_bus: Any,
        skip_failures: bool = True,
        **kwargs: Any,
    ) -> list[T]:
        """Instantiate all registered components.

        *kwargs* are filtered per-class via :func:`inspect.signature`.  When
        *skip_failures* is True (default), components whose constructor raises
        are logged and skipped; otherwise the exception propagates.
        """
        self._discover()
        instances: list[T] = []
        for name, cls in list(self._entries.items()):
            sig = inspect.signature(cls.__init__)
            valid = {k: v for k, v in kwargs.items() if k in sig.parameters}
            try:
                instances.append(cls(event_bus=event_bus, **valid))  # type: ignore[call-arg]
            except Exception as exc:
                if skip_failures:
                    logger.warning("Skipping %s %r: %s", self._kind, name, exc)
                else:
                    raise
        return instances
