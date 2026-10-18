"""What a perception backend is, and how one is chosen.

A backend answers two questions about one image:

    check_status(image, fixture_hint="")  -> FixtureStatus
    detect_stains(image, status=None)     -> [Detection, ...]   PIXELS of that image

and declares three things about itself, so a caller can tell a capability it does
not have from a thing it looked for and did not find:

    supports_state_classification   can it read lid/seat state, or only guess?
    supports_escalation             can it recognise "a person must see this"?
    supports_permanent_detection    can it tell permanent marks from dirt?

Those three come from butterfli (ai_client.py:730-744), where they were derived
by matching class names against pattern lists. The need they answer is real and
survives: without them a model that CANNOT read state is indistinguishable from
one that read it and was unsure, and the difference decides whether a low
confidence means "retake the picture" or "this model will never answer that".

It does not do geometry, does not read TF, does not write configs and does not
know what a region will be cleaned with. Everything after the pixels belongs to
`geometry`, `regions` and the caller — which is what makes a backend swappable
and testable against a directory of images with no robot present.

**Local is the default**, and that is structural rather than a config value:
`DEFAULT_BACKEND` is "local", and there is no automatic fall-through to a cloud
backend. butterfli's node fell back to Gemini whenever local weights were
missing, so a deployment with a broken model path silently became a deployment
that needed the network. On this robot the network is not guaranteed, so a
missing model is an error that says so.
"""

from __future__ import annotations

from typing import Callable, Protocol, runtime_checkable

from ..types import Detection, FixtureStatus

DEFAULT_BACKEND = "local"


class BackendError(RuntimeError):
    """A backend could not answer. Carries text fit for a service `message`."""


@runtime_checkable
class Backend(Protocol):
    """The interface `PerceptionCore` talks to."""

    name: str

    def check_status(self, image, fixture_hint: str = "") -> FixtureStatus:
        """Identify fixture type and state, and locate the fixture.

        `fixture_hint` is what the CALLER already knows — the task being run
        names its fixture, so there is no reason to make the model re-derive it
        from pixels and no reason to accept its answer over the task's. A hint
        that is not a known fixture type is ignored rather than trusted.

        Raise `BackendError` if the question cannot be answered. A
        `FixtureStatus` with no `fixture_type` means "looked, saw no fixture",
        which is a legitimate answer and must not look like a failure.
        """
        ...

    def detect_stains(self, image, status: FixtureStatus | None = None
                      ) -> list[Detection]:
        """Find stains in the image. Pixel coordinates of the image as given.

        `status` is the result of `check_status` on the SAME image. It carries
        the fixture box the region bands are measured against and the state that
        selects which band table applies. Passing None is allowed and means the
        regions cannot be decided — the detections come back unrouted rather
        than mapped against a stale or guessed box.

        An empty list means "looked, found nothing", which is a real and common
        answer. Failure raises.
        """
        ...

    # -- capabilities ---------------------------------------------------
    # Defaults are the conservative answer, so a backend that does not override
    # them is described as less capable than it might be rather than more.

    @property
    def supports_state_classification(self) -> bool:
        """Can this backend read fixture state, or does it fall back to a default?"""
        return False

    @property
    def supports_escalation(self) -> bool:
        """Can this backend recognise a condition needing a person?"""
        return False

    @property
    def supports_permanent_detection(self) -> bool:
        """Can this backend tell permanent discolouration from removable dirt?"""
        return False


#: name -> zero-argument factory. Registered rather than imported eagerly so that
#: a backend whose runtime is not installed costs nothing until it is asked for.
_REGISTRY: dict[str, Callable[..., Backend]] = {}


def register(name: str, factory: Callable[..., Backend]) -> None:
    _REGISTRY[name] = factory


def available() -> tuple[str, ...]:
    return tuple(sorted(_REGISTRY))


def create(name: str = DEFAULT_BACKEND, **kwargs) -> Backend:
    """Build a backend by name.

    Unknown names raise rather than falling back. A typo in a parameter should
    stop the node with a readable error, not quietly select something else and
    run for weeks.
    """
    if name not in _REGISTRY:
        raise BackendError(
            f"unknown inference backend '{name}'; available: "
            f"{', '.join(available()) or 'none'}")
    return _REGISTRY[name](**kwargs)
