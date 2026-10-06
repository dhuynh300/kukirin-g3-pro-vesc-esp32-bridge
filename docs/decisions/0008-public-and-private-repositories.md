# Released Firmware On main, Development In A Private Repository

Status: Accepted.

## Context

This firmware controls a vehicle. The public repository should only contain firmware that runs on the scooter, while development needs a place for unfinished and untested changes.

## Decision

- The public repository has `main` and release tags only.
- A private repository mirrors `main` and adds `lab` (development) and `field` (exactly what is flashed on the scooter).
- A change moves from `lab` to a bench test, to `field`, to a ride, and then into `main` as one pull request with its test report linked, followed by a release tag.

## Alternatives Considered

- One public repository with all branches: unfinished and unridden changes would be visible next to released firmware.
- Releasing straight from development: a release would not be tied to a ride.

## Evidence

- Process decision; the test reports linked from each release are the evidence that the flow was followed.

## Consequences

- Every firmware change on `main` after the first release was bench tested and ridden.
- Work in progress is not visible publicly until it is released.
