# Front Motor Off-Delay Sent With canset-current-rel

Status: Accepted.

## Context

When the throttle is released while a wheel is rolling, the script commands zero current with a 1 s off-delay, so the VESC keeps switching and the next throttle input engages smoothly ([powertrain](../powertrain.md#coasting)). The rear motor gets this with `set-current`. The front motor is commanded over CAN.

In VESC firmware 7.00, `canset-current` with an off-delay sends a packet whose fields are in a different order on each side: the sender packs the current, then the off-delay ([comm_can.c](../../third_party/vesc_firmware_v7/comm/comm_can.c) lines 523-530); the receiver reads the off-delay, then the current (lines 1613-1619). A 1.0 s off-delay arrives as a current of 1.0 A.

## Decision

Use `canset-current-rel` whenever the front motor needs an off-delay. Its packet has the relative current first and the off-delay second on both sides ([comm_can.c](../../third_party/vesc_firmware_v7/comm/comm_can.c): sender lines 576-583, receiver lines 1810-1816). Never call `canset-current` with three arguments ([AGENTS.md](../../AGENTS.md)).

## Alternatives Considered

- No off-delay on the front motor: the front would stop switching as soon as the throttle is released and re-engage less smoothly than the rear.
- Patching the VESC firmware: both VESCs run standard firmware, and a patched build would have to be maintained.

## Evidence

- VESC firmware source at the pinned version, as linked above.

## Consequences

- Front and rear coast the same way.
- If a later VESC firmware fixes the field order, the workaround still works; `canset-current-rel` is correct either way.
