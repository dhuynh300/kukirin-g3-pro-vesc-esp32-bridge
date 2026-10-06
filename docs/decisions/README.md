# Decision Records

Why the system is built the way it is, and what was rejected. Each record has the context, the decision, the alternatives, the evidence and the consequences.

| Record | Decision |
|---|---|
| [0001](0001-display-frame-timing.md) | Display link timeouts sized for about 8 frames per second |
| [0002](0002-control-frame-dispatch.md) | Control frames sent on each display frame and every 20 ms |
| [0003](0003-current-braking-not-handbrake.md) | Electric braking uses VESC current braking, not the handbrake mode |
| [0004](0004-zero-current-below-regen-floor.md) | Zero current below the regen floor instead of brake mode |
| [0005](0005-can-off-delay-workaround.md) | Front motor off-delay sent with canset-current-rel |
| [0006](0006-motor-control-in-the-vesc-script.md) | Control that must survive a lost ESP32 link runs in the VESC script |
| [0007](0007-relative-current-commands.md) | Current commands relative to the VESC Tool limits |
| [0008](0008-public-and-private-repositories.md) | Released firmware on main, development in a private repository |
