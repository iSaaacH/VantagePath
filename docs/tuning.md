# Characterization and tuning

Tune from the inside out. Changing an earlier layer invalidates later gains.

## 1. Verify mechanics and units

Lift the drive. Positive voltage must increase both reported wheel distances.
Pushing straight must increase both equally. A counter-clockwise turn must
increase heading. Measure effective wheel circumference under robot load and
effective track width from repeated multi-turn tests, not CAD alone.

## 2. Identify each side's feedforward

Log voltage, wheel velocity, and wheel acceleration during slow quasistatic
ramps and faster steps. Fit `volts = kS*sign(v) + kV*v + kA*a` independently for
left and right. Validate on held-out runs. Use a voltage ceiling below nominal
while first testing.

## 3. Tune wheel velocity feedback

Start with `ki = kd = 0`. Increase `kp` until measured velocity follows steps
without chatter. Add only enough filtered `kd` to damp a repeatable overshoot.
Use `ki` only for a persistent loaded bias that feedforward did not capture;
set a tight `integralLimit`.

## 4. Establish conservative trajectory limits

Measure sustainable wheel speed and use 70–80% initially. Increase acceleration
until tracking error rises or wheels slip, then back off. Set deceleration
separately. Lower centripetal acceleration until tight turns no longer scrub or
tip. A good controller cannot recover traction that the plan already demanded.

## 5. Tune pose feedback

Keep damping around `0.7–0.9`. Raise `convergence` until injected 5–10 cm pose
errors recover promptly. If it snakes on straights, reduce convergence or fix
delayed/noisy localization. Keep heading tolerance realistic; demanding less
than sensor noise guarantees dithering.

## 6. Validate adversarially

Test low battery, added payload, reverse paths, starts offset in every direction,
tight S-curves, and endpoint disturbances. Log reference/measured pose, both
wheel targets/measurements, voltages, saturation, status, and actual loop `dt`.
Do not tune based only on the final pose.
