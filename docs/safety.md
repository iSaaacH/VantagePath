# Robot validation checklist

Do not begin at full voltage. Put the robot on blocks for sign checks, then use
a clear field, an accessible disable control, and a conservative voltage cap.

- [ ] Left/right voltage signs and encoder signs agree.
- [ ] Heading convention is radians and counter-clockwise positive.
- [ ] Length units match across poses, encoders, track width, and limits.
- [ ] The control loop is periodic and measured `dt` stays below 100 ms.
- [ ] Wheel circumference and effective track width are measured.
- [ ] Feedforward is identified independently for both sides.
- [ ] Wheel PID is stable before pose feedback is enabled.
- [ ] Divergence limit stops a deliberately incorrect starting pose.
- [ ] Timeout and operator cancellation command zero volts.
- [ ] Low-battery saturation preserves left/right ratio.
- [ ] Straight, curved, reverse, S-curve, and short paths pass repeatedly.
- [ ] Payload and intentional starting offsets do not cause oscillation.
- [ ] Logs show reference and measured pose/wheel speeds/voltage/status.
