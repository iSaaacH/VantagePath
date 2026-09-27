// A JavaScript port of the geometry and speed passes in src/trajectory.cpp, so
// Studio previews the same stops, slow-downs and timing the robot will run.
// The voltage constraint is not ported: Studio has no feedforward gains, and
// without them the C++ generator skips that pass too.
//
// Units follow the document: inches, seconds, radians CCW from +X.

// Kept local so model.js can import this module without a cycle.
export function wrapRadians(angle) {
  let wrapped = angle;
  while (wrapped > Math.PI) wrapped -= Math.PI * 2;
  while (wrapped <= -Math.PI) wrapped += Math.PI * 2;
  return wrapped;
}
// The C++ generator rejects joins whose tangent jumps by more than this.
export const JOIN_TOLERANCE = 1e-3;
const MIN_SEGMENT_SAMPLES = 16;

function bezierValue(points, t) {
  const work = points.map(({ x, y }) => ({ x, y }));
  for (let size = work.length - 1; size > 0; size -= 1) {
    for (let i = 0; i < size; i += 1) {
      work[i].x += (work[i + 1].x - work[i].x) * t;
      work[i].y += (work[i + 1].y - work[i].y) * t;
    }
  }
  return work[0];
}

function bezierDerivative(points) {
  const degree = points.length - 1;
  const result = [];
  for (let i = 1; i < points.length; i += 1) {
    result.push({ x: degree * (points[i].x - points[i - 1].x), y: degree * (points[i].y - points[i - 1].y) });
  }
  return result.length ? result : [{ x: 0, y: 0 }];
}

function quintic(p0, v0, p1, v1) {
  const a3 = -10 * p0 - 6 * v0 + 10 * p1 - 4 * v1;
  const a4 = 15 * p0 + 8 * v0 - 15 * p1 + 7 * v1;
  const a5 = -6 * p0 - 3 * v0 + 6 * p1 - 3 * v1;
  return {
    value: (t) => p0 + t * (v0 + t * t * (a3 + t * (a4 + t * a5))),
    first: (t) => v0 + t * t * (3 * a3 + t * (4 * a4 + t * 5 * a5)),
    second: (t) => t * (6 * a3 + t * (12 * a4 + t * 20 * a5)),
  };
}

/**
 * Geometry evaluator for one segment in the direction the C++ generator builds
 * it. Reversed segments use nose headings rotated by π, exactly as the export
 * writes them, so a reversed Hermite segment backs out of its waypoint.
 */
export function segmentGeometry(path, index) {
  const start = path.waypoints[index];
  const end = path.waypoints[index + 1];
  const controls = path.controlPoints?.[index];
  if (Array.isArray(controls)) {
    const polygon = [start, ...controls, end].map(({ x, y }) => ({ x, y }));
    const first = bezierDerivative(polygon);
    const second = bezierDerivative(first);
    const polygonLength = polygon.slice(1).reduce((sum, point, i) => sum + Math.hypot(point.x - polygon[i].x, point.y - polygon[i].y), 0);
    return {
      bezier: true,
      polygon,
      nominalLength: polygonLength,
      position: (t) => bezierValue(polygon, t),
      velocity: (t) => bezierValue(first, t),
      acceleration: (t) => bezierValue(second, t),
    };
  }
  const flip = path.segmentReversed?.[index] === true ? Math.PI : 0;
  const startHeading = start.heading + flip;
  const endHeading = end.heading + flip;
  const x = quintic(start.x, Math.cos(startHeading) * start.tangent, end.x, Math.cos(endHeading) * end.tangent);
  const y = quintic(start.y, Math.sin(startHeading) * start.tangent, end.y, Math.sin(endHeading) * end.tangent);
  return {
    bezier: false,
    nominalLength: Math.hypot(end.x - start.x, end.y - start.y),
    position: (t) => ({ x: x.value(t), y: y.value(t) }),
    velocity: (t) => ({ x: x.first(t), y: y.first(t) }),
    acceleration: (t) => ({ x: x.second(t), y: y.second(t) }),
  };
}

/**
 * Travel direction of the geometry at one end of a segment (end = 0 or 1).
 * Falls back to the first distinct control when the derivative vanishes, the
 * same limiting tangent the C++ generator uses for repeated controls.
 */
export function segmentTangent(path, index, end) {
  const geometry = segmentGeometry(path, index);
  const derivative = geometry.velocity(end);
  if (Math.hypot(derivative.x, derivative.y) > 1e-9) return Math.atan2(derivative.y, derivative.x);
  if (geometry.bezier) {
    const points = end === 0 ? geometry.polygon : [...geometry.polygon].reverse();
    for (let i = 1; i < points.length; i += 1) {
      const dx = points[i].x - points[0].x;
      const dy = points[i].y - points[0].y;
      if (Math.hypot(dx, dy) > 1e-9) return end === 0 ? Math.atan2(dy, dx) : Math.atan2(-dy, -dx);
    }
  }
  const a = path.waypoints[index];
  const b = path.waypoints[index + 1];
  return Math.atan2(b.y - a.y, b.x - a.x);
}

/** Direction the robot's nose points at one end of a segment. */
export function segmentNoseHeading(path, index, end) {
  const tangent = segmentTangent(path, index, end);
  return wrapRadians(path.segmentReversed?.[index] === true ? tangent + Math.PI : tangent);
}

/**
 * Splits a route into sections the robot drives without stopping to turn:
 * a new section starts at every drive-direction change and at every join the
 * C++ generator would reject as not tangent-continuous. Each section becomes
 * one exported trajectory, so the export never throws at startup.
 */
export function routeSections(path) {
  const count = path.waypoints.length - 1;
  if (count < 1) return [];
  const sections = [];
  let first = 0;
  for (let segment = 1; segment <= count; segment += 1) {
    const atEnd = segment === count;
    let turn = 0;
    let kink = false;
    let directionChange = false;
    if (!atEnd) {
      directionChange = path.segmentReversed[segment] !== path.segmentReversed[segment - 1];
      turn = wrapRadians(segmentNoseHeading(path, segment, 0) - segmentNoseHeading(path, segment - 1, 1));
      const tangentJump = wrapRadians(segmentTangent(path, segment, 0) - segmentTangent(path, segment - 1, 1));
      kink = !directionChange && Math.abs(tangentJump) > JOIN_TOLERANCE;
    }
    if (atEnd || directionChange || kink) {
      sections.push({
        firstSegment: first,
        lastSegment: segment - 1,
        reversed: path.segmentReversed[first] === true,
        // How the robot must turn in place before the NEXT section starts.
        turnAfter: atEnd ? 0 : turn,
        endsAtKink: kink,
        endsAtDirectionChange: directionChange,
      });
      first = segment;
    }
  }
  return sections;
}

function sampleSection(path, section, sampleDistance) {
  const samples = [];
  const stopIndices = [];
  let arcLength = 0;
  for (let segment = section.firstSegment; segment <= section.lastSegment; segment += 1) {
    const isBezierJoin = segment > section.firstSegment &&
      (Array.isArray(path.controlPoints[segment - 1]) || Array.isArray(path.controlPoints[segment]));
    if (isBezierJoin) stopIndices.push(samples.length - 1);
    const a = path.waypoints[segment];
    const b = path.waypoints[segment + 1];
    if (Math.hypot(b.x - a.x, b.y - a.y) < 1e-8) throw new Error(`P${segment + 1} and P${segment + 2} are on top of each other`);
    const geometry = segmentGeometry(path, segment);
    const count = Math.max(MIN_SEGMENT_SAMPLES, Math.ceil(geometry.nominalLength / sampleDistance * 2));
    for (let i = segment > section.firstSegment ? 1 : 0; i <= count; i += 1) {
      const t = i / count;
      const position = geometry.position(t);
      const velocity = geometry.velocity(t);
      const acceleration = geometry.acceleration(t);
      const denominator = Math.pow(velocity.x ** 2 + velocity.y ** 2, 1.5);
      const curvature = denominator > 1e-12 ? (velocity.x * acceleration.y - velocity.y * acceleration.x) / denominator : 0;
      let heading = Math.atan2(velocity.y, velocity.x);
      if (Math.hypot(velocity.x, velocity.y) < 1e-9 && (i === 0 || i === count)) heading = segmentTangent(path, segment, i === 0 ? 0 : 1);
      if (samples.length) arcLength += Math.hypot(position.x - samples.at(-1).x, position.y - samples.at(-1).y);
      samples.push({ x: position.x, y: position.y, heading, curvature, distance: arcLength, segmentIndex: segment });
    }
  }
  return { samples, stopIndices };
}

/**
 * Per-segment speed ceilings for one section as the library's pathSpeedScale:
 * a step function of arc-length percent. Boundaries sit midway between the
 * join sample and the next sample, so no sample lies on one and JS and C++
 * classify every sample the same way. Returns null when nothing is limited.
 */
export function sectionSpeedScale(path, section, samples, maxVelocity) {
  const speeds = path.segmentSpeed ?? [];
  let limited = false;
  const steps = [];
  for (let segment = section.firstSegment; segment <= section.lastSegment; segment += 1) {
    const speed = speeds[segment];
    const scale = Number.isFinite(speed) && speed > 0 ? Math.min(1, speed / maxVelocity) : 1;
    if (scale < 1) limited = true;
    steps.push({ segment, scale });
  }
  if (!limited) return null;
  const length = samples.at(-1).distance;
  const percent = (i) => (length > 1e-9 ? samples[i].distance / length * 100 : 100);
  const boundaries = [];
  for (let i = 1; i < samples.length; i += 1) {
    if (samples[i].segmentIndex !== samples[i - 1].segmentIndex) {
      // samples[i - 1] is the join (last sample of the earlier segment).
      boundaries.push(Number(((percent(i - 1) + percent(i)) / 2).toFixed(9)));
    }
  }
  return { boundaries, scales: steps.map((step) => Number(step.scale.toFixed(9))) };
}

export function evaluateSpeedScale(spec, percent) {
  const index = spec.boundaries.findIndex((boundary) => percent < boundary);
  return spec.scales[index < 0 ? spec.scales.length - 1 : index];
}

function planVelocities(samples, stopIndices, config) {
  const scaleSpec = config.speedScale;
  const length = samples.at(-1).distance;
  const scales = scaleSpec ? samples.map((sample) => evaluateSpeedScale(scaleSpec, length > 1e-9 ? sample.distance / length * 100 : 100)) : null;
  const velocity = samples.map((sample, i) => {
    const curvature = Math.abs(sample.curvature);
    const curvatureBound = Math.max(curvature, Math.abs(samples[Math.max(0, i - 1)].curvature), Math.abs(samples[Math.min(i + 1, samples.length - 1)].curvature));
    // Lower the preceding sample too, bounding interpolation at a step down.
    const ceiling = scales ? config.maxVelocity * Math.min(scales[i], scales[Math.min(i + 1, samples.length - 1)]) : config.maxVelocity;
    let limit = Math.min(ceiling, config.maxWheelVelocity / (1 + curvatureBound * config.trackWidth * 0.5));
    if (curvature > 1e-9) limit = Math.min(limit, Math.sqrt(config.maxCentripetalAcceleration / curvature));
    return limit;
  });
  for (const index of stopIndices) velocity[index] = 0;
  velocity[0] = Math.min(velocity[0], config.startVelocity);
  for (let i = 1; i < velocity.length; i += 1) {
    const ds = samples[i].distance - samples[i - 1].distance;
    velocity[i] = Math.min(velocity[i], Math.sqrt(velocity[i - 1] ** 2 + 2 * config.maxAcceleration * ds));
  }
  velocity[velocity.length - 1] = Math.min(velocity.at(-1), config.endVelocity);
  for (let i = velocity.length - 2; i >= 0; i -= 1) {
    const ds = samples[i + 1].distance - samples[i].distance;
    velocity[i] = Math.min(velocity[i], Math.sqrt(velocity[i + 1] ** 2 + 2 * config.maxDeceleration * ds));
  }
  return velocity;
}

/** Plans one section. Returns time-stamped states in document units. */
export function planSection(path, section, config) {
  const { samples, stopIndices } = sampleSection(path, section, config.sampleDistance);
  const speedScale = sectionSpeedScale(path, section, samples, config.maxVelocity);
  const velocity = planVelocities(samples, stopIndices, { ...config, speedScale });
  let time = 0;
  const states = samples.map((sample, i) => {
    if (i > 0) {
      const ds = sample.distance - samples[i - 1].distance;
      const sum = velocity[i] + velocity[i - 1];
      if (ds > 1e-9 && sum <= 1e-9) throw new Error("The route has a stretch the robot can never drive (zero speed)");
      time += sum > 1e-9 ? 2 * ds / sum : 0;
    }
    const heading = section.reversed ? wrapRadians(sample.heading + Math.PI) : wrapRadians(sample.heading);
    return { ...sample, heading, velocity: velocity[i], time };
  });
  const stops = stopIndices.map((index) => ({ x: samples[index].x, y: samples[index].y, waypointIndex: samples[index].segmentIndex, reason: "bezier-join" }));
  return { states, stops, duration: time, length: samples.at(-1).distance, speedScale };
}

/** Time for a point turn of `angle` radians using the robot's wheel limits. */
export function turnDuration(angle, robot) {
  const radius = Math.max(0.1, robot.trackWidth / 2);
  const maxRate = robot.maxWheelVelocity / radius;
  const maxAcceleration = Math.min(robot.maxAcceleration, robot.maxDeceleration) / radius;
  const distance = Math.abs(angle);
  if (distance < 1e-9) return 0;
  const rampDistance = maxRate * maxRate / maxAcceleration;
  if (distance <= rampDistance) return 2 * Math.sqrt(distance / maxAcceleration);
  return 2 * maxRate / maxAcceleration + (distance - rampDistance) / maxRate;
}

export function trajectoryConfig(robot, section, sectionIndex, sectionCount) {
  return {
    maxVelocity: robot.maxVelocity,
    maxAcceleration: robot.maxAcceleration,
    maxDeceleration: robot.maxDeceleration,
    maxCentripetalAcceleration: robot.maxCentripetalAcceleration,
    maxWheelVelocity: robot.maxWheelVelocity,
    trackWidth: robot.trackWidth,
    sampleDistance: robot.sampleDistance,
    startVelocity: sectionIndex === 0 ? robot.startVelocity : 0,
    endVelocity: sectionIndex === sectionCount - 1 ? robot.endVelocity : 0,
    reversed: section.reversed,
  };
}

/**
 * Plans a whole route as the robot will run it: each section as a trajectory,
 * with a point turn wherever the heading jumps between sections.
 * Returns { steps, duration, length, stops, error }.
 */
export function planRoute(path, robot) {
  const empty = { steps: [], duration: 0, length: 0, stops: [], error: null };
  if (!path || path.waypoints.length < 2) return empty;
  let sections;
  try {
    sections = routeSections(path);
  } catch (error) {
    return { ...empty, error: error.message };
  }
  const steps = [];
  const stops = [];
  let time = 0;
  let length = 0;
  try {
    sections.forEach((section, index) => {
      const plan = planSection(path, section, trajectoryConfig(robot, section, index, sections.length));
      steps.push({ type: "drive", section, startTime: time, startDistance: length, ...plan });
      stops.push(...plan.stops);
      time += plan.duration;
      length += plan.length;
      if (index === sections.length - 1) return;
      const joint = path.waypoints[section.lastSegment + 1];
      stops.push({ x: joint.x, y: joint.y, waypointIndex: section.lastSegment + 1, reason: section.endsAtKink ? "corner" : "direction-change", turn: section.turnAfter });
      const endHeading = plan.states.at(-1).heading;
      const duration = turnDuration(section.turnAfter, robot);
      if (duration > 0) {
        steps.push({ type: "turn", startTime: time, startDistance: length, duration, x: joint.x, y: joint.y, fromHeading: endHeading, angle: section.turnAfter, waypointIndex: section.lastSegment + 1 });
        time += duration;
      }
    });
  } catch (error) {
    return { ...empty, error: error.message };
  }
  return { steps, duration: time, length, stops, error: null };
}

function stepAtTime(plan, time) {
  let found = plan.steps[0];
  for (const step of plan.steps) if (step.startTime <= time + 1e-12) found = step;
  return found;
}

/** Interpolated robot pose at a playback time. */
export function poseAtTime(plan, time) {
  if (!plan.steps.length) return null;
  const t = Math.min(plan.duration, Math.max(0, time));
  const step = stepAtTime(plan, t);
  const local = t - step.startTime;
  if (step.type === "turn") {
    const fraction = step.duration > 0 ? Math.min(1, local / step.duration) : 1;
    return { x: step.x, y: step.y, heading: wrapRadians(step.fromHeading + step.angle * fraction), velocity: 0, distance: step.startDistance };
  }
  const states = step.states;
  let upper = states.findIndex((state) => state.time >= local);
  if (upper < 0) upper = states.length - 1;
  const lower = states[Math.max(0, upper - 1)];
  const next = states[upper];
  const span = next.time - lower.time;
  const u = span > 1e-12 ? (local - lower.time) / span : 0;
  return {
    x: lower.x + (next.x - lower.x) * u,
    y: lower.y + (next.y - lower.y) * u,
    heading: wrapRadians(lower.heading + wrapRadians(next.heading - lower.heading) * u),
    velocity: lower.velocity + (next.velocity - lower.velocity) * u,
    distance: step.startDistance + lower.distance + (next.distance - lower.distance) * u,
  };
}

/** All driven states along the route in route distance, for scrubbing. */
export function routeSamples(plan) {
  return plan.steps.filter((step) => step.type === "drive").flatMap((step) =>
    step.states.map((state) => ({ x: state.x, y: state.y, distance: step.startDistance + state.distance, time: step.startTime + state.time })));
}

/** Playback time at a route distance (the first moment the robot is there). */
export function timeAtDistance(plan, distance) {
  const samples = routeSamples(plan);
  if (!samples.length) return 0;
  const upper = samples.findIndex((sample) => sample.distance >= distance);
  if (upper <= 0) return upper === 0 ? samples[0].time : samples.at(-1).time;
  const a = samples[upper - 1];
  const b = samples[upper];
  const span = b.distance - a.distance;
  return a.time + (span > 1e-12 ? (distance - a.distance) / span : 0) * (b.time - a.time);
}
