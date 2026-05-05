import type { Quat, Vec3 } from "../types";

export const degToRad = (deg: number) => (deg * Math.PI) / 180;
export const radToDeg = (rad: number) => (rad * 180) / Math.PI;

export function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

export function smoothstep(edge0: number, edge1: number, x: number): number {
  const t = clamp((x - edge0) / (edge1 - edge0), 0, 1);
  return t * t * (3 - 2 * t);
}

export function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

export function scale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

export function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

export function cross(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]
  ];
}

export function norm(a: Vec3): number {
  return Math.hypot(a[0], a[1], a[2]);
}

export function normalize(a: Vec3, fallback: Vec3 = [1, 0, 0]): Vec3 {
  const n = norm(a);
  if (n < 1e-12 || !Number.isFinite(n)) return fallback;
  return scale(a, 1 / n);
}

export function angleBetweenDeg(a: Vec3, b: Vec3): number {
  const an = normalize(a);
  const bn = normalize(b);
  return radToDeg(Math.acos(clamp(dot(an, bn), -1, 1)));
}

export function rotateZ(v: Vec3, angleRad: number): Vec3 {
  const c = Math.cos(angleRad);
  const s = Math.sin(angleRad);
  return [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]];
}

export function rotateX(v: Vec3, angleRad: number): Vec3 {
  const c = Math.cos(angleRad);
  const s = Math.sin(angleRad);
  return [v[0], c * v[1] - s * v[2], s * v[1] + c * v[2]];
}

export function wrapDegrees(deg: number): number {
  return ((deg % 360) + 360) % 360;
}

export function signedAngleDeltaDeg(fromDeg: number, toDeg: number): number {
  const d = wrapDegrees(toDeg - fromDeg + 180) - 180;
  return d === -180 ? 180 : d;
}

export function interpolateAngleDeg(fromDeg: number, toDeg: number, t: number): number {
  return wrapDegrees(fromDeg + signedAngleDeltaDeg(fromDeg, toDeg) * t);
}

export function quaternionFromBasis(xAxis: Vec3, yAxis: Vec3, zAxis: Vec3): Quat {
  const m00 = xAxis[0];
  const m01 = yAxis[0];
  const m02 = zAxis[0];
  const m10 = xAxis[1];
  const m11 = yAxis[1];
  const m12 = zAxis[1];
  const m20 = xAxis[2];
  const m21 = yAxis[2];
  const m22 = zAxis[2];
  const trace = m00 + m11 + m22;
  let qx: number;
  let qy: number;
  let qz: number;
  let qw: number;

  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    qw = 0.25 * s;
    qx = (m21 - m12) / s;
    qy = (m02 - m20) / s;
    qz = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    qw = (m21 - m12) / s;
    qx = 0.25 * s;
    qy = (m01 + m10) / s;
    qz = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    qw = (m02 - m20) / s;
    qx = (m01 + m10) / s;
    qy = 0.25 * s;
    qz = (m12 + m21) / s;
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    qw = (m10 - m01) / s;
    qx = (m02 + m20) / s;
    qy = (m12 + m21) / s;
    qz = 0.25 * s;
  }

  return normalizeQuat([qw, qx, qy, qz]);
}

export function normalizeQuat(q: Quat): Quat {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (n < 1e-12) return [1, 0, 0, 0];
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

export function buildRightHandedBasis(primaryZ: Vec3, preferredX: Vec3): { x: Vec3; y: Vec3; z: Vec3 } {
  const z = normalize(primaryZ, [0, 0, 1]);
  const projectedX = sub(preferredX, scale(z, dot(preferredX, z)));
  const x = normalize(projectedX, Math.abs(z[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0]);
  const y = normalize(cross(z, x), [0, 1, 0]);
  return { x, y, z };
}
