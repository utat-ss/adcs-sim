import { describe, expect, it } from "vitest";
import { EARTH_RADIUS_KM } from "../physics/constants";
import { DETUMBLE_RESIDUAL_RATE_DEG_S, attitudeForMode, detumbleAngularRateDegS, detumbleProgressForRate } from "../physics/attitude";
import { eclipseFactor } from "../physics/environment";
import { buildMissionTimeline, defaultMissionConfig } from "../physics/mission";
import { elementsToState, hohmannDeltaV, orbitalPeriodSeconds, stateToElements } from "../physics/orbit";
import { norm, scale } from "../physics/math";
import { sceneScaleInfo } from "../render/AdcsScene";

describe("orbital mechanics", () => {
  it("propagates a near-circular LEO orbit with stable radius", () => {
    const config = defaultMissionConfig();
    const state0 = elementsToState(config.startOrbit, 0, false);
    const stateQuarter = elementsToState(config.startOrbit, orbitalPeriodSeconds(config.startOrbit.semiMajorAxisKm) / 4, false);
    const maxNearCircularRadiusErrorKm = config.startOrbit.semiMajorAxisKm * config.startOrbit.eccentricity + 1;

    expect(Math.abs(norm(state0.positionKm) - config.startOrbit.semiMajorAxisKm)).toBeLessThan(maxNearCircularRadiusErrorKm);
    expect(Math.abs(norm(stateQuarter.positionKm) - config.startOrbit.semiMajorAxisKm)).toBeLessThan(maxNearCircularRadiusErrorKm);
    expect(norm(state0.velocityKmS)).toBeGreaterThan(7.0);
    expect(norm(state0.velocityKmS)).toBeLessThan(8.0);
  });

  it("round trips state vectors back into orbital elements", () => {
    const config = defaultMissionConfig();
    const state = elementsToState(config.startOrbit, 120, false);
    const elements = stateToElements(state, config.epochIso);

    expect(elements.semiMajorAxisKm).toBeCloseTo(config.startOrbit.semiMajorAxisKm, 3);
    expect(elements.eccentricity).toBeCloseTo(config.startOrbit.eccentricity, 3);
    expect(elements.inclinationDeg).toBeCloseTo(config.startOrbit.inclinationDeg, 2);
  });

  it("computes realistic Hohmann transfer effort for a 500 km to 650 km raise", () => {
    const dv = hohmannDeltaV(EARTH_RADIUS_KM + 500, EARTH_RADIUS_KM + 650);
    expect(dv.totalKmS).toBeGreaterThan(0.07);
    expect(dv.totalKmS).toBeLessThan(0.11);
    expect(dv.transferTimeSeconds).toBeGreaterThan(2800);
  });

  it("detects cylindrical Earth eclipse geometry", () => {
    const sun: [number, number, number] = [1, 0, 0];
    expect(eclipseFactor([EARTH_RADIUS_KM + 500, 0, 0], sun)).toBeCloseTo(1, 5);
    expect(eclipseFactor(scale(sun, -(EARTH_RADIUS_KM + 500)), sun)).toBeLessThan(0.05);
  });
});

describe("mission ADCS timeline", () => {
  it("builds samples, phases, eclipse state, and unit quaternions", () => {
    const config = defaultMissionConfig();
    const timeline = buildMissionTimeline(config, [], "unit-test");
    const sample = timeline.samples[20];

    expect(timeline.samples.length).toBe(config.sampleCount);
    expect(timeline.phases.length).toBeGreaterThan(4);
    expect(timeline.transferDeltaVKmS).toBeGreaterThan(0);
    expect(sample.altitudeKm).toBeGreaterThan(300);
    expect(Math.hypot(...sample.attitude.quaternion)).toBeCloseTo(1, 5);
  });

  it("points target-track boresight toward the target vector", () => {
    const config = defaultMissionConfig();
    const timeline = buildMissionTimeline(config, [], "unit-test");
    const sample = timeline.samples.find((item) => item.environment.targetVisible) ?? timeline.samples[0];
    const attitude = attitudeForMode("target-track", sample.state, sample.environment, sample.tSeconds, "target-track");

    expect(norm(attitude.boresightEci)).toBeCloseTo(1, 5);
    expect(attitude.mode).toBe("target-track");
  });

  it("detumbling decays body rate toward the residual rate floor", () => {
    const rate0 = detumbleAngularRateDegS(0);
    const rateMid = detumbleAngularRateDegS(2700);
    const rateLate = detumbleAngularRateDegS(7200);

    expect(rate0).toBeCloseTo(1.8, 5);
    expect(rateMid).toBeLessThan(0.1);
    expect(rateLate).toBeCloseTo(DETUMBLE_RESIDUAL_RATE_DEG_S, 5);
    expect(detumbleProgressForRate(rate0)).toBeCloseTo(0, 5);
    expect(detumbleProgressForRate(rateLate)).toBeCloseTo(1, 5);
  });

  it("keeps the rendered spacecraft smaller than low-orbit atmospheric clearance", () => {
    const config = defaultMissionConfig();
    const scale = sceneScaleInfo();
    const lowOrbitRadiusScene = scale.sceneRadiusForPhysicalRadiusKm(config.startOrbit.semiMajorAxisKm);
    const clearanceScene = lowOrbitRadiusScene - scale.atmosphereRadiusScene;

    expect(scale.satelliteVisualScale).toBeLessThan(0.2);
    expect(scale.orbitAltitudeVisualScale).toBeGreaterThan(1);
    expect(scale.satelliteMaxExtentScene).toBeLessThan(clearanceScene);
  });
});
