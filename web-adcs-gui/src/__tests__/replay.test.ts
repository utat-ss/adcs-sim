import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import * as THREE from "three";
import { defaultMissionConfig } from "../physics/mission";
import { norm } from "../physics/math";
import { parseReplayJson, qEciToBodyToBodyInEci, replayToMissionTimeline, rotateByScalarFirstQuat } from "../data/replay";
import { quaternionFromEciBodyAxes } from "../render/AdcsScene";

describe("MATLAB replay import", () => {
  it("preserves epoch, units and unsampled mode events from an actual 300-second MATLAB export", () => {
    const raw = JSON.parse(readFileSync(new URL("./fixtures/matlab-300s-subset.json", import.meta.url), "utf8"));
    const replay = parseReplayJson(JSON.stringify(raw));
    const timeline = replayToMissionTimeline(replay, defaultMissionConfig());
    expect(timeline.config.epochIso).toBe("2026-02-12T00:00:00.000Z");
    expect(timeline.samples[timeline.samples.length-1].date.toISOString()).toBe("2026-02-12T00:05:00.000Z");
    expect(timeline.samples[0].altitudeKm).toBeCloseTo(499.3129, 6);
    expect(timeline.samples[0].state.velocityKmS).toEqual(raw.samples[0].velocityEciKmps);
    expect(timeline.samples[0].replay?.rwTorqueBodyNm).toEqual(raw.samples[0].reactionWheelTorqueBodyNm);
    expect(timeline.samples[0].replay?.rwTauWheelsNm).toEqual(raw.samples[0].reactionWheelTorqueWheelNm);
    expect(timeline.events).toHaveLength(raw.events.modeChanges.length);
    expect(timeline.events?.[1]).toEqual({ tSec: 269.38, kind: "mode", label: "coarse" });
    expect(timeline.samples.some((s) => s.tSeconds === 269.38)).toBe(false);
    raw.samples = raw.samples.slice(1);
    const cropped = replayToMissionTimeline(parseReplayJson(JSON.stringify(raw)), defaultMissionConfig());
    expect(cropped.config.epochIso).toBe("2026-02-12T00:02:30.000Z");
    expect(cropped.samples[1].date.toISOString()).toBe("2026-02-12T00:05:00.000Z");
    raw.events.modeChanges = { tSec: 0, modeId: 1 };
    expect(parseReplayJson(JSON.stringify(raw)).events).toEqual([{ tSec: 0, kind: "mode", label: "detumble" }]);
    raw.events.modeChanges = [];
    expect(parseReplayJson(JSON.stringify(raw)).events).toEqual([]);
  });

  it("does not report a wheel command as delivered torque", () => {
    const raw = JSON.parse(readFileSync(new URL("./fixtures/matlab-300s-subset.json", import.meta.url), "utf8"));
    raw.samples[0].reactionWheelCommandNm = [0.01, 0.02, 0.03];
    raw.samples[0].reactionWheelTorqueBodyNm = [0.001, 0.002, 0.003];
    let timeline = replayToMissionTimeline(parseReplayJson(JSON.stringify(raw)), defaultMissionConfig());
    expect(timeline.samples[0].replay?.rwTorqueBodyNm).toEqual([0.001, 0.002, 0.003]);
    expect(timeline.samples[0].replay?.reactionWheelCommandNm).toEqual([0.01, 0.02, 0.03]);
    delete raw.samples[0].reactionWheelTorqueBodyNm;
    timeline = replayToMissionTimeline(parseReplayJson(JSON.stringify(raw)), defaultMissionConfig());
    expect(timeline.samples[0].replay?.rwTorqueBodyNm).toBeUndefined();
  });
  it("keeps scalar-first passive quaternion numerics and derives body axes in ECI", () => {
    const q = [Math.SQRT1_2, 0, 0, Math.SQRT1_2] as [number, number, number, number];
    const renderQ = qEciToBodyToBodyInEci(q);

    expect(renderQ[0]).toBeCloseTo(q[0], 12);
    expect(renderQ[3]).toBeCloseTo(q[3], 12);
    expect(rotateByScalarFirstQuat(renderQ, [1, 0, 0])[0]).toBeCloseTo(0, 12);
    expect(rotateByScalarFirstQuat(renderQ, [1, 0, 0])[1]).toBeCloseTo(1, 12);
  });

  it("maps ECI body axes into the Three.js scene frame before orienting the spacecraft", () => {
    const qScene = quaternionFromEciBodyAxes([0, 1, 0], [-1, 0, 0], [0, 0, 1]);
    const localBodyX = new THREE.Vector3(1, 0, 0).applyQuaternion(qScene);
    const localBodyY = new THREE.Vector3(0, 1, 0).applyQuaternion(qScene);
    const localBodyZ = new THREE.Vector3(0, 0, 1).applyQuaternion(qScene);

    expect(localBodyX.x).toBeCloseTo(0, 12);
    expect(localBodyX.y).toBeCloseTo(0, 12);
    expect(localBodyX.z).toBeCloseTo(-1, 12);
    expect(localBodyY.x).toBeCloseTo(-1, 12);
    expect(localBodyY.y).toBeCloseTo(0, 12);
    expect(localBodyY.z).toBeCloseTo(0, 12);
    expect(localBodyZ.x).toBeCloseTo(0, 12);
    expect(localBodyZ.y).toBeCloseTo(1, 12);
    expect(localBodyZ.z).toBeCloseTo(0, 12);
  });

  it("parses the agreed samples schema with provenance and logged actuator telemetry", () => {
    const replay = parseReplayJson(
      JSON.stringify({
        schema: "utat.adcs.replay.v1",
        provenance: { source: "MATLAB/Simulink", model: "adcs_sim", matlabVersion: "R2026a" },
        frames: { attitudeQuaternion: "scalar-first ECI-to-body", units: { position: "km", velocity: "km/s" } },
        constants: { earthRadiusKm: 6371 },
        spacecraft: { cameraHalfAngleDeg: 8, sensorBoresightsBody: [{ id: "camera", kind: "conic", axisBody: [0, 0, 1] }] },
        samples: [
          {
            tSec: 0,
            utc: "2026-06-01T12:00:00.000Z",
            positionEciKm: [6871, 0, 0],
            velocityEciKmS: [0, 7.61, 0],
            qEciToBody: [1, 0, 0, 0],
            omegaBodyRadS: [0.01, 0, 0],
            wheelMomentumNms: [0.1, 0, 0, 0],
            rwTorqueBodyNm: [0.001, 0, 0],
            bEciT: [0, 0, 0.00003],
            mode: "detumble",
            quality: { attitudeValid: true, starTrackerValid: false, sampleSource: "unit" }
          },
          {
            tSec: 10,
            utc: "2026-06-01T12:00:10.000Z",
            positionEciKm: [6870.6, 76.1, 0],
            velocityEciKmS: [-0.08, 7.61, 0],
            qEciToBody: [Math.SQRT1_2, 0, 0, Math.SQRT1_2],
            omegaBodyRadS: [0.009, 0, 0],
            wheelMomentumNms: [0.11, 0, 0, 0],
            mode: "target-track",
            quality: { attitudeValid: true, starTrackerValid: true, sampleSource: "unit" }
          }
        ]
      })
    );

    const timeline = replayToMissionTimeline(replay, defaultMissionConfig());
    expect(timeline.sourceKind).toBe("replay");
    expect(timeline.samples).toHaveLength(2);
    expect(timeline.constants?.earthRadiusKm).toBe(6371);
    expect(timeline.samples[0].altitudeKm).toBeCloseTo(500, 3);
    expect(timeline.samples[0].replay?.rwTorqueBodyNm?.[0]).toBeCloseTo(0.001);
    expect(timeline.samples[0].replay?.starTrackerValid).toBe(false);
    expect(norm(timeline.samples[1].attitude.bodyX)).toBeCloseTo(1, 12);
  });

  it("converts the current legacy timeseries exporter draft into samples", () => {
    const legacy = parseReplayJson(
      JSON.stringify({
        metadata: { schema: "adcs-simulink-replay-v1", model: "adcs_sim", duration_s: 5 },
        parameters: { Re: 6371000 },
        timeseries: {
          q: { time_s: [0, 5], values: [[1, 0, 0, 0], [1, 0, 0, 0]] },
          pos_eci_m: { time_s: [0, 5], values: [[6871000, 0, 0], [6870000, 38050, 0]] },
          vel_eci_mps: { time_s: [0, 5], values: [[0, 7610, 0], [-40, 7610, 0]] },
          omega_rad_s: { time_s: [0, 5], values: [[0.01, 0, 0], [0.009, 0, 0]] },
          h_w_nms: { time_s: [0, 5], values: [[0, 0, 0, 0], [0.01, 0, 0, 0]] },
          mode_id: { time_s: [0, 5], values: [1, 3] },
          att_valid: { time_s: [0, 5], values: [1, 1] }
        }
      })
    );

    expect(legacy.schema).toBe("utat.adcs.replay.v1");
    expect(legacy.constants?.earthRadiusKm).toBe(6371);
    expect(legacy.samples[0].positionEciKm[0]).toBeCloseTo(6871);
    expect(legacy.samples[1].mode).toBe("fine");
    expect(legacy.samples[1].wheelMomentumNms?.[0]).toBeCloseTo(0.01);
  });

  it("accepts the current integration root schema field names without dropping telemetry", () => {
    const replay = parseReplayJson(
      JSON.stringify({
        schema: "utat.adcs.replay.v1",
        provenance: { source: "MATLAB/Simulink exported by export_adcs_results.m", model: "adcs_sim", generatedUtc: "2026-09-21T12:00:00Z" },
        centralBody: { name: "Earth", radiusKm: 6371 },
        simulation: {
          dtSec: 0.1,
          attitudeQuaternionConvention: "scalar-first passive ECI-to-body",
          positionFrame: "ECI",
          positionUnits: "km"
        },
        visualization: { cameraBoresightBody: [0, 0, 1], cameraFovHalfDeg: 8 },
        samples: [
          {
            tSec: 0,
            positionEciKm: [6871, 0, 0],
            velocityEciKmps: [0, 7.61, 0],
            qEciToBody: [1, 0, 0, 0],
            omegaBodyRadS: [0.01, 0, 0],
            magneticFieldEciT: [0, 0, 0.00003],
            trackerValid: 0,
            attitudeValid: 1,
            modeId: 1,
            reactionWheelMomentumNms: [0.01, 0.02, 0.03, 0.04],
            reactionWheelCommandNm: [0.001, 0.002, 0.003],
            magnetorquerCommandAm2: [0.1, 0.2, 0.3]
          },
          {
            tSec: 5,
            positionEciKm: [6870, 38.05, 0],
            velocityEciKmps: [-0.04, 7.61, 0],
            qEciToBody: [1, 0, 0, 0],
            omegaBodyRadS: [0.009, 0, 0],
            magneticFieldEciT: [0, 0, 0.00003],
            trackerValid: 1,
            attitudeValid: 1,
            modeId: 3,
            reactionWheelMomentumNms: [0.02, 0.02, 0.03, 0.04],
            reactionWheelCommandNm: [0.004, 0.005, 0.006],
            magnetorquerCommandAm2: [0.4, 0.5, 0.6]
          }
        ]
      })
    );
    const timeline = replayToMissionTimeline(replay, defaultMissionConfig());

    expect(timeline.constants?.earthRadiusKm).toBe(6371);
    expect(timeline.provenance?.generatedAtUtc).toBe("2026-09-21T12:00:00Z");
    expect(timeline.samples[0].phase).toBe("Detumble");
    expect(timeline.samples[0].replay?.starTrackerValid).toBe(false);
    expect(timeline.samples[0].replay?.reactionWheelCommandNm?.[2]).toBeCloseTo(0.003);
    expect(timeline.samples[0].replay?.rwTorqueBodyNm).toBeUndefined();
    expect(timeline.samples[0].replay?.mtqDipoleAm2?.[1]).toBeCloseTo(0.2);
    expect(timeline.samples[0].replay?.wheelMomentumNms?.[3]).toBeCloseTo(0.04);
  });
});
