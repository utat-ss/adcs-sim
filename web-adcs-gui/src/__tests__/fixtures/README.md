# MATLAB replay fixture

`matlab-300s-subset.json` retains unmodified samples at 0, 150, and 300 seconds
from a `final_adcs_results_300s_replay.json` export of the `simulink` branch, exported
21 September 2026. Original metadata and all 15 mode-change events are retained.
The original sampleCount describes the full export, not this three-sample fixture.
This is historical simulation evidence from before the 22 September tracker fix.

The unit test checks epoch, radius/velocity units, delivered wheel telemetry and
events between samples. A separate modified copy tests commanded versus delivered
torque because the retained samples have zero wheel commands.

Set `ADCS_REPLAY_PATH` to the full local export when running the "imports actual"
Playwright test to exercise every exported sample. Without that environment
variable, the test uses this portable fixture.
