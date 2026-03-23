# ADCS Simulator & Controller Repo
This repo is intended to house all code development of the UTAT-SS in-house attitude and flight dynamics simulator & controller.

# Development Environment
Simulator & controller software is developed primarily using MATLAB & Simulink. The recommended version is R2025b.

Secondary development may be done using Python. **IMPORTANT**: Python shall not be used for any flight software.
## Packages
Developers will require the following MATLAB/Simulink add-ons:
- Aerospace Blockset
## Flight Software
In the event UTAT-SS decides to deploy in-house developed ADCS software, it should be derived from the code developed in this repo. This may be done using the MATLAB Coder add-on for direct conversion or it may be manually translated. Flight software will be subject to standard UTAT-SS coding & code review standards.

# Repo Usage
## Branching
Pushing directly to `main` is discouraged. Please do all development on a secondary branch and only merge to `main` once the code has been reviewed.

Branches should be feature-specific. Developers are encouraged to favour creating more branches over packing single branches with large amounts of new code. This improves traceability, reviewability, and overall organization.

Branch names should be short and provide some insight into what is being developed in that branch.
## Commits
Changes should be committed often, but ensure changes are non-breaking. For example, commit when a new function has been written and tested but prior to its integration with existing codepaths. Another commit can be pushed upon successful integration. Avoid committing large additions, deletions, or refactors all at once.

Commit messages should be concise and informative. Developers are encouraged to use the following tags along with a brief description of the included work:
- `ADD`: Adding a new feature, function, file, etc.
- `CHG`: Changing a function signature or codepath.
- `DEL`: Removing a feature, function, file, etc.
- `FIX`: Fixing a known bug.
If possible, it is preferrable to refer to a specific work ticket relating to the commit.
## Merging to `main`
Currently, no CI/CD pipelines exist for automatic code testing. Developers are asked to adhere to the usage rules described herein. Code should only be merged when it has passed appropriate unit testing and has proven to integrate with existing code in `main` without breaking.

# Code Standards
All submitted code must be appropriately documented. Classes, systems, functions, and other callables must include docstrings providing a detailed description of the code's purpose, the name, type, and description of inputs, the name, type, and description of outputs, and any discretely defined errors.

MATLAB code is often written in a math-like fashion, favouring simple variable names (e.g. `x` or `v` rather than `pos` or `vel`). This is acceptable for compactification of mathematically-heavy algorithms, but developers should include detailed comments throughout to improve readability.

# Simulator vs Controller
To perform development of ADCS algorithms and software, it is necessary to have a "ground truth" to supply data to the estimation and control algorithms. This is the domain of the simulator, and it is generally most convenient to colocate the simulator in the same model as the controller.

Simulator elements exist strictly to provide this "ground truth." It is important to understand the separation. Simulator elements make use of high-fidelity models to provide the most precise and accurate representation of reality as possible. It *must only* accept as an input the exact state of the system at any given time.

The controller (and estimator, along with other elements) draws from the simulator's output stream after it has been made to resemble the data it would observe in flight. For example, the exact state of the spacecraft is never observed directly. Instead, it is observed via the sensors, which provide a noisy, biased, rotated, and potentially incomplete measurement at some discrete rate.

This dichotomy can drive confusing design decisions, such as seemingly duplicated code. For example, consider how we define a sensor's orientation on the spacecraft body. The design is likely to call for a nice value like `[1, 0, 0]`, but when the satellite is built that sensor may actually be slightly skewed, say `[0.989, 0.104, 0.104]`. Our belief is that the sensor is pointed along `[1, 0, 0]`, so this is what the controller will use, but it will not be exactly right. To ensure this does not significantly degrade performance, we may choose to model any number of imperfect mounting orientation in the simulator (we never know which one is exactly the right one). We end up having two distinct values for the same thing, with one used by the simulator and one used by the controller. It is important to hold this distinction in mind and to keep track of changes such that both values are updated as necessary.
