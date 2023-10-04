# Changelog

## [1.0.0] - 2023-02-28
### Changed
- Updated Templates along with breaking changes to omni.isaac.ui replacing UIFrameWrapper with CollapsableFrame instance of UIElementWrapper
- Updated Configuration Tooling Template to use rebuild() function of a CollapsableFrame instead of pre-allocating 100 invisible frames.

### Added
- Added "UI Component Library Template" to show the usage of each UIElementWrapper

## [0.3.1] - 2023-02-17
### Fixed
- Fixed UI bug in Template Generator that allowed unnamed templates to be created

## [0.3.0] - 2023-02-15
### Added
- Added "Configuration Tooling Template" which allows the user to interact with a loaded robot Articulation through the UI.

### Removed
- Removed "Async Scenario Template" since it doesn't stand out well from "Loaded Scenario Template"

## [0.2.0] - 2023-02-13
### Added
- Added "Loaded Scenario Template" which has a load and reset button that are connected to Core.

## [0.1.3] - 2023-02-01
### Fixed
- Async Scenario Template "Run Scenario" button resets properly

## [0.1.2] - 2023-01-10
### Fixed
- Async Scenario Template now disables buttons until an Articulation is selected

## [0.1.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [0.1.0] - 12-09-2022

### Added

- Initial version of Extension Template Generator
