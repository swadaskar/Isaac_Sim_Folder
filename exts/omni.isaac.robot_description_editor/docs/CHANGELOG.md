# Changelog

## [2.1.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [2.1.0] - 2022-11-18

### Fixed

- Catch bug where error was thrown on saving with no Active Joint selected Now a better error is thrown that tells you what you need to fix.

### Changed

- Command Panel now completely expands upon selecting robot.

## [2.0.0] - 2022-10-22

### Changed

- Rebranded extension as "Robot Description Editor"
- Can now load from / export to entire Lula Robot Description files. 

## [1.1.0] - 2022-11-15

### Added

- Added ability to generate collision spheres on a per-link basis

## [1.0.0] - 2022-09-12

### Changed

- Modified structure of extension to organize all features around a pre-specified robot link to eliminate the need to type in correct prim paths and simplify the user experience.

### Added

- Ability to toggle visiblility on selected robot link or the robot as a whole
- Selected link has its own color for nested spheres

## [0.2.0] - 2022-09-02

### Changed

- Enhanced collision sphere interpolation feature to generate spheres with radii following a geometric sequence, positioned so as to produce a smooth conical frustum in the limit of infinite spheres.

## [0.1.1] - 2022-09-02

### Removed

- Removed unused dependencies
- Removed from load function: automatic bookmark to file path outside of Isaac Sim directories.

## [0.1.0] - 2022-08-15

### Added

- Initial version of Collision Sphere Editor Extension
