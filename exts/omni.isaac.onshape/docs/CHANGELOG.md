# Changelog
## [0.6.2] - 2023-02-17
### Fixed
- Fix Ui for preferences pane to not depend on kit version
## [0.6.1] - 2023-02-16
### Fixed
- Tests take into account usd Stage.
## [0.6.0] - 2023-02-04
### Added
- Preferences are set up on Preferences panel now
- Direct import using current asset folder as destination (or default import folder if new stage)
- Support for Assembly configurations
- Adding base and edit layer so user changes are preserved when changing assembly configuration
- Namespace configuration for enterprise users
### Fixed
- properly aligning the document workspace/version across every component being imported.
- Some parts wouldn't load due to wrong part ID vs Encoded part ID being used on request
- General performance and UI responsiveness improvements

## [0.5.0] - 2022-11-22
### Added
- get document directly by pasting the URL in the search bar
- Add Ball (spherical) Mates support
## [0.4.2] - 2022-10-12
### Fixed
- Fixed missing icons for Windows build
- Fixed slider mates with limits that were breaking on import
- Fixed Importing on newer versions that would break when creating USD stages.

## [0.4.1] - 2022-09-02
### Fixed
- Issue where installing requests-oauthlib would return an error even when the package was installed successfully

## [0.4.0] - 2022-08-10
### Changed
- Using Local onshape_client package
- Improvements on import process.
### Fixed
- Increased concurrent pool size to avoid issues on large assemblies.
- Fixed download tracker to only vanish once process is complete
- Handling of errors so import can continue without failures. 
## [0.3.20] - 2022-06-02

- Improvements on import process.

## [0.3.19] - 2022-05-31

- General improvements. 

## [0.3.18] - 2022-05-17

- Add joint values API
- Use current stage meters per unit scaling when importing stage. 

## [0.3.17] - 2022-04-04

- Bugfix for cross-failure when omnigraph USD notice listener captured changes on the stage done within not the main thread. 
- stability improvements

## [0.3.16] - 2022-01-11

- Fix bug where cylindrical mates with same name get overriden.
- Change naming convention for duplicate names
- bugfix for non-ascii parts name that were failing to create usd due to empty filename.

## [0.3.15] - 2022-01-04

- Update dependencies list for standalone import.

## [0.3.14] - 2021-12-15

- Removed mass properties when Import physics is not selected
- Live import / Add to scene as meshes get imported
- Add option to display/Hide unsupported documents
- General UI improvements

## [0.3.13] - 2021-12-13

- Add Cylindrical Mates support
- Change all transforms to Translate/Orient pairs
- Fix document search bug when an empty string is provided after something was typed in and no results were showing.

## [0.3.12] - 2021-09-09

- Renaming to Onshape Importer.
- Make log in non-blocking

## [0.3.11] - 2021-08-10

- Minor bugfixes
- Add Icon and readme description

## [0.3.10] - 2021-08-10

- Fix bug for imported assemblies with physics.

## [0.3.9] - 2021-07-29

- Optimized performance of imported assets

## [0.3.8] - 2021-07-23

- Minor Bugfixes

## [0.3.7] - 2021-06-04

- Minor bugs and Menu Reorg.

## [0.3.6] - 2021-05-28

- UI Reorg

## [0.3.5] - 2021-05-25

- Fix group reorganizing for physics

## [0.3.4] - 2021-05-20

- Change authentication Port to 4851

## [0.3.3] - 2021-05-20

- Bugfixing for mates and mate groups
- Change authentication Port to 3080

## [0.3.2] - 2021-05-10

- Fix import assembly to include sub-assembly mates

## [0.3.1] - 2021-04-25

- Update UI response
- Update Export functions to new API

## [0.3.0] - 2021-04-14

- Implemented Oauth Authentication

## [0.2.0] - 2021-04-14

- Full Import pipeline

## [0.1.3] - 2021-04-12

- Automatically rigs physics when mates are present

## [0.1.2] - 2021-03-15

- Onshape Assembly window
- Loads Assemblies and Parts with mass information where applicable.

## [0.1.1] - 2021-02-02

- Bug fixes

## [0.1.0] - 2021-02-02

- On Shape documents browser widget
