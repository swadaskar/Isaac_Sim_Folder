# Changelog

## [0.8.0] - 2023-03-13
### Fixed
- Added missing imports in element_wrappers __init__.py
- Fixed bad formatting on CheckBox docstring

### Added
- Added get_value() function to CheckBox UIElementWrapper

## [0.7.0] - 2023-02-28

### Changed
- Breaking Change: Removed UIFrameWrapper and replaced with CollapsableFrame UIElementWrapper instance
- Breaking Change: UIElementWrapper get_ui_element() function replaced with .container_frame property that
    gives the user a UI frame that contains everything in the UIElementWrapper (label,buttons,fields,etc.)

### Fixed
- FloatField UIElementWrapper was giving an error when no joint limits were being set
- FloatField setters cast arguments to floats to avoid problems with np.float types

### Added
- Completed basic UI element wrappers in omni.isaac.ui.element_wrappers
- Added accessors to each UIElementWrapper instance to get each omni.ui.Widget that is used.

## [0.6.2] - 2023-02-16

### Added
- Added detailed docstrings to all omni.isaac.ui.element_wrappers.* __init__ and public member functions

## [0.6.1] - 2023-02-15

### Added
- Added DropDown and FloatField UI wrappers in omni.issac.ui/element_wrappers

## [0.6.0] - 2023-02-13

### Added
- Added omni.isaac.ui/element_wrappers with helpful wrappers around UI elements that simplify button and frame creation and management.

## [0.5.2] - 2023-01-19

### Fixed
- split calback tests to reduce errors
### Changed
- rename startup to test_ui

## [0.5.1] - 2023-01-11

### Fixed
- revert to old menu click function to fix hot reload errors

## [0.5.0] - 2023-01-06

### Added
- make_menu_item_description to old menus can use the new action registry

## [0.4.3] - 2022-12-11
### Fixed
- Fixed issue with error when outputting warning message for opening web browser

## [0.4.2] - 2022-12-02
### Fixed
- Fixed bug: File Picker adding extra slash to file path when selecting non-existing files

## [0.4.1] - 2022-11-22
### Fixed
- Missing min/max limits for int field

## [0.4.0] - 2022-11-16
### Added
- Dpad Controller Class

## [0.3.1] - 2022-09-10
### Fixed
- Fix screen print test

## [0.3.0] - 2022-07-18
### Added
- Add a class to print directly onto the screen using Omnigraph.

## [0.2.1] - 2022-06-02
### Changed
- expose labels for file/folder picker

## [0.2.0] - 2022-05-25
### Added
- Windows support to open vscode and folders

## [0.1.3] - 2021-01-18
### Fixed
- Fixes layout issues

## [0.1.2] - 2021-12-14
### Changed
- adjust tooltip for ui Buttons with changed background color

## [0.1.1] - 2021-09-28
### Fixed
- General bugfixes

## [0.1.0] - 2021-07-22

### Added
- Initial version of Isaac Sim UI
