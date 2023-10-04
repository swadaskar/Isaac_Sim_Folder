from omni.isaac.onshape import SETTINGS_PATH

# Settings Client

USE_ONSHAPE_KEY = "{}/USE_API_KEY".format(SETTINGS_PATH)
DEFAULT_ONSHAPE_KEY = "{}/API_KEY".format(SETTINGS_PATH)
DEFAULT_ONSHAPE_SECRET = "{}/API_SECRET".format(SETTINGS_PATH)
ONSHAPE_BASE_URL = "{}/BASE_URL".format(SETTINGS_PATH)
ONSHAPE_AUTH_URL = "{}/AUTH_URL".format(SETTINGS_PATH)
ONSHAPE_TOKEN_URL = "{}/TOKEN_URL".format(SETTINGS_PATH)

# Settings Tesselation

ONSHAPE_CHORD_TOLERANCE = "{}/chord_tolerance".format(SETTINGS_PATH)
ONSHAPE_ANGLE_TOLERANCE = "{}/angle_tolerance".format(SETTINGS_PATH)
ONSHAPE_MAX_CHORD = "{}/max_chord".format(SETTINGS_PATH)

# Settings General

ONSHAPE_IMPORT_PHYSICS = "{}/import_physics".format(SETTINGS_PATH)
ONSHAPE_FILTER_UNSUPPORTED = "{}/filter_unsupported".format(SETTINGS_PATH)
ONSHAPE_DEFAULT_FOLDER = "{}/default_import_folder".format(SETTINGS_PATH)
ONSHAPE_IMPORT_IN_PLACE = "{}/import_in_place".format(SETTINGS_PATH)


# Attributes

ONSHAPE_DOCUMENT = "omni:onshape:document"  # documents/[doc]/[wvm]/[wdid]/e/[eid]
ONSHAPE_PART = "omni:onshape:part_id"  # Part ID
