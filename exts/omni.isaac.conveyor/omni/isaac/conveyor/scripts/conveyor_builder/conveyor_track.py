from enum import Enum
import asyncio
from typing import List
from omni.kit.widget.filebrowser import find_thumbnails_for_files_async


class CheckEnum(Enum):
    @classmethod
    def has_value(cls, value):
        return value in cls._value2member_map_


class Style(CheckEnum):
    BELT = 0
    ROLLER = 1
    DUAL = 2


class Angle(CheckEnum):
    NONE = 0
    HALF = 1
    FULL = 2


class Curvature(CheckEnum):
    NONE = 0
    SMALL = 1
    MEDIUM = 2
    LARGE = 3


class Ramp(CheckEnum):
    FLAT = 0
    ONE = 1
    TWO = 2
    THREE = 3
    FOUR = 4


class Type(CheckEnum):
    START = 0
    STRAIGHT = 1
    Y_MERGE = 2
    T_MERGE = 3
    FORK_MERGE = 4
    END = 5


class ConveyorTrack:
    """
    Base Conveyor track class, every track type should derive from this.
    """

    def __init__(
        self,
        base_usd,
        style: Style = Style.BELT,
        angle: Angle = Angle.NONE,
        curvature: Curvature = Curvature.SMALL,
        ramp: Ramp = Ramp.FLAT,
        type: Type = Type.STRAIGHT,
        start_level: int = 0,
        direction: int = 1,
        anchors: List[str] = [""],
        thumb_loaded_callback=None,
        **kwargs,
    ):
        self._style = Style[style]
        self._angle = Angle[angle]
        self._curvature = Curvature[curvature]
        self._ramp = Ramp[ramp]
        self._type = Type[type]
        self._start_level = start_level  # At which level it starts the convey (for dual belts it's always zero)
        self._direction = (
            direction  # For curves, -1 is left, 1 is right, for ramps 1 is up, -1 is down. No effect on other assets.
        )

        # self._reference_asset = args.get("asset", None)
        self.base_usd = base_usd
        self._prim = None
        self.thumb = None
        self.thumb_task = asyncio.ensure_future(self.get_thumb_async())
        self.thumb_task.add_done_callback(self.thumb_callback)
        self._anchors = anchors
        self._parent_tracks = []  # For dual tracks there should be two parents, one for each track
        self._child_tracks = []  # For dual tracks there can be two children, one for each track.
        self._thumb_callback = thumb_loaded_callback

        # physics authoring configs
        self.conveyor_nodes = kwargs.get("conveyor_nodes", {})

    async def get_thumb_async(self):
        thumb = await find_thumbnails_for_files_async([self.base_usd])
        if self.base_usd in thumb:
            self.thumb = thumb[self.base_usd]
        return None

    def thumb_callback(self, task):
        if self._thumb_callback:
            self._thumb_callback(self)

    def get_thumb(self):
        if self.thumb:
            return self.thumb
        return ""

    def get_anchors(self, direction=1):
        if direction == 1:
            new_anchors = [a for a in self._anchors]
        else:
            new_anchors = [a for a in self._anchors.__reversed__()]
        return new_anchors

    @property
    def style(self):
        return self._style

    @style.setter
    def style(self, value):
        if Style.has_value(value):
            self._style = Style(value)

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, value):
        if Angle.has_value(value):
            self._angle = Angle(value)

    @property
    def curvature(self):
        return self._curvature

    @curvature.setter
    def curvature(self, value):
        if Curvature.has_value(value):
            self._angle = Curvature(value)

    @property
    def ramp(self):
        return self._ramp

    @ramp.setter
    def ramp(self, value):
        if Ramp.has_value(value):
            self._ramp = Ramp(value)

    @property
    def type(self):
        return self._type

    @type.setter
    def type(self, value):
        if Type.has_value(value):
            self._type = Type(value)

    def start_level(self, trackIndex: int = 0, direction=1):
        if direction == 1:
            return self.get_start()
        else:
            return self.get_end()

    def get_start(self, trackIndex: int = 0):
        if self.style == Style.DUAL:
            return trackIndex
        else:
            return self._start_level

    def get_end(self, trackIndex: int = 0):
        if self.style == Style.DUAL:
            return trackIndex
        else:
            return self._start_level + int(self.ramp.value)

    def end_level(self, trackIndex: int = 0, direction=1):
        if direction == 1:
            return self.get_end(trackIndex)
        else:
            return self.get_start(trackIndex)

    def get_config(self):
        config = {}
        config["style"] = self.style
        config["angle"] = self.angle
        config["curvature"] = self.curvature
        config["ramp"] = self.ramp
        config["start_level"] = self._start_level
        config["type"] = self.type
