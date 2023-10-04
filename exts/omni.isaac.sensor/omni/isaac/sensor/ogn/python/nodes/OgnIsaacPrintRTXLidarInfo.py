__copyright__ = "Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved."
__license__ = """
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

import ctypes
import carb


class sensorPose(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        # world space translation. [X, Y, Z] in m (trace begin)
        ("posM", ctypes.c_float * 3),
        # world space rotation. [X, Y, Z, W] quaternion (trace begin)
        ("orientation", ctypes.c_float * 4),
    ]


class lidarAsyncParameter(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        # number of ticks (sensor positions) in this trace data
        ("numTicks", ctypes.c_uint32),
        # ensor frequency in hz
        ("scanFrequency", ctypes.c_float),
        # number of ticks of one full scan of the sensor
        ("ticksPerScan", ctypes.c_uint32),
        # maximum possible size of the lidar trace data in bytes (can be used for initialization)
        ("maxSizeBuffer", ctypes.c_size_t),
        # current size of the lidar trace data
        ("currentSizeBuffer", ctypes.c_size_t),
        # current size of the lidar trace data
        ("numChannels", ctypes.c_uint32),
        # number of echos per detector/laser
        ("numEchos", ctypes.c_uint8),
        # start time of the trace data
        ("startTimeNs", ctypes.c_uint64),
        # delta time of the trace data
        ("deltaTimeNs", ctypes.c_uint64),
        # start time of the corresponding scan (i.e. full rotation of a spinning lidar)
        ("scanStartTimeNs", ctypes.c_uint64),
        # start tick of this frame/trace
        ("startTick", ctypes.c_uint32),
        # sensor transformation at frame start
        ("frameStart", sensorPose),
        # sensor transformation at frame end
        ("frameEnd", sensorPose),
    ]


def printparams(params):
    print(
        f"numTicks {params.numTicks}, scanFrequency {params.scanFrequency}, ticksPerScan {params.ticksPerScan}, maxSizeBuffer {params.maxSizeBuffer}, currentSizeBuffer {params.currentSizeBuffer}, numChannels {params.numChannels}, numEchos {params.numEchos}, startTimeNs {params.startTimeNs}, deltaTimeNs {params.deltaTimeNs}, scanStartTimeNs {params.scanStartTimeNs}, startTick {params.startTick}, frameStart.posM {params.frameStart.posM[0]}, {params.frameStart.posM[1]}, {params.frameStart.posM[2]}, frameStart.orientation {params.frameStart.orientation[0]}, {params.frameStart.orientation[1]}, {params.frameStart.orientation[2]}, {params.frameStart.orientation[3]}, frameEnd.posM {params.frameEnd.posM[0]}, {params.frameEnd.posM[1]}, {params.frameEnd.posM[2]}, frameEnd.orientation {params.frameEnd.orientation[0]}, {params.frameEnd.orientation[1]}, {params.frameEnd.orientation[2]}, {params.frameEnd.orientation[3]}"
    )


class lidarReturn(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        # azimuth in deg [-180,180]
        ("azimuthDeg", ctypes.c_float),
        # elevation in deg [-90, 90]
        ("elevationDeg", ctypes.c_float),
        # distance in m
        ("distance", ctypes.c_float),
        # intensity [0,1]
        ("intensity", ctypes.c_float),
        # velocity at hit point in sensor coordinates [m/s]
        ("velocityMs", ctypes.c_float * 3),
        # deltatime in ns from the head (relative to tick time)
        ("deltaTimeNs", ctypes.c_uint32),
        # beam emitter id
        ("emitterId", ctypes.c_uint32),
        # beam/laser detector id
        ("beamId", ctypes.c_uint32),
        # hit point material id
        ("materialId", ctypes.c_uint32),
        # hit point normal
        ("hitPointNormal", ctypes.c_float * 3),
        # hit point object id
        ("objectId", ctypes.c_uint64),
    ]


def printreturn(re):
    print(
        f"azimuthDeg {re.azimuthDeg}, elevationDeg {re.elevationDeg}, distance {re.distance}, intensity {re.intensity}, velocityMs ({re.velocityMs[0]}, {re.velocityMs[1]}, {re.velocityMs[2]}), deltaTimeNs {re.deltaTimeNs}, emitterId {re.emitterId} , beamId {re.beamId}, materialId {re.materialId}, hitPointNormal ({re.hitPointNormal[0]}, {re.hitPointNormal[1]}, {re.hitPointNormal[2]}), objectId {re.objectId}"
    )


class lidarTick(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        # azimuth for tick
        ("tickAzimuth", ctypes.c_float),
        # emitter state tick belongs to.
        ("state", ctypes.c_uint32),
        # timestamp of tick
        ("timeStampNs", ctypes.c_uint64),
    ]


def printtick(tick):
    print(f"tickAzimuth {tick.tickAzimuth}, state {tick.state}, timeStampNs {tick.timeStampNs}")


class OgnIsaacPrintRTXLidarInfo:
    """
    Example to read raw rtx data in python.
    """

    @staticmethod
    def compute(db) -> bool:
        """read a pointer and print data from it assuming it is Rtx"""
        if not db.inputs.cpuPointer:
            carb.log_warn("invalid data input to OgnIsaacPrintRTXLidarInfo")
            return True
        # raw cpuPointer params start after 36 bytes
        params_p = db.inputs.cpuPointer + 36
        params = ctypes.cast(params_p, ctypes.POINTER(lidarAsyncParameter))
        printparams(params[0])

        ticks_p = params_p + ctypes.sizeof(lidarAsyncParameter)
        ticks = ctypes.cast(ticks_p, ctypes.POINTER(lidarTick))
        printtick(ticks[0])
        printtick(ticks[params.contents.numTicks - 1])

        # idx =  echo + channel*numEchos + tick * numEchos * numChannels
        returns_p = ticks_p + ctypes.sizeof(lidarTick) * params.contents.numTicks
        returns = ctypes.cast(returns_p, ctypes.POINTER(lidarReturn))
        printreturn(returns[0])
        printreturn(returns[params.contents.numTicks * params.contents.numEchos * params.contents.numChannels - 1])

        return True
