__copyright__ = "Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved."
__license__ = """
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# from the two classes in RadarProvider_Types.h
import ctypes
import carb


# Defines the structure for a raw radar detection
class radarDetection(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        # Radial distance (m)
        ("r_m", ctypes.c_float),
        # Radial velocity (m/s)
        ("rv_ms", ctypes.c_float),
        # Azimuth angle (radians)
        ("az_ang_rad", ctypes.c_float),
        # Angle of elevation (radians)
        ("elev_ang_rad", ctypes.c_float),
        # radar cross section
        ("rcs_dbsm", ctypes.c_float),
        #
        ("semId", ctypes.c_uint32),
        #
        ("matId", ctypes.c_uint32),
        #
        ("objId", ctypes.c_uint32),
    ]


def printRadarDetection(pd):
    print(
        f"r_m {pd.r_m}, rv_ms {pd.rv_ms}, az_ang_rad {pd.az_ang_rad}, elev_ang_rad {pd.elev_ang_rad}, rcs_dbsm {pd.rcs_dbsm}, semId {pd.semId}, matId {pd.matId}, objId {pd.objId}"
    )


# Represents a full radar stream for a major cycle
class radarPointCloud(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        # there is an 64 bit pointer to sync data at top we are skipping.
        # Sensor Id for sensor that generated the scan
        ("sensorID", ctypes.c_uint8),
        # Scan index for sensors with multi scan support
        ("scanIdx", ctypes.c_uint8),
        # Scan timestamp in nanoseconds
        ("timeStampNS", ctypes.c_uint64),
        # Scan cycle count
        ("cycleCnt", ctypes.c_uint64),
        # The max unambiguous range for the scan
        ("maxRangeM", ctypes.c_float),
        # The min unambiguous velocity for the scan
        ("minVelMps", ctypes.c_float),
        # The max unambiguous velocity for the scan
        ("maxVelMps", ctypes.c_float),
        # The min unambiguous azimuth for the scan
        ("minAzRad", ctypes.c_float),
        # The max unambiguous azimuth for the scan
        ("maxAzRad", ctypes.c_float),
        # The min unambiguous elevation for the scan
        ("minElRad", ctypes.c_float),
        # The max unambiguous elevation for the scan
        ("maxElRad", ctypes.c_float),
        # The number of valid detections in the array
        ("numDetections", ctypes.c_uint16),
        # In the c object, the array or detections actually lives here
    ]


def printRadarPointCloud(ps):
    print(
        f"sensorID {ps.sensorID}, scanIdx {ps.scanIdx}, timeStampNS {ps.timeStampNS}, cycleCnt {ps.cycleCnt}, maxRangeM {ps.maxRangeM}, minVelMps {ps.minVelMps}, maxVelMps {ps.maxVelMps}, minAzRad {ps.minAzRad}, maxAzRad {ps.maxAzRad}, minElRad {ps.minElRad}, maxElRad {ps.maxElRad}, numDetections {ps.numDetections}"
    )


class OgnIsaacPrintRTXRadarInfo:
    """
    Example to read raw rtx data in python.
    """

    @staticmethod
    def compute(db) -> bool:
        """read a pointer and print data from it assuming it is Rtx"""
        if not db.inputs.cpuPointer:
            carb.log_warn("invalid data input to OgnIsaacPrintRTXRadarInfo")
            return True
        # raw cpuPointer scan start after 8 bytes
        scan_p = db.inputs.cpuPointer + 8
        scan = ctypes.cast(scan_p, ctypes.POINTER(radarPointCloud))
        printRadarPointCloud(scan[0])

        detections_p = scan_p + ctypes.sizeof(radarPointCloud)
        detections = ctypes.cast(detections_p, ctypes.POINTER(radarDetection))
        printRadarDetection(detections[0])
        printRadarDetection(detections[scan.contents.numDetections - 1])

        return True
