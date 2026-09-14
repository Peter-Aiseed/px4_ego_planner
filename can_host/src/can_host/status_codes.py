from enum import IntEnum


class StatusCAN(IntEnum):
    STATUS_OK = 0
    STATUS_IDLE = 0
    STATUS_IGNORE = 1
    STATUS_BUSY = 2
    STATUS_INITIALIZING = 3
    STATUS_ARGUS_STARTING = 100
    STATUS_ARGUS_ACTIVE = 103
    STATUS_ARGUS_BUFFER_BUSY = 104
    STATUS_ARGUS_POWERLIMIT = 105
    STATUS_ARGUS_PLL_NOT_LOCKED = 106
    STATUS_ARGUS_NO_OBJECT = 108
    STATUS_ARGUS_EEPROM_BIT_ERROR = 109
    STATUS_ARGUS_INVALID_EEPROM = 110
    STATUS_ARGUS_BUSY_TEST = 191
    STATUS_ARGUS_BUSY_UPDATE = 192
    STATUS_ARGUS_BUSY_CAL_SEQ = 195
    STATUS_ARGUS_BUSY_MEAS = 196
    ERROR_FAIL = -1
    ERROR_ABORTED = -2
    ERROR_READ_ONLY = -3
    ERROR_OUT_OF_RANGE = -4
    ERROR_INVALID_ARGUMENT = -5
    ERROR_TIMEOUT = -6
    ERROR_NOT_INITIALIZED = -7
    ERROR_NOT_SUPPORTED = -8
    ERROR_NOT_IMPLEMENTED = -9
    ERROR_S2PI_RX_ERROR = -51
    ERROR_S2PI_TX_ERROR = -52
    ERROR_S2PI_INVALID_STATE = -53
    ERROR_S2PI_INVALID_BAUDRATE = -54
    ERROR_S2PI_INVALID_SLAVE = -55
    ERROR_NVM_EMPTY = -98
    ERROR_NVM_OUT_OF_RANGE = -99
    ERROR_ARGUS_NOT_CONNECTED = -101
    ERROR_ARGUS_INVALID_CFG = -102
    ERROR_ARGUS_BUFFER_EMPTY = -103
    ERROR_ARGUS_INVALID_SLAVE = -104
    ERROR_ARGUS_INVALID_MODE = -105
    ERROR_ARGUS_BIAS_VOLTAGE_REINIT = -107
    ERROR_ARGUS_LASER_MONITOR_INACTIVE = -108
    ERROR_ARGUS_EEPROM_FAILURE = -109
    ERROR_ARGUS_STALLED = -110
    ERROR_ARGUS_BGL_EXCEEDANCE = -111
    ERROR_ARGUS_XTALK_AMPLITUDE_EXCEEDANCE = -112
    ERROR_ARGUS_LASER_FAILURE = -113
    ERROR_ARGUS_DATA_INTEGRITY_LOST = -114
    ERROR_ARGUS_RANGE_OFFSET_CALIBRATION_FAILED = -115
    ERROR_ARGUS_VSUB_CALIBRATION_FAILED = -116
    ERROR_ARGUS_BUSY = -191
    ERROR_ARGUS_UNKNOWN_LASER = -197
    ERROR_ARGUS_UNKNOWN_CHIP = -198
    ERROR_ARGUS_UNKNOWN_MODULE = -199

    @property
    def description(self) -> str:
        descriptions = {
            0: "Operation Successful",
            1: "Status to be ignored",
            2: "Device busy",
            3: "Device initializing",
            100: "ASIC initializing a new measurement",
            103: "ASIC performing an integration cycle",
            104: "All internal raw data buffers in use",
            105: "Measurement not executed due to power limitations",
            106: "PLL not locked; range may be off",
            108: "No object detected in FOV and measurement range",
            109: "EEPROM bit error corrected (replace sensor soon)",
            110: "Inconsistent EEPROM readout data",
            191: "Busy testing SPI connection",
            192: "Busy updating settings parameters",
            195: "Busy executing calibration sequence",
            196: "Busy executing measurement cycle",
            -1: "Generic fail/error",
            -2: "Process aborted by user/external",
            -3: "Invalid read only operation",
            -4: "Out of range parameters",
            -5: "Invalid argument",
            -6: "Timeout occurred",
            -7: "Not initialized modules",
            -8: "Not supported",
            -9: "Not implemented",
            -51: "SPI error on Rx line",
            -52: "SPI error on Tx line",
            -53: "Function called at wrong driver state",
            -54: "Specified baud rate is not valid",
            -55: "Invalid slave identifier",
            -98: "Read memory block not previously written",
            -99: "Memory is out of range",
            -101: "No device connected; initial SPI tests failed",
            -102: "Inconsistent configuration parameters",
            -103: "Evaluation called but no raw data available",
            -104: "Invalid slave identifier",
            -105: "Invalid measurement mode configuration",
            -107: "APD bias voltage re-initializing (harsh ambient light)",
            -108: "Laser safety monitoring inactive (bias dropout)",
            -109: "EEPROM readout failed (3 distinct invalid reads)",
            -110: "All pixel signals invalid; range stalled",
            -111: "Background light too bright",
            -112: "Crosstalk vector amplitude too high",
            -113: "Laser malfunction; laser safety may not be given",
            -114: "Register data integrity lost (unexpected power cycle)",
            -115: "Range offsets calibration failed",
            -116: "VSUB calibration failed",
            -191: "Device is currently busy",
            -197: "Unknown laser type number",
            -198: "Unknown chip version number",
            -199: "Unknown module number",
        }
        return descriptions.get(self.value, f"Undefined status code ({self.value})")

    @classmethod
    def from_code(cls, value: int) -> "StatusCAN":
        try:
            return cls(value)
        except ValueError:
            return cls.ERROR_FAIL

    def to_string(self) -> str:
        return f"{self.name} ({self.description})"
