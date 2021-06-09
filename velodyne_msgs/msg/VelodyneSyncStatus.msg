# Velodyne LIDAR time sync status.
## Constants (status codes).
## Following Velodyne's documentation - 0 = No PPS detected, 1 = Syncing to PPS, 2 = PPS locked, 3 = Error
uint8 PPS_UNKNOWN = 0
uint8 PPS_SYNCING = 1
uint8 PPS_LOCKED = 2
uint8 PPS_ERROR = 3

## Message
Header           header                 # standard ROS message header
time             lidar_timestamp        # Lidar internal timestamp
uint8            pps_lock               # PPS lock status. This field is expected to take a value from the PPS_XYZ constants previously declared.
