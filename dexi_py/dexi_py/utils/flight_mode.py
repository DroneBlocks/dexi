from enum import IntEnum

# These map to PX4 msgs VechicleStatus nav_state
class FlightMode(IntEnum):
    MANUAL = 0
    ALTITUDE = 1
    POSITION = 2
    MISSION = 3
    HOLD = 4
    RTL = 5
    SLOW = 6
    FREE5 = 7
    FREE4 = 8
    FREE3 = 9
    ACRO = 10
    FREE2 = 11
    DESCEND = 12
    TERMINATION = 13
    OFFBOARD = 14
    STABILIZED = 15
    FREE1 = 16
    TAKEOFF = 17
    LAND = 18
    TARGET = 19
    PRECLAND = 20
    ORBIT = 21
    VTOL_TAKOFF = 22