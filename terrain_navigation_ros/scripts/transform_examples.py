"""
Example coordinate transformations using Vector, Matrix, and Quaternion classes
from pymavlink.
"""

import math
import numpy as np
from pymavlink import quaternion
from pymavlink.quaternion import Quaternion
from pymavlink.rotmat import Vector3
from pymavlink.rotmat import Matrix3


def main():
    print("transform examples")

    # transform from FRD to FLU
    print("transform from FRD to FLU")
    m_flu_frd = Matrix3()
    m_flu_frd.from_euler(math.radians(180), 0, 0)
    q_flu_frd = Quaternion(m_flu_frd)
    v1_frd = Vector3(1, 0, 0)
    v2_frd = Vector3(0, 1, 0)
    v3_frd = Vector3(0, 0, 1)
    v1_flu = q_flu_frd.transform(v1_frd)
    v2_flu = q_flu_frd.transform(v2_frd)
    v3_flu = q_flu_frd.transform(v3_frd)
    print("v1_flu: {}".format(v1_flu))
    print("v2_flu: {}".format(v2_flu))
    print("v3_flu: {}".format(v3_flu))

    # transform from NED to ENU
    print("transform from NED to ENU")
    m_enu_ned = Matrix3()
    m_enu_ned.from_euler(math.radians(180), 0, math.radians(90))
    q_enu_ned = Quaternion(m_enu_ned)
    v1_ned = Vector3(1, 0, 0)
    v2_ned = Vector3(0, 1, 0)
    v3_ned = Vector3(0, 0, 1)
    v1_enu = q_enu_ned.transform(v1_ned)
    v2_enu = q_enu_ned.transform(v2_ned)
    v3_enu = q_enu_ned.transform(v3_ned)
    print("v1_enu: {}".format(v1_enu))
    print("v2_enu: {}".format(v2_enu))
    print("v3_enu: {}".format(v3_enu))

    # transform orientation from NED to ENU.
    print("transform orientation from NED to ENU")
    m_enu_ned = Matrix3()
    m_enu_ned.from_euler(math.radians(180), 0, math.radians(90))
    q_enu_ned = Quaternion(m_enu_ned)

    m_rot_ned = Matrix3()
    m_rot_ned.from_euler(math.radians(90), math.radians(0), math.radians(0))
    q_rot_ned = Quaternion(m_rot_ned)

    euler_ned = np.degrees(q_rot_ned.euler)
    print("euler_ned: {:.0f} {:.0f} {:.0f}".format(
        euler_ned[0], euler_ned[1], euler_ned[2]))

    q_rot_enu = q_enu_ned * q_rot_ned * q_enu_ned.inversed
    euler_enu = np.degrees(q_rot_enu.euler)
    print("euler_enu: {:.0f} {:.0f} {:.0f}".format(
        euler_enu[0], euler_enu[1], euler_enu[2]))


if __name__ == "__main__":
    main()
