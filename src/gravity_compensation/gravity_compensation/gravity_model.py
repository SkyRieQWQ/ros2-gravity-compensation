import math


def calculate_gravity_torques(
    q1,
    q2,
    m1=1.0,
    m2=1.0,
    l1=1.0,
    lc1=0.5,
    lc2=0.5,
    g=9.81,
):
    """计算平面两连杆机械臂的重力补偿力矩。

    q1、q2 的单位为弧度。
    角度零点定义为连杆水平向右。
    """
    tau1 = (
        (m1 * lc1 + m2 * l1) * g * math.cos(q1)
        + m2 * lc2 * g * math.cos(q1 + q2)
    )
    tau2 = m2 * lc2 * g * math.cos(q1 + q2)

    return tau1, tau2


if __name__ == "__main__":
    torque1, torque2 = calculate_gravity_torques(0.0, 0.0)
    print(f"joint1 gravity torque: {torque1:.3f} N*m")
    print(f"joint2 gravity torque: {torque2:.3f} N*m")
