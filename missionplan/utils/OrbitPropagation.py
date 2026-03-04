import numpy as np
import math
from utils.solve_kepler import solve_kepler


def OrbitPropagation(oev_0, dt):
    # mu = 398600.4415
    # a0 = oev_0[0]
    # e0 = oev_0[1]
    # phi0 = oev_0[5]
    # E0 = np.arctan2(np.sin(phi0) * np.sqrt(1 - e0 ** 2), e0 + np.cos(phi0))
    # M0 = E0 - e0 * np.sin(E0)
    # Mt = M0 + dt * np.sqrt(mu / a0 ** 3)
    # Et = solve_kepler(e0, Mt)
    #
    # ft = np.mod(2 * np.arctan(np.sqrt((1 + e0) / (1 - e0)) * np.tan(Et / 2)), 2 * np.pi)
    # oev_t = oev_0.copy()
    # oev_t[5] = ft

    a = oev_0[0]
    e = oev_0[1]
    i = oev_0[2]
    omega_0 = oev_0[3]
    Omega_0 = oev_0[4]
    nu_0 = oev_0[5]

    J2 = 1.08263e-3  # 地球的J2摄动系数
    R_e = 6378.137  # 地球半径，单位为千米
    mu = 398600.4418  # 地球引力常数，单位为km  ** 3 / s  ** 2
    omega_earth = 7.2921159e-5  # 地球自转角速度，单位为rad / s

    # 时间相关参数
    T = 2 * math.pi * math.sqrt(a ** 3 / mu)  # 轨道周期，单位为秒
    n = math.sqrt(mu / a ** 3)  # 平均运动，单位为弧度 / 秒

    # 1.将初始真近点角nu_0转换为偏近点角E_0
    E_0 = 2 * math.atan(math.sqrt((1 - e) / (1 + e)) * math.tan(nu_0 / 2)) 

    # 2.使用开普勒方程计算初始平近点角M_0
    M_0 = E_0 - e * math.sin(E_0) 

    # 3.J2摄动对升交点黄经的影响
    Omega_dot = -1.5 * J2 * (R_e ** 2 / a ** 2) * (n / (1 - e ** 2) ** 2) * math.cos(i)

    # 4.J2摄动对近地点幅角的影响
    omega_dot = 0.75 * J2 * (R_e ** 2 / a ** 2) * (n / (1 - e ** 2) ** 2) * (5 * math.cos(i) ** 2 - 1)

    # 5.更新轨道参数
    Omega_t = Omega_0 + Omega_dot * dt  # 考虑J2后的升交点黄经
    omega_t = omega_0 + omega_dot * dt  # 考虑J2后的近地点幅角
    M_t = M_0 + n * dt  # 平近点角的演化（无J2影响）

    # 6.牛顿迭代法解开普勒方程，得到偏近点角E_t
    tol = 1e-6 
    E_t = M_t  # 初值

    while True:
        if np.isnan(E_t).any():
            raise ValueError("轨道参数有误")
        E_new = E_t - (E_t - e * math.sin(E_t) - M_t) / (1 - e * math.cos(E_t))
        if abs(E_new - E_t) < tol:
            break
        E_t = E_new

    # 计算真实近点角nu
    nu_t = 2 * math.atan2(math.sqrt(1 + e) * math.sin(E_t / 2), math.sqrt(1 - e) * math.cos(E_t / 2))
    if nu_t < 0:
        nu_t = 2 * math.pi + nu_t

    oev_t = [a, e, i % (2*math.pi), omega_t % (2*math.pi), Omega_t % (2*math.pi), nu_t % (2*math.pi)]
    return oev_t
