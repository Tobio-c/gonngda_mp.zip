import numpy as np
from math import acos, degrees
from utils.PV_J2000_OEV import PV_J2000_OEV
from utils.OrbitPropagation import OrbitPropagation
from utils.OEV2PV_J2000 import OEV2PV_J2000
from utils.ImageAngle import ImageAngle
from utils.PV_J2000_WGS84 import PV_J2000_WGS84
from utils.P_WGS84_LLA import P_WGS84_LLA
from utils.haversine_distance import haversine_distance
from utils.geodetic_to_ecef import geodetic_to_ecef


def cal_roll(oev, image_time, lat2, lon2, alt2):
    """
    输入:
    oev: 卫星轨道六根数 [a, e, i, Omega, w, M]
    image_time: 计算时刻（单位: 秒，表示自某起始时刻起的时间）
    lat2, lon2, alt2: 目标点的经纬度和高度 单位度
    输出:
    pitch_angle: 俯仰角（单位：度）
    roll_angle: 滚转角（单位：度）
    """

    # 1. 计算卫星在 image_time 时刻的位置和速度
    oev_image = OrbitPropagation(oev, image_time)  # 卫星轨道传播
    # oev_image_deg = np.degrees(oev_image)
    r_sat, v_sat = OEV2PV_J2000(oev_image)  # 转换为位置和速度向量

    # 2. 将卫星位置从轨道坐标系转换为地心地固坐标系（ECEF）
    r_w84, _ = PV_J2000_WGS84(image_time / 86400, r_sat, v_sat)  # 转换为 WGS84 坐标系
    r_sat_ecef = r_w84[:3]  # 获取卫星的位置

    # 3. 计算目标点的 ECEF 坐标
    r_target_ecef = geodetic_to_ecef(lat2, lon2, alt2)

    # 4. 计算卫星与目标点的相对位置向量
    r_relative = (r_target_ecef - np.transpose(r_sat_ecef))[0]

    # 5. 计算目标点的法向量（单位向量）
    r_target_unit = r_target_ecef / np.linalg.norm(r_target_ecef)

    # 6. 计算卫星与目标点的相对单位向量
    r_relative_unit = r_relative / np.linalg.norm(r_relative)

    # 7. 计算俯仰角（单位：度）
    # 使用点积公式计算俯仰角
    cos_pitch = np.dot(r_relative_unit, r_target_unit)
    pitch_angle = degrees(acos(cos_pitch))  # 返回俯仰角（单位：度）

    # 输出俯仰角
    print(f'俯仰角为: {pitch_angle}°')

    r_sat = r_sat.T
    v_sat = v_sat.T

    # 计算卫星的姿态轴
    # Z轴：卫星的反位置矢量（偏航轴）
    sat_z_J2000 = -r_sat
    # Y轴：轨道面法线方向，计算为Z轴与卫星速度的叉积
    sat_y_J2000 = np.cross(sat_z_J2000, v_sat)
    # X轴：滚转轴，由Y轴与Z轴的叉积得到
    sat_x_J2000 = np.cross(sat_y_J2000, sat_z_J2000)

    # 8. 计算滚转角（单位：度）
    # 假设初始滚转轴为sat_x_initial（可以通过初始条件给定）
    sat_x_initial = np.array([1, 0, 0])  # 假设初始滚转轴为[1, 0, 0]

    # 使用点积计算滚转角
    cos_roll = np.dot(sat_x_J2000, sat_x_initial) / (np.linalg.norm(sat_x_J2000) * np.linalg.norm(sat_x_initial))
    roll_angle = degrees(acos(cos_roll))  # 返回滚转角（单位：度）
    print(f'滚转角为: {roll_angle}°')

    return roll_angle, pitch_angle
