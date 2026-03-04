import numpy as np
from math import acos, degrees, atan2
from utils.OrbitPropagation import OrbitPropagation  # 轨道传播（输入六根数+时间，输出六根数）
from utils.OEV2PV_J2000 import OEV2PV_J2000  # 六根数转J2000系位置/速度
from utils.PV_J2000_WGS84 import PV_J2000_WGS84  # J2000系转WGS84（ECEF）系
from utils.geodetic_to_ecef import geodetic_to_ecef  # 经纬度（LLA）转ECEF坐标


def cal_roll(orbit_elements, overpass_time, target_lat, target_lon, target_alt=0):
    """
    计算卫星对目标点的侧摆角（即绕轨道X轴的滚转角）
    输入：
        orbit_elements: 轨道六根数 [半长轴a, 偏心率e, 轨道倾角i, 升交点赤经Ω, 近地点幅角ω, 平近点角M]（单位：弧度，a单位：米）
        overpass_time: 过顶时刻（单位：秒，自起始时刻起的时间）
        target_lat: 目标点纬度（单位：度）
        target_lon: 目标点经度（单位：度）
        target_alt: 目标点高度（单位：米，默认0，即大地水准面）
    输出：
        side_swing_angle: 侧摆角（单位：度，正值为轨道右侧，负值为左侧）
    """
    # -------------------------- 步骤1：计算卫星在过顶时刻的位置和速度（ECEF系） --------------------------
    # 1.1 轨道传播：由初始六根数和过顶时刻，得到过顶时刻的六根数
    oev_overpass = OrbitPropagation(orbit_elements, overpass_time)  # 输出：过顶时刻的六根数（弧度）

    # 1.2 六根数转J2000系位置/速度向量
    r_j2000, v_j2000 = OEV2PV_J2000(oev_overpass)  # r_j2000: (3,1)或(3,)，位置向量（米）；v_j2000: 速度向量（米/秒）

    # 1.3 J2000系转WGS84（ECEF）系：卫星位置/速度从惯性系转到地固系
    # 注：PV_J2000_WGS84输入通常为“儒略日差”（overpass_time/86400）、J2000位置/速度
    r_sat_ecef, v_sat_ecef = PV_J2000_WGS84(overpass_time / 86400, r_j2000, v_j2000)
    # 确保向量形状为(3,)（避免矩阵维度问题）
    r_sat_ecef = np.squeeze(r_sat_ecef)  # 卫星ECEF位置：(3,)
    v_sat_ecef = np.squeeze(v_sat_ecef)  # 卫星ECEF速度：(3,)

    # -------------------------- 步骤2：计算目标点的ECEF坐标 --------------------------
    # 经纬度（度）转ECEF坐标（米）：geodetic_to_ecef(lat, lon, alt)
    r_target_ecef = geodetic_to_ecef(target_lat, target_lon, target_alt)  # 输出：(3,)

    # -------------------------- 步骤3：构建卫星的“轨道坐标系”（关键！） --------------------------
    # 轨道坐标系三轴定义（右手定则，ECEF系下）：
    # X轨（x_orbit）：沿卫星飞行方向（速度向量单位化）
    v_sat_unit = v_sat_ecef / np.linalg.norm(v_sat_ecef)  # 速度单位向量
    x_orbit = v_sat_unit  # 轨道X轴（前向）

    # Z轨（z_orbit）：沿卫星到地心的反方向（星下点方向，垂直地面）
    r_sat_unit = r_sat_ecef / np.linalg.norm(r_sat_ecef)  # 卫星到地心的单位向量
    z_orbit = -r_sat_unit  # 轨道Z轴（星下点方向）

    # Y轨（y_orbit）：轨道平面法向，由Z轨×X轨（右手定则）确保三轴正交
    y_orbit = np.cross(z_orbit, x_orbit)  # 轨道Y轴（法向）
    y_orbit = y_orbit / np.linalg.norm(y_orbit)  # 单位化（消除计算误差）

    # 验证轨道坐标系正交性（可选，用于debug）
    dot_xy = np.dot(x_orbit, y_orbit)
    dot_yz = np.dot(y_orbit, z_orbit)
    dot_zx = np.dot(z_orbit, x_orbit)
    print(f"轨道坐标系正交性验证（应接近0）：XY={dot_xy:.6f}, YZ={dot_yz:.6f}, ZX={dot_zx:.6f}")

    # -------------------------- 步骤4：计算“载荷视轴向量”（卫星→目标点） --------------------------
    # 卫星到目标点的相对向量（ECEF系）
    r_sat2target = r_target_ecef - r_sat_ecef  # (3,)
    r_sat2target_unit = r_sat2target / np.linalg.norm(r_sat2target)  # 单位化视轴向量

    # -------------------------- 步骤5：计算侧摆角（滚转角） --------------------------
    # 原理：侧摆角是“视轴向量在轨道Y轴方向的投影”对应的角度——即绕X轨轴旋转的角度
    # 步骤：
    # 1. 将视轴向量投影到“轨道Y-Z平面”（消除X轨方向分量，因为滚转不影响X方向）
    # 2. 计算投影向量与Z轨（星下点方向）的夹角，即为侧摆角
    # 3. 通过Y轨方向的投影符号判断左右（正：右侧，负：左侧）

    # 5.1 视轴向量在轨道Y-Z平面的投影（点积投影）
    proj_y = np.dot(r_sat2target_unit, y_orbit)  # Y轴方向投影分量
    proj_z = np.dot(r_sat2target_unit, z_orbit)  # Z轴方向投影分量

    # 5.2 计算侧摆角（用反正切求角度，自动处理正负，避免acos的二义性）
    side_swing_angle = degrees(atan2(proj_y, proj_z))  # atan2(y, z)：返回[-180°, 180°]

    # （可选）将角度范围调整为[0°, 360°]或[-90°, 90°]（根据任务需求）
    # if side_swing_angle < 0:
    #     side_swing_angle += 360

    print(f"目标点侧摆角：{side_swing_angle:.2f}°（正值：轨道右侧，负值：轨道左侧）")
    return side_swing_angle