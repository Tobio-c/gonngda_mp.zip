import numpy as np
from datetime import timedelta
from utils.time_windows_extended import time_windows_extended
from utils.track_point import track_point


def cal_time_windows(current_time, jd0, oev0, lon, lat):
    radius_earth = 6371  # 地球半径 (km)
    step = 1  # 轨道预报间隔 (s)
    dt = 21600  # 轨道预报的总时长(s)
    dt = 7200  # 轨道预报的总时长(s)
    # 生成卫星星下点轨迹
    sat_lla = track_point(jd0, oev0, dt, step)  # (rad)
    # 循环依此计算每个目标点的时间窗口
    result = []
    for i in range(0, len(lon)):
        # 生成时间窗口大小
        time_windows = time_windows_extended(sat_lla, radius_earth, lon[i], lat[i])

        # 计算可见时间窗口
        # time_windows = [current_time + timedelta(seconds=t) for t in time_windows]
        res_windows = []
        for window in time_windows:
            res_windows.append([current_time + timedelta(seconds=int(window[0])), current_time + timedelta(seconds=int(window[1]))])
        result.append(res_windows)
    # print('时间窗口', time_windows)
    return result
