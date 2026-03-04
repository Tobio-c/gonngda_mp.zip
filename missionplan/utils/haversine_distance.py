import numpy as np
import math
from geopy.distance import geodesic
from pyproj import Geod

def haversine_distance(lon1, lat1, lon2, lat2, r):
    """
    计算两个经纬度点之间的距离
    :param lon1: 第一个点的经度
    :param lat1: 第一个点的纬度
    :param lon2: 第二个点的经度
    :param lat2: 第二个点的纬度
    :param R: 地球半径
    :return: 两点之间的距离
    """
    # lon1, lat1, lon2, lat2 = map(np.radians, [lon1, lat1, lon2, lat2])  # 角度转弧度
    # delta_lon = lon2 - lon1
    # delta_lat = lat2 - lat1
    # a = np.sin(delta_lat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(delta_lon / 2) ** 2
    # # c = 2 * np.arcsin(np.sqrt(a))
    # # c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    # c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    #
    # return r * c

    # 定义两个经纬度坐标点
    point1 = (lon1, lat1)  # 北京某点
    point2 = (lon2, lat2)  # 上海某点

    distance_cgcs2000 = geodesic(point1, point2)
    return distance_cgcs2000.km




# 定义两个经纬度坐标点
# point1 = (39.904200, 116.407400)  # 北京某点
# point2 = (31.230400, 121.473700)  # 上海某点
#
# # 计算距离，单位为千米
# # 2. 手动指定其他椭球体（如CGCS2000，中国境内更高精度）
# distance_cgcs2000 = geodesic(point1, point2)
# print(f"CGCS2000椭球体计算（米）：{distance_cgcs2000.km:.3f}")

# print(haversine_distance(37.193462, 100.1347, 37.194363, 100.1348, 6371))

# # 初始化WGS84椭球的大地测量对象
# geod = Geod(ellps='WGS84')
#
# # 计算距离（返回：方位角1, 方位角2, 距离（米））
# az1, az2, distance = geod.inv(point1[1], point1[0], point2[1], point2[0])
# print(f"pyproj高精度距离（米）：{distance:.3f}")