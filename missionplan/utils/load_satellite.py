import numpy as np
from utils.PV_J2000_OEV import PV_J2000_OEV


def load_satellite(path):
    # 从data.txt加载初始轨道状态, max_rows=1限制为单星
    rv0 = np.loadtxt(path, max_rows=1)
    # 格式化为1x6向量: [x, y, z, dx, dy, dz]
    rv0 = rv0.reshape(1, -1)
    # 打印卫星初始状态信息: 卫星个数、数据类型、初始状态
    sat_num = len(rv0)
    # print("卫星个数：\n", sat_num)
    # print("卫星轨道数据类型：\n", type(rv0))
    # print("卫星轨道数据维度：\n", np.shape(rv0))
    # print("卫星轨道初始位置和速度：\n", rv0)

    # 计算轨道六根数
    oev0 = np.zeros((sat_num, 6))
    for i in range(sat_num):
        # 坐标转换为轨道六根数: [半长轴a, 偏心率e, 倾角i, 升交点赤经Ω, 近地点幅角w, 真近点角v]
        oev0[i, :] = PV_J2000_OEV(rv0[i, 0:3], rv0[i, 3:6])
    # 弧度转角度
    ove1 = np.rad2deg(oev0)

    return rv0, oev0
