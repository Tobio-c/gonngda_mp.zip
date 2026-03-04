import numpy as np
def get_key_target_lib():
    # 输入
    dtype = [('name', 'U20'), ('lon', 'float'), ('lat', 'float')]
    key_target_list = np.array([
        # [120.2864, 22.6815],  # 左营海军基地(高雄)
        # [121.5654, 25.0330],  # 台北101大楼
        # [120.4620, 22.6720],  # 屏东空军基地(屏东)
        # [121.6239, 24.0137],  # 花莲空军基地
        # [121.7544, 24.7597],  # 苏澳海军基地(宜兰)
        # [120.8400, 22.1200]
        [121.644, 39.4492],
        # [122.805, 39.4492],
        # [122.805, 38.867],
        # [121.644, 38.867]

    ]) # 九鹏导弹试验场
    # 定义各目标点高度、重要程度
    target_nums = len(key_target_list)
    height = np.zeros((target_nums, 1))
    target_values = 0.5 + 0.5 * np.random.rand(target_nums, 1)  # 这里为随机设置各目标的重要程度
    # 数组形式：[[序号, 经度, 维度, 高度, 重要程度], [序号, 经度, 维度, 高度, 重要程度]]
    key_target_lib = np.column_stack(((np.arange(1, target_nums + 1)).reshape(-1, 1), key_target_list[:, 0] * np.pi/180, key_target_list[:, 1] * np.pi/180, height, target_values))
    print("重点目标库: -- [序号, 经度, 维度, 高度, 重要程度]\n", key_target_lib)
    return key_target_lib

