
from datetime import datetime, timezone, timedelta
from utils.JulianDay import JulianDay
from utils.LOS_YOZ_Angle import LOS_YOZ_Angle


if __name__ == '__main__':
    jd0 = JulianDay(2025, 7, 3, 4, 0, 0)
    # print(LOS_YOZ_Angle([6906.64, 0.001, 1.69995, 0.174533, 4.88692, 1.74533], jd0, 8 * 60 + 8.468, 39.4492, 121.644, 0))
    print(LOS_YOZ_Angle([6906.64, 1e-15, 1.7, 0, 2.0944, 4.71239], jd0, 34 * 60 + 25, 39.4492, 121.644, 0))


