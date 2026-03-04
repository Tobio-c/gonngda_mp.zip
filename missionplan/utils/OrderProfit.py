from utils.NextProfit import NextProfit
def OrderProfit(order, tar_cal):
    i = 0
    order = order.astype(int)
    profit = NextProfit(0, order[0], tar_cal)
    if order.size == 1:
        return profit
    while order[i+1] != 0:
        profit1 = NextProfit(order[i], tar_cal)
        profit = profit + profit1
        i = i + 1
    return profit
