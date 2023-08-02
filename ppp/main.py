import numpy as np# 这是一个示例 Python 脚本。

# 按 Shift+F10 执行或将其替换为您的代码。
# 按 双击 Shift 在所有地方搜索类、文件、工具窗口、操作和设置。


def print_hi():
    # p= np.array([[547.6,111.98],[550.42,112.59],[552.75,110.10],[552.46,108.75],[548.21,107.31]])
    # sum=0
    # for i in range(p.shape[0]):
    #     sum+=np.sqrt((p[i][0]-550)**2+(p[i][1]-110)**2)
    #
    # print(sum/5)
    p = np.array([[6,2], [15,5]])
    q = np.array([[3], [2]])
    print(np.dot(np.transpose(p),p))



# 按间距中的绿色按钮以运行脚本。
if __name__ == '__main__':
    print_hi()

# 访问 https://www.jetbrains.com/help/pycharm/ 获取 PyCharm 帮助
