# -*- coding: utf-8 -*-
# @Time    : 2023/5/24 10:06
# @Author  : Star
# @File    : 02.py
# @Software: PyCharm

import sys

print('命令行参数如下:')
# sys.argv 是一个包含命令行参数的列表
for i in sys.argv:
    print(i)
# sys.path 包含了一个 Python 解释器自动查找所需模块的路径的列表
print('\nPython 路径为：', sys.path)
