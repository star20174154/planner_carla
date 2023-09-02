# -*- coding: utf-8 -*-
# @Time    : 2023/5/23 22:35
# @Author  : Star
# @File    : class_test.py
# @Software: PyCharm


# class MyClass:
#     def __init__(self):
#         print("^^^^^^^^")
#     def public_method(self):
#         print("This is a public method.")
#
#
# obj = MyClass()  # 创建类的实例对象
#
# # 调用公有方法
# obj.public_method()  # 输出: This is a public method.
#
# # 也可以通过类名调用公有方法  不行吧
# MyClass.public_method()  # 输出: This is a public method.


#
# 私有方法是在类中以双下划线（__）开头的方法，它们只能在类的内部被访问和调用，无法从类的外部直接访问。
# 私有方法用于实现类的内部功能或辅助方法，不被外部使用

class MyClass:
    def _private_method(self):
        print("This is a private method.")

    def public_method(self):
        print("This is a public method.")
        self._private_method()  # 在公有方法内部调用私有方法

# 创建类的实例对象
obj = MyClass()

# 调用公有方法
# obj.public_method()
# 输出:
# This is a public method.
# This is a private method.

# 无法直接调用私有方法
obj._private_method()  # 报错: AttributeError: 'MyClass' object has no attribute '__private_method'

# 尽管不能直接访问和调用私有方法，但可以通过其他公有方法内部调用私有方法。
# 需要注意的是，Python中的私有方法并非真正的私有，只是约定俗成的一种规范。实际上，可以通过特定的方式间接访问和调用私有方法，但并不推荐这样做，因为私有方法通常用于内部实现，不应被外部直接使用。

