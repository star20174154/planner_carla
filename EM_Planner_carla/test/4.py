# -*- coding: utf-8 -*-
# @Time    : 2023/8/15 10:45
# @Author  : Star
# @File    : 4.py.py
# @Software: PyCharm
import multiprocessing

def worker(conn):
    data = conn.recv()
    print("Received:", data)

if __name__ == "__main__":
    parent_conn, child_conn = multiprocessing.Pipe()

    p = multiprocessing.Process(target=worker, args=(child_conn,))
    p.start()

    parent_conn.send("Hello from parent!")
    p.join()
