#!/usr/bin/env python3
if __name__ == '__main__':
    with open('/home/wangdak/catkin_ws/src/function/points/waypoints.csv') as f, open('/home/wangdak/catkin_ws/src/function/points/waypoints_dake.csv', 'w') as fout:
        fout.writelines(reversed(f.readlines()))
