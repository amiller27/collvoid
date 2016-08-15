import json
import matplotlib.pyplot as plt
import numpy as np
import rospy
from std_msgs.msg import String
from sys import argv
import threading
import time
from visualization_msgs.msg import Marker
import Queue

def lines_callback(data):
    lines_queue.put(data)

metadata_lock = threading.Lock()
def metadata_callback(data):
    next_obj = json.loads(data.data)
    metadata_lock.acquire()
    if not metadata_list or metadata_list[-1]['seq'] < next_obj['seq']:
        metadata_list.append(next_obj)
    else:
        i = len(metadata_list) - 2
        inserted = False
        while i >= 0:
            if metadata_list[i]['seq'] > next_obj['seq']:
                i -= 1
            elif metadata_list[i]['seq'] < next_obj['seq']:
                metadata_list.insert(i+1, next_obj)
                inserted = True
                break
            else:
                # they are equal
                metadata_list[i].update(next_obj)
                inserted = True
                break
        if not inserted:
            metadata_list.insert(0, next_obj)
    metadata_lock.release()

def draw(data, metadata):
    plt.clf()
    for p1, p2, line in zip(data.points[::2], data.points[1::2], metadata['data']):
        x1 = p1.x
        y1 = p1.y
        x2 = p2.x
        y2 = p2.y
        x = np.linspace(-5, 5) # np.linspace(x1, x2)
        y = (y2 - y1) / (x2 - x1) * x + y1
        last_line, = plt.plot(x, y)
        y_fill_to = y + (0.3 if line['dir'] else -0.3)
        plt.fill_between(x, y, y_fill_to, facecolor=last_line.get_color(), alpha=0.2)

    plt.axis([-5, 5, -5, 5])
    plt.draw()

if __name__ == '__main__':
    namespace = argv[1] if len(argv) > 1 else '/robot_0'
    rospy.init_node('orca_listener')
    rospy.Subscriber(namespace + '/orca_lines', Marker, lines_callback)
    rospy.Subscriber(namespace + '/orca_metadata', String, metadata_callback)

    plt.ion()

    lines_queue = Queue.Queue()
    metadata_list = []

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        metadata_lock.acquire()
        while not lines_queue.empty():
            next_line = lines_queue.get()


        while not lines_queue.empty() and metadata_list:
            next_lines = lines_queue.get()
            while metadata_list and ('timestamp' not in metadata_list[0] or metadata_list[0]['timestamp'] < next_lines['header']['stamp']):
                metadata_list.pop(0)

            draw(lines_queue.get(), metadata_list.pop(0))
        metadata_lock.release()
        plt.pause(0.001)
        rate.sleep()
