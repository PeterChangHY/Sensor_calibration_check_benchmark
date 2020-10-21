import rosbag
import difflib
import fastbag
from munch import Munch
import os


def buffered_message_generator(bags, topics, tolerance):
    topic_queues = {}
    topic_skips = {}
    topic_counts = {}
    for topic in topics:
        topic_queues[topic] = []
        topic_skips[topic] = 0
        topic_counts[topic] = 0
    for b in bags:
        for topic, msg, timestamp in b.read_messages(topics=topics):
            m = Munch()
            m.topic = topic
            m.message = msg
            m.timestamp = timestamp
            if 'perception' in m.topic:
                # would need to parse the proto...
                current_timestamp = m.timestamp.to_sec()
            else:
                current_timestamp = m.message.header.stamp.to_sec()
                current_timestamp = m.timestamp.to_sec()
            topic_queues[m.topic].append((current_timestamp, m))
            topic_counts[m.topic] += 1

            # assuming that messages do not arrive out of order, we can
            # safely discard anything older than the newest message on the
            # ends of the queues
            newest_tip = 0
            for topic in topics:
                if topic_queues[topic] and topic_queues[topic][0][0] > newest_tip:
                    newest_tip = topic_queues[topic][0][0]
            for topic in topics:
                while topic_queues[topic] and newest_tip - topic_queues[topic][0][0] > tolerance:
                    print(newest_tip - topic_queues[topic][0][0])
                    topic_skips[topic] += 1
                    topic_queues[topic].pop(0)

            # we need at least one message on each topic
            if not all(topic_queues.values()):
                continue

            # at this point, they must be within tolerance and present
            frame = {}
            for topic in topics:
                frame[topic] = topic_queues[topic].pop(0)[1]
            yield frame

    print("skips", topic_skips)
    print("counts", topic_counts)


def load_rosbags_from_files(bag_files):
    bags = []
    for bag_file in bag_files:
        if os.path.splitext(bag_file)[1] == '.db':
            bag = fastbag.Reader(bag_file)
            bag.open()
        elif os.path.splitext(bag_file)[1] == '.bag':
            bag = rosbag.Bag(bag_file)
        else:
            raise RuntimeError
        bags.append(bag)
    return bags


def check_topic_exist_in_bag(ros_bag, topic):
    topics = ros_bag.get_type_and_topic_info().topics.keys()

    if topic in topics:
        return True
    else:
        print('Cannot find topic: %s in bag, maybe use: %s' % (topic, difflib.get_close_matches(topic, topics, n=1)[0]))
        return False
