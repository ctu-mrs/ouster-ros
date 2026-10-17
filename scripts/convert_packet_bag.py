#!/usr/bin/env python3
"""
Convert a bag with the old ouster_ros/PacketMsg (only `uint8[] buf`) to the current
PacketMsg (`time rcv_stamp` + `uint8[] buf`) so that it can be replayed by this driver.

The record time of each packet is used as its rcv_stamp. All other topics are copied
unchanged (including their connection headers, so latched topics such as /tf_static
stay latched). Optionally, only a time window of the input bag is converted - /tf_static
messages from before the window are then copied to its start, as they are only published once.

usage: rosrun ouster_ros convert_packet_bag.py input.bag output.bag [--start S] [--end S]
"""
import argparse
import struct
import sys

import rosbag
import rospy
from ouster_ros.msg import PacketMsg

OLD_PACKET_MD5 = '4f7b5949e76f86d01e96b0e33ba9b5e3'  # PacketMsg with `uint8[] buf` only


def old_packet_buf(data):
    """deserialize the old PacketMsg by hand (uint32 length + bytes) - its class no longer exists"""
    (length,) = struct.unpack_from('<I', data, 0)
    return bytes(data[4:4 + length])


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('input', help='input bag with old-style ouster_ros/PacketMsg')
    parser.add_argument('output', help='output bag')
    parser.add_argument('--start', type=float, default=None, help='start of the converted window [s from the start of the bag]')
    parser.add_argument('--end', type=float, default=None, help='end of the converted window [s from the start of the bag]')
    parser.add_argument('--compression', choices=['none', 'bz2', 'lz4'], default='none', help='compression of the output bag')
    args = parser.parse_args()

    if PacketMsg._md5sum == OLD_PACKET_MD5:
        sys.exit('The ouster_ros/PacketMsg in the sourced workspace is the old one - nothing to convert to.')

    with rosbag.Bag(args.input) as inb:
        bag_start = inb.get_start_time()
        start_time = rospy.Time.from_sec(bag_start + args.start) if args.start is not None else None
        end_time = rospy.Time.from_sec(bag_start + args.end) if args.end is not None else None

        n_packets = n_other = 0
        packet_topics = set()
        with rosbag.Bag(args.output, 'w', compression=args.compression) as outb:
            if start_time is not None:
                for _, raw_msg, _, conn_header in inb.read_messages(topics=['/tf_static'], end_time=start_time, raw=True, return_connection_header=True):
                    outb.write('/tf_static', raw_msg, start_time, raw=True, connection_header=conn_header)
                    n_other += 1
            msgs = inb.read_messages(start_time=start_time, end_time=end_time, raw=True, return_connection_header=True)
            for topic, raw_msg, t, conn_header in msgs:
                datatype, data, md5sum = raw_msg[:3]
                if datatype == 'ouster_ros/PacketMsg' and md5sum == OLD_PACKET_MD5:
                    packet_topics.add(topic)
                    msg = PacketMsg()
                    msg.rcv_stamp = t
                    msg.buf = old_packet_buf(data)
                    outb.write(topic, msg, t)
                    n_packets += 1
                else:
                    outb.write(topic, raw_msg, t, raw=True, connection_header=conn_header)
                    n_other += 1

    if not packet_topics:
        print(f'WARNING: no old-style ouster_ros/PacketMsg found in {args.input} (in the selected window), the output is just a copy.')
    else:
        print(f'converted topics: {", ".join(sorted(packet_topics))}')
    print(f'wrote {n_packets} converted packets and {n_other} other messages to {args.output}')


if __name__ == '__main__':
    main()
