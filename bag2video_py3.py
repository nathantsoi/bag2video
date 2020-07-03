#!/usr/bin/env python3

from __future__ import division
import rosbag, rospy, numpy as np
import sys, os, cv2, glob
from itertools import repeat
import argparse

import imageio

# pip install --ignore-installed  --extra-index-url https://rospypi.github.io/simple/ rospy rosbag roslz4 cv_bridge sensor_msgs geometry_msgs

# try to find cv_bridge:
try:
    from cv_bridge import CvBridge
except ImportError:
    # assume we are on an older ROS version, and try loading the dummy manifest
    # to see if that fixes the import error
    try:
        import roslib; roslib.load_manifest("bag2video")
        from cv_bridge import CvBridge
    except:
        print("Could not find ROS package: cv_bridge")
        print("If ROS version is pre-Groovy, try putting this package in ROS_PACKAGE_PATH")
        sys.exit(1)

def get_info(bag, topic=None, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize)):
    size = (0,0)
    times = []

    # read the first message to get the image size
    g = bag.read_messages(topics=topic)
    _, f, _ = next(g)
    size = (f.width, f.height)

    with open("times.txt", 'w') as fp:

        # now read the rest of the messages for the rates
        iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)#, raw=True)
        for _, msg, _ in iterator:

            time = msg.header.stamp

            fp.write("{:f}\n".format(time.to_sec()))

            times.append(time.to_sec())
            size = (msg.width, msg.height)
        diffs = 1/np.diff(times)
    return np.median(diffs), min(diffs), max(diffs), size, times

def calc_n_frames(times, precision=10):
    # the smallest interval should be one frame, larger intervals more
    intervals = np.diff(times)
    return np.int64(np.round(precision*intervals/min(intervals)))

def write_frames(bag, writer, total, topic=None, nframes=repeat(1), start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize), viz=False, encoding='bgr8'):
    bridge = CvBridge()
    if viz:
        cv2.namedWindow('win')
    count = 1
    iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)
    for (topic, msg, time), reps in zip(iterator, nframes):
        print('Writing frame %s of %s at time %s' % (count, total, time))
        img = np.asarray(bridge.imgmsg_to_cv2(msg, encoding))
        for rep in range(reps):
            writer.write(img)
        imshow('win', img)
        count += 1

def imshow(win, img):
    cv2.imshow(win, img)
    cv2.waitKey(1)

def noshow(win, img):
    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract and encode video from bag files.')
    parser.add_argument('--outfile', '-o', action='store', default=None,
                        help='Destination of the video file. Defaults to the location of the input file.')
    parser.add_argument('--precision', '-p', action='store', default=10, type=int,
                        help='Precision of variable framerate interpolation. Higher numbers\
                        match the actual framerater better, but result in larger files and slower conversion times.')
    parser.add_argument('--viz', '-v', action='store_true', help='Display frames in a GUI window.')
    parser.add_argument('--start', '-s', action='store', default=rospy.Time(0), type=rospy.Time,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store', default=rospy.Time(sys.maxsize), type=rospy.Time,
                        help='Rostime representing where to stop in the bag.')

    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8'), default='bgr8',
                        help='Encoding of the deserialized image.')

    parser.add_argument('--meanfps', action='store_true', help='Set a constant fps')

    parser.add_argument("--imageio", action='store_true')

    parser.add_argument('topic')
    parser.add_argument('bagfile')

    args = parser.parse_args()

    if not args.viz:
        imshow = noshow

    for bagfile in glob.glob(args.bagfile):
        print( bagfile )
        outfile = args.outfile
        if not outfile:
            outfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '.mp4'
        bag = rosbag.Bag(bagfile, 'r')
        print('Calculating video properties')

        rate, minrate, maxrate, size, times = get_info(bag, args.topic, start_time=args.start, stop_time=args.end)

        print("Rate: {:f}; min rate: {:f}, maxrate: {:f}".format(rate,minrate,maxrate))

        if args.imageio:
            print("Creating move with imageio")

            w = imageio.get_writer(outfile, format='FFMPEG', mode='I', fps=rate,
                                   codec='h264' )

            bridge = CvBridge()

            count = 1
            nframes = repeat(1)

            iterator = bag.read_messages(topics=args.topic, start_time=args.start, end_time=args.end)

            for (topic, msg, time), reps in zip(iterator, nframes):

                print ('Writing frame %s of %s at time %s' % (count, len(times), time))
                img = np.asarray(bridge.imgmsg_to_cv2(msg, 'rgb8'))

                for rep in range(reps):
                    w.append_data(img)

                count += 1

            w.close()


        elif args.meanfps:

            print("Creating movie with constant {:f} fps".format(rate))
            writer = cv2.VideoWriter(outfile, cv2.VideoWriter_fourcc(*'avc1'), rate, size)
            write_frames(bag, writer, len(times), topic=args.topic, start_time=args.start, stop_time=args.end, encoding=args.encoding)
            writer.release()


        else:
            nframes = calc_n_frames(times, args.precision)
            # writer = cv2.VideoWriter(outfile, cv2.cv.CV_FOURCC(*'DIVX'), rate, size)
            writer = cv2.VideoWriter(outfile, cv2.VideoWriter_fourcc(*'avc1'), np.ceil(maxrate*args.precision), size)

            write_frames(bag, writer, len(times), topic=args.topic, nframes=nframes, start_time=args.start, stop_time=args.end, encoding=args.encoding)
            writer.release()

        print('\n')
