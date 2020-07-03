#!/usr/bin/env python3

from __future__ import division
import rosbag, rospy, numpy as np
import sys, os, cv2, glob
from itertools import repeat
import argparse
import logging
import traceback

# try to find cv_bridge:
try:
    from cv_bridge import CvBridge
except ImportError:
    # assume we are on an older ROS version, and try loading the dummy manifest
    # to see if that fixes the import error
    try:
        import roslib; roslib.load_manifest("bag2video")
        from cv_bridge import CvBridge
    except Exception:
        logging.critical("Could not find ROS package: cv_bridge.")
        logging.critical("If ROS version is pre-Groovy, try putting this package in ROS_PACKAGE_PATH.")
        traceback.print_exc()
        sys.exit(1)

def get_info(bag, topic=None, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize)):
    logging.info("Gathering info about topic %s."%topic)
    size = (0,0)
    times = []

    # read the first message to get the image size
    msg = next(bag.read_messages(topics=topic))[1]
    size = (msg.width, msg.height)

    # now read the rest of the messages for the timestamps
    iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)#, raw=True)
    for _, msg, _ in iterator:
        time = msg.header.stamp
        times.append(time.to_sec())
    if len(times)<2:
        logging.warning("Less than two timestamps found.")
    if size == (0,0):
        logging.warning("Either no images, or images of size (0,0).")

    logging.debug("Start time %s." % times[0])
    logging.debug("End time %s." % times[-1])

    return size, times

def calc_n_frames(times, fps=30.0,start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize)):
    try:
        # adds fake timestamps to the ends so the first and last images don't get their duration cut in half
        times = np.insert(times,0,2*times[0]-times[1])
        times = np.append(times,2*times[-1]-times[-2])

        # converts timestamps to frame numbers of final video
        rate = 1.0/fps
        frames = [int(time/rate) for time in times]

        # converts frame numbers to duration each image is displayed
        num_frames = [frames[i+1]-frames[i] for i in range(0,len(frames)-2)]
    except IndexError:
        logging.critical("IndexError when calculating number of frames each image is displayed for.")
        logging.critical("Are there less than two timestamps?")
        traceback.print_exc()
        sys.exit(1)
    return num_frames

def write_frames(bag, writer, total, topic=None, num_frames=repeat(1), start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize), viz=False, encoding='bgr8'):
    bridge = CvBridge()
    iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)
    count=1
    for (topic, msg, time), reps in zip(iterator, num_frames):
        logging.debug('Writing image %s of %s at time %s for %s frames.' % (count, total, time, reps))
        img = np.asarray(bridge.imgmsg_to_cv2(msg, 'bgr8'))
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
    parser.add_argument('--fps', '-f', action='store', default=30, type=int,
                        help='FPS of the output video.')
    parser.add_argument('--viz', '-v', action='store_true', help='Display frames in a GUI window.')
    parser.add_argument('--start', '-s', action='store', default=0, type=float,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store', default=sys.maxsize, type=float,
                        help='Rostime representing where to stop in the bag.')
    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8'), default='bgr8',
                        help='Encoding of the deserialized image.')
    parser.add_argument('--log', '-l',action='store',default='INFO',help='Logging level.')
    parser.add_argument('bagfile', help='Specifies the location of the bag file. Works with glob.')
    parser.add_argument('topic',nargs='+',help='Image topics to merge in output video.')

    args = parser.parse_args()

    numeric_level = getattr(logging, args.log.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)
    logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s',level=numeric_level)
    logging.info('Logging at level %s.',args.log.upper())
    if not args.viz:
        imshow = noshow

    start_time=rospy.Time(args.start)
    stop_time=rospy.Time(args.end)

    for bagfile in glob.glob(args.bagfile):
        logging.info(bagfile)
        outfile = args.outfile
        if not outfile:
            outfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '.avi'
        bag = rosbag.Bag(bagfile, 'r')
        logging.info('Calculating video properties.')
        size, times = get_info(bag, args.topic, start_time=start_time, stop_time=stop_time)
        logging.info('Calculating frame durations.')
        num_frames = calc_n_frames(times, args.fps)
        # writer = cv2.VideoWriter(outfile, cv2.cv.CV_FOURCC(*'DIVX'), rate, size)
        logging.info('Opening video writer.')
        fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
        writer = cv2.VideoWriter(outfile, fourcc, np.ceil(args.fps), size)
        logging.info('Writing video.')
        write_frames(bag, writer, len(times), topic=args.topic, num_frames=num_frames, start_time=start_time, stop_time=stop_time, encoding=args.encoding)
        writer.release()
        logging.info('Done.')
