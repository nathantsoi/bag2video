#!/usr/bin/env python3

from __future__ import division
import rosbag, rospy, numpy as np
import sys, os, cv2, glob
from itertools import repeat
import imageio
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

def get_sizes(bag, topics=None, index=0, scale=1.0, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize)):
    sizes = []

    for topic in topics:
        try:
            iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)#, raw=True)
            msg = next(iterator)[1]
            sizes.append((msg.width, msg.height))
        except:
            logging.critical("No messages found for topic %s, or message does not have height/width." % topic)
            traceback.print_exc()
            sys.exit(1)

    target_height = int(sizes[index][0]*scale)

    # output original and scaled sizes
    for i in range(len(topics)):
        logging.info('Topic %s originally of height %s and width %s' % (topics[i],sizes[i][0],sizes[i][1]))
        image_height = sizes[i][1]
        image_width = sizes[i][0]
        sizes[i] = (int(1.0*image_width*target_height/image_height),target_height)
        logging.info('Topic %s rescaled to height %s and width %s.' % (topics[i],sizes[i][0],sizes[i][1]))

    return sizes

def get_frequency(bag,topics=None, start_time=rospy.Time(0),stop_time=rospy.Time(sys.maxsize)):
    info = bag.get_type_and_topic_info(topics)
    logging.debug(info)

    frequency = 0
    for topic in topics:
        topic_frequency = info[1][topic][3]
        logging.info("Topic %s has a frequency of %s."%(topic,topic_frequency))
        frequency = max(frequency, topic_frequency)

    assert frequency>0
    return frequency

def calc_out_size(sizes):
    return (sum(size[0] for size in sizes),sizes[0][1])

def merge_images(images, sizes):
    return cv2.hconcat([cv2.resize(images[i],sizes[i]) for i in range(len(images))])

def write_frames(bag, writer, topics, sizes, fps, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxsize), viz=False, encoding='bgr8'):
    bridge = CvBridge()
    convert = { topics[i]:i for i in range(len(topics))}
    frame_duration = 1.0/fps

    images = [np.zeros((sizes[i][1],sizes[i][0],3), np.uint8) for i in range(len(topics))]
    frame_num = 0
    count = 1

    iterator = bag.read_messages(topics=topics, start_time=start_time, end_time=stop_time)

    topic, msg, t = next(iterator)
    image = np.asarray(bridge.imgmsg_to_cv2(msg, 'bgr8'))
    images[convert[topic]] = image
    frame_num = int(t.to_sec()/frame_duration)

    for topic, msg, t in iterator:

        frame_num_next = int(t.to_sec()/frame_duration)
        reps = frame_num_next-frame_num

        logging.debug('Topic %s updated at rostime %s, frame %s.' % (topic, t, frame_num_next))

        # prevent unnecessary calculations
        if reps>0:
            # record the current information up to this point in time
            logging.info('Writing image %s at frame %s for %s frames.' % (count, frame_num, reps))
            merged_image = merge_images(images, sizes)
            for i in range(reps):
                writer.write(merged_image) # opencv
                # writer.append_data(merged_image) # imageio
            imshow('win', merged_image)
            frame_num = frame_num_next
            count += 1

        image = np.asarray(bridge.imgmsg_to_cv2(msg, 'bgr8'))
        images[convert[topic]] = image

def imshow(win, img):
    logging.debug("Window redrawn.")
    cv2.imshow(win, img)
    cv2.waitKey(1)

def noshow(win, img):
    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract and encode video from bag files.')
    parser.add_argument('bagfile', help='Specifies the location of the bag file.')
    parser.add_argument('topics',nargs='+',help='Image topics to merge in output video.')
    parser.add_argument('--index', '-i', action='store',default=0, type=int,
                        help='Resizes all images to match the height of the topic specified.')
    parser.add_argument('--scale', '-x', action='store',default=1, type=float,
                        help='Global scale for all images.')
    parser.add_argument('--outfile', '-o', action='store', default=None,
                        help='Destination of the video file. Defaults to the location of the input file.')
    parser.add_argument('--fps', '-f', action='store', default=None, type=float,
                        help='FPS of the output video.')
    parser.add_argument('--viz', '-v', action='store_true', help='Display frames in a GUI window.')
    parser.add_argument('--start', '-s', action='store', default=0, type=float,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store', default=sys.maxsize, type=float,
                        help='Rostime representing where to stop in the bag.')
    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8'), default='bgr8',
                        help='Encoding of the deserialized image.')
    parser.add_argument('--log', '-l',action='store',default='INFO',
                        help='Logging level.')

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
        logging.info('Proccessing bag %s.'% bagfile)
        outfile = args.outfile
        if not outfile:
            outfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '.avi'
        bag = rosbag.Bag(bagfile, 'r')

        fps = args.fps
        if not fps:
            logging.info('Calculating ideal output framerate.')
            fps = get_frequency(bag, args.topics, start_time, stop_time)
            logging.info('Output framerate of %.3f.'%fps)
        else:
            logging.info('Using manually set framerate of %.3f.'%fps)

        logging.info('Calculating video sizes.')
        sizes = get_sizes(bag, topics=args.topics, index=args.index,scale = args.scale, start_time=start_time, stop_time=stop_time)

        logging.info('Calculating final image size.')
        out_width, out_height = calc_out_size(sizes)
        logging.info('Resulting video of width %s and height %s.'%(out_width,out_height))

        logging.info('Opening video writer.')
        fourcc = cv2.VideoWriter_fourcc('M','J','P','G') # opencv
        writer = cv2.VideoWriter(outfile, fourcc, fps, (out_width,out_height)) # opencv
        # writer = imageio.get_writer(outfile, fps=fps, mode='I', format="FFMPEG", macro_block_size=1) # imageio

        logging.info('Writing video at %s.'% outfile)
        write_frames(bag=bag, writer=writer, topics=args.topics, sizes=sizes, fps=fps, start_time=start_time, stop_time=stop_time, encoding=args.encoding)
        writer.release() # opencv
        # writer.close() # imageio

        logging.info('Done.')
    
