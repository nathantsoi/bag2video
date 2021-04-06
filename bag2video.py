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
    logging.debug("Resizing height to topic %s (index %d)." % (topics[index] , index))
    sizes = []

    for topic in topics:
        try:
            iterator = bag.read_messages(topics=topic, start_time=start_time, end_time=stop_time)#, raw=True)
            msg = next(iterator)[1] # read one message
        except:
            logging.critical("No messages found for topic %s" % topic)
            traceback.print_exc()
            sys.exit(1)
        try:
            sizes.append((msg.width, msg.height))
        except:
            logging.critical("No width or height on topic %s" % topic)
            if not (args.width and args.height):
                traceback.print_exc()
                sys.exit(1)
            sizes.append((args.width, args.height))

    target_height = int(sizes[index][1]*scale)

    # output original and scaled sizes
    for i in range(len(topics)):
        logging.info('Topic %s originally of height %s and width %s' % (topics[i],sizes[i][0],sizes[i][1]))
        image_height = sizes[i][1]
        image_width = sizes[i][0]

        # rescale to desired height while keeping aspect ratio
        sizes[i] = (int(1.0*image_width*target_height/image_height),target_height)
        logging.info('Topic %s rescaled to height %s and width %s.' % (topics[i],sizes[i][0],sizes[i][1]))

    return sizes

def get_frequency(bag,topics=None, start_time=rospy.Time(0),stop_time=rospy.Time(sys.maxsize)):
    info = bag.get_type_and_topic_info(topics)
    logging.debug(info)

    # uses the highest topic message frequency as framerate
    frequency = 0
    for topic in topics:
        topic_frequency = info[1][topic][3] # returns the reciprocal of the median difference in timestamps for the topic
        logging.info("Topic %s has a frequency of %s."%(topic,topic_frequency))
        if topic_frequency is not None:
            frequency = max(frequency, topic_frequency)

    try:
        assert frequency>0
    except:
        logging.critical("Unable to calculate framerate from topic frequency.")
        logging.critical("May be caused by a lack of messages.")
        traceback.print_exc()
        sys.exit(1)

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
    if topic.endswith('/compressed'):
        image = np.asarray(bridge.compressed_imgmsg_to_cv2(msg, encoding))
    else:
        image = np.asarray(bridge.imgmsg_to_cv2(msg, encoding))
    images[convert[topic]] = image
    frame_num = int(t.to_sec()/frame_duration)

    for topic, msg, t in iterator:

        time=t.to_sec()

        frame_num_next = int(time/frame_duration)
        reps = frame_num_next-frame_num

        logging.debug('Topic %s updated at time %s seconds, frame %s.' % (topic, time, frame_num_next))

        # prevent unnecessary calculations
        if reps>0:
            # record the current information up to this point in time
            logging.debug('Writing image %s at time %.6f seconds, frame %s for %s frames.' % (count, time, frame_num, reps))
            merged_image = merge_images(images, sizes)
            for i in range(reps):
                #writer.write(merged_image) # opencv
                writer.append_data(merged_image) # imageio
            imshow('win', merged_image)
            frame_num = frame_num_next
            count += 1

        if topic.endswith('/compressed'):
            image = np.asarray(bridge.compressed_imgmsg_to_cv2(msg, encoding))
        else:
            image = np.asarray(bridge.imgmsg_to_cv2(msg, encoding))
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
    parser.add_argument('topics', nargs='+',help='Image topics to merge in output video.')
    parser.add_argument('--index', '-i', action='store',default=0, type=int,
                        help='Resizes all images to match the height of the topic specified. Default 0.')
    parser.add_argument('--height', action='store',default=0, type=int,
                        help='input height, required if not specified on the topic')
    parser.add_argument('--width', action='store',default=0, type=int,
                        help='input width, required if not specified on the topic')
    parser.add_argument('--scale', '-x', action='store',default=1, type=float,
                        help='Global scale for all images. Default 1.')
    parser.add_argument('--outfile', '-o', action='store', default=None,
                        help='Destination of the video file. Defaults to the folder of the bag file.')
    parser.add_argument('--fps', '-f', action='store', default=None, type=float,
                        help='FPS of the output video. If not specified, FPS will be set to the maximum frequency of the topics.')
    parser.add_argument('--viz', '-v', action='store_true', help='Display frames in a GUI window.')
    parser.add_argument('--start', '-s', action='store', default=0, type=float,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store', default=sys.maxsize, type=float,
                        help='Rostime representing where to stop in the bag or duration when used with --stop_topic.')
    parser.add_argument('--stop_topic', action='store', default='',
                        help='A boolean topic. Recording stops when the topic is false. When used with --end, --end becomes the debounce time for --stop_topic in float seconds.')
    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8'), default='rgb8',
                        help='Encoding of the deserialized image. Default rgb8.')
    parser.add_argument('--fourcc', '-c', action='store', default='MJPG',
                        help='Specifies FourCC for the output video. Default MJPG.')
    parser.add_argument('--log', '-l',action='store',default='INFO',
                        help='Logging level. Default INFO.')

    args = parser.parse_args()

    # logging setup
    numeric_level = getattr(logging, args.log.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError('Invalid log level: %s' % loglevel)
    logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s',level=numeric_level)
    logging.info('Logging at level %s.',args.log.upper())
    if not args.viz:
        imshow = noshow

    # convert numbers into rospy Time
    start_time=rospy.Time(args.start)
    stop_time=rospy.Time(args.end)

    try:
        assert start_time <= stop_time
    except:
        logging.critical("Start time is after stop time.")
        traceback.print_exc()
        sys.exit(1)

    try:
        assert args.index < len(args.topics)
    except:
        logging.critical("Index specified for resizing is out of bounds.")
        traceback.print_exc()
        sys.exit(1)

    for bagfile in glob.glob(args.bagfile):
        logging.info('Proccessing bag %s.'% bagfile)
        outfile = args.outfile
        if outfile is None:
            folder, name = os.path.split(bagfile)
            outfile = os.path.join(folder, name[:name.rfind('.')]) + '.mp4'
        bag = rosbag.Bag(bagfile, 'r')

        if args.stop_topic != '':
            logging.info('Calculating stop_time based on stop_topic: {}.'.format(args.stop_topic))
            debounce_duration = rospy.Duration.from_sec(15.0)
            if args.end != sys.maxsize:
                debounce_duration = rospy.Duration.from_sec(args.end)
            debounce_time = start_time + debounce_duration
            ts = None
            for topic, msg, ts in bag.read_messages(topics=[args.stop_topic], start_time=start_time):
                stop_time = ts
                if msg.data:
                    if ts > debounce_time:
                        break
                else:
                    debounce_time = ts + debounce_duration
            if stop_time == ts:
                stop_time += debounce_duration
            logging.info('stop_time: {}, last ts: {}.'.format(stop_time, ts))

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
        #fourcc = cv2.VideoWriter_fourcc(*args.fourcc) # opencv
        #writer = cv2.VideoWriter(outfile, fourcc, fps, (out_width,out_height)) # opencv
        #writer = imageio.get_writer(outfile, format='FFMPEG', mode='I', fps=fps, codec='h264' )
        writer = imageio.get_writer(outfile, mode='I', fps=fps)
        #writer = imageio.get_writer(outfile, fps=fps, mode='I', format="FFMPEG", macro_block_size=1) # imageio

        logging.info('Writing video at %s.'% outfile)
        write_frames(bag=bag, writer=writer, topics=args.topics, sizes=sizes, fps=fps, start_time=start_time, stop_time=stop_time, encoding=args.encoding)
        #writer.release() # opencv
        writer.close() # imageio

        logging.info('Done.')
