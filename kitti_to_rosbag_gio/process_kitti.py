import itertools
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

import pykitti

import tf
import os
import rospy
import rosbag
import progressbar
from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from cv_bridge import CvBridge
import numpy as np
import argparse

def test_point_format(dataset):
    # 选取姿态和点云
    choose_pose = dataset.oxts[1].T_w_imu
    choose_cloud = dataset.get_velo(6)

    # 显示信息
    np.set_printoptions(precision=4, suppress=True)
    print('\nDrive: ' + str(dataset.drive))
    print('\nFrame range: ' + str(dataset.frames))

    print('\nIMU-to-Velodyne transformation:\n' + str(dataset.calib.T_velo_imu))

    print('\nFirst timestamp: ' + str(dataset.timestamps[0]))
    print('\nSecond IMU pose:\n' + str(choose_pose))

    # 显示点云
    f1 = plt.figure()
    ax1 = f1.add_subplot(111, projection='3d')
    # 点云下采样
    velo_range = range(0, choose_cloud.shape[0], 10)
    
    colors = np.array(velo_range)
    ax1.scatter(choose_cloud[velo_range, 0],
                choose_cloud[velo_range, 1],
                choose_cloud[velo_range, 2],
                c=colors,
                cmap='gray')
    ax1.set_title('Velodyne scan')

    ## 显示点云中每个点的位置关系
    x = choose_cloud[velo_range, 0]
    y = choose_cloud[velo_range, 1]
    z = choose_cloud[velo_range, 2]

    # 显示点绕z轴角度关系
    theta = np.degrees(np.arctan2(y, x))
    theta = (theta + 360) % 360
    f2 = plt.figure(figsize=(10, 4))
    ax2 = f2.add_subplot(111)  
    ax2.plot(velo_range, theta, linewidth=0.8)
    ax2.set_xlabel('Index')
    ax2.set_ylabel('Theta (degrees)')
    ax2.set_title('Angle of point (x, y) to X-axis')
    ax2.grid(True)
    f2.tight_layout()

    # 显示点pitch
    pitch = np.arctan2(z, np.sqrt(x**2 + y**2))
    pitch = np.degrees(pitch)
    f3 = plt.figure(figsize=(10, 4))
    ax3 = f3.add_subplot(111)
    ax3.plot(velo_range, pitch, linewidth=0.8)
    ax3.set_xlabel('Index')
    ax3.set_ylabel('Pitch (degrees)')
    ax3.set_title('Pitch angle of point cloud')
    ax3.grid(True)
    plt.tight_layout()

    # 显示pitch和theta关系
    plt.figure(figsize=(10, 5))
    plt.scatter(theta, pitch, s=1, alpha=0.5,c=colors)
    plt.xlabel("Theta (°)")
    plt.ylabel("Pitch (°)")
    plt.title("Pitch vs. Theta")


    ## 验证分配环数
    # 计算相邻点角度变化量
    d_theta = np.diff(theta)
    jump_indices = np.where(d_theta < -180)[0] + 1 
    MAX_RING = 63
    # 如果超过线数，只保留64条线
    if len(jump_indices) > MAX_RING:
        jump_indices = jump_indices[:MAX_RING]
    # 构造每个点的 ring 编号数组
    rings = np.zeros_like(theta, dtype=int)

    # 分段赋值 ring
    start_idx = 0
    for ring_id, end_idx in enumerate(np.append(jump_indices, len(theta))):
        rings[start_idx:end_idx] = min(ring_id, MAX_RING)
        start_idx = end_idx

    num_points_per_ring = np.bincount(rings, minlength=MAX_RING+1)
    print('每个ring的点个数'+str(num_points_per_ring))

    f5 = plt.figure(figsize=(10, 4))
    ax5 = f5.add_subplot(111)
    ax5.plot(velo_range, rings, linewidth=0.8)
    ax5.set_xlabel('Index')
    ax5.set_ylabel('rings')
    ax5.set_title('rings')
    ax5.grid(True)
    plt.tight_layout()

    plt.show()

def get_static_transform(from_frame_id, to_frame_id, transform):
    t = transform[0:3, 3]
    q = tf.transformations.quaternion_from_matrix(transform)
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = from_frame_id
    tf_msg.child_frame_id = to_frame_id
    tf_msg.transform.translation.x = float(t[0])
    tf_msg.transform.translation.y = float(t[1])
    tf_msg.transform.translation.z = float(t[2])
    tf_msg.transform.rotation.x = float(q[0])
    tf_msg.transform.rotation.y = float(q[1])
    tf_msg.transform.rotation.z = float(q[2])
    tf_msg.transform.rotation.w = float(q[3])
    return tf_msg

def save_static_transforms(bag, transforms, timestamps):
    print("Exporting static transformations")
    tfm = TFMessage()
    for transform in transforms:
        t = get_static_transform(from_frame_id=transform[0], to_frame_id=transform[1], transform=transform[2])
        tfm.transforms.append(t)
    for timestamp in timestamps:
        time = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        for i in range(len(tfm.transforms)):
            tfm.transforms[i].header.stamp = time
        bag.write('/tf_static', tfm, t=time)

def save_dynamic_tf(bag, kitti):
    print("Exporting time dependent transformations")
    
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        tf_oxts_msg = TFMessage()
        tf_oxts_transform = TransformStamped()
        tf_oxts_transform.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        tf_oxts_transform.header.frame_id = 'world'
        tf_oxts_transform.child_frame_id = 'base_link'

        transform = (oxts.T_w_imu)
        t = transform[0:3, 3]
        q = tf.transformations.quaternion_from_matrix(transform)
        oxts_tf = Transform()

        oxts_tf.translation.x = t[0]
        oxts_tf.translation.y = t[1]
        oxts_tf.translation.z = t[2]

        oxts_tf.rotation.x = q[0]
        oxts_tf.rotation.y = q[1]
        oxts_tf.rotation.z = q[2]
        oxts_tf.rotation.w = q[3]

        tf_oxts_transform.transform = oxts_tf
        tf_oxts_msg.transforms.append(tf_oxts_transform)

        bag.write('/tf', tf_oxts_msg, tf_oxts_msg.transforms[0].header.stamp)

def save_imu_data(bag, kitti, imu_frame_id, topic):
    print("Exporting IMU")
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        q = tf.transformations.quaternion_from_euler(oxts.packet.roll, oxts.packet.pitch, oxts.packet.yaw)
        imu = Imu()
        imu.header.frame_id = imu_frame_id
        imu.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        imu.linear_acceleration.x = oxts.packet.ax
        imu.linear_acceleration.y = oxts.packet.ay
        imu.linear_acceleration.z = oxts.packet.az
        imu.angular_velocity.x = oxts.packet.wx
        imu.angular_velocity.y = oxts.packet.wy
        imu.angular_velocity.z = oxts.packet.wz
        bag.write(topic, imu, t=imu.header.stamp)

def save_gps_fix_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header.frame_id = gps_frame_id
        navsatfix_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        navsatfix_msg.latitude = oxts.packet.lat
        navsatfix_msg.longitude = oxts.packet.lon
        navsatfix_msg.altitude = oxts.packet.alt
        navsatfix_msg.status.service = 1
        bag.write(topic, navsatfix_msg, t=navsatfix_msg.header.stamp)


def save_gps_vel_data(bag, kitti, gps_frame_id, topic):
    for timestamp, oxts in zip(kitti.timestamps, kitti.oxts):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = gps_frame_id
        twist_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
        twist_msg.twist.linear.x = oxts.packet.vf
        twist_msg.twist.linear.y = oxts.packet.vl
        twist_msg.twist.linear.z = oxts.packet.vu
        twist_msg.twist.angular.x = oxts.packet.wf
        twist_msg.twist.angular.y = oxts.packet.wl
        twist_msg.twist.angular.z = oxts.packet.wu
        bag.write(topic, twist_msg, t=twist_msg.header.stamp)

def save_velo_data(bag, kitti, velo_frame_id, topic):
    print("Exporting velodyne data")
    velo_path = os.path.join(kitti.data_path, 'velodyne_points')
    velo_data_dir = os.path.join(velo_path, 'data')
    velo_filenames = sorted(os.listdir(velo_data_dir)) # 文件名
    with open(os.path.join(velo_path, 'timestamps.txt')) as f:
        lines = f.readlines()
        velo_datetimes = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line.strip(), '%Y-%m-%d %H:%M:%S.%f')
            velo_datetimes.append(dt)

    with open(os.path.join(velo_path, 'timestamps_start.txt')) as f:
        lines = f.readlines()
        velo_datetimes_start = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line.strip(), '%Y-%m-%d %H:%M:%S.%f')
            velo_datetimes_start.append(dt)

    with open(os.path.join(velo_path, 'timestamps_end.txt')) as f:
        lines = f.readlines()
        velo_datetimes_end = []
        for line in lines:
            if len(line) == 1:
                continue
            dt = datetime.strptime(line.strip(), '%Y-%m-%d %H:%M:%S.%f')
            velo_datetimes_end.append(dt)

    iterable = zip(velo_datetimes,velo_datetimes_start,velo_datetimes_end, velo_filenames)
    bar = progressbar.ProgressBar()
    for dt,dt_start,dt_end, filename in bar(iterable):
        if dt is None:
            continue

        velo_filename = os.path.join(velo_data_dir, filename)

        # 一帧点云
        scan = np.loadtxt(velo_filename, dtype=np.float32) 
        num_points = scan.shape[0]
        
        x = scan[:, 0]
        y = scan[:, 1] 
        theta = np.degrees(np.arctan2(y, x))
        theta = (theta + 360) % 360 

        ## 定义每个点的 ring
        # 计算相邻点角度变化量
        d_theta = np.diff(theta)
        jump_indices = np.where(d_theta < -180)[0] + 1 
        MAX_RING = 63
        # 如果超过线数，只保留64条线
        if len(jump_indices) > MAX_RING:
            jump_indices = jump_indices[:MAX_RING]
        # 构造每个点的 ring 编号数组
        rings = np.zeros_like(theta, dtype=int)
        # 分段赋值 ring
        start_idx = 0
        for ring_id, end_idx in enumerate(np.append(jump_indices, len(theta))):
            rings[start_idx:end_idx] = min(ring_id, MAX_RING)
            start_idx = end_idx

        # 定义每个点的 time 
        frame_time = dt_end.timestamp() - dt_start.timestamp()
        times = np.zeros_like(rings, dtype=np.float32)

        for ring_id in range(MAX_RING + 1):
            idx = np.where(rings == ring_id)[0]        # 该 ring 的点索引
            n = len(idx)
            if n > 1:
                times[idx] = np.linspace(0, frame_time, n, endpoint=False)
            elif n == 1:
                times[idx] = 0.0

        # 定义复合 dtype
        point_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('time', np.float32),
            ('ring', np.uint16),
            ('padding', np.uint16),
        ])

        # 创建 structured array
        points = np.zeros(num_points, dtype=point_dtype)
        points['x'] = scan[:, 0]
        points['y'] = scan[:, 1]
        points['z'] = scan[:, 2]
        points['intensity'] = (scan[:, 3] * 255.0).clip(0, 255)
        points['time'] = times
        points['ring'] = rings
        points['padding'] = 0  # 填充保证每行 24 字节

        # 创建 PointCloud2 消息
        header = Header()
        header.frame_id = velo_frame_id
        header.stamp = rospy.Time.from_sec(dt_start.timestamp())  # 点云中第一个点
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('time', 16, PointField.FLOAT32, 1),
            PointField('ring', 20, PointField.UINT16, 1),
            PointField('padding', 22, PointField.UINT16, 1),
        ]

        pcl_msg = pcl2.create_cloud(header, fields, points)
        bag.write(topic, pcl_msg, t=header.stamp)

def make_bag(dataset):
    # 创建空包
    compression = rosbag.Compression.NONE
    bag = rosbag.Bag("kitti_{}_drive_{}.bag".format(date, drive), 'w', compression=compression)

    # 定义话题
    imu_frame_id = 'imu_link'
    imu_topic = '/kitti/imu'
    gps_fix_topic = '/kitti/gps/fix'
    gps_vel_topic = '/kitti/gps/vel'
    velo_frame_id = 'velo_link'
    velo_topic = '/kitti/velo'

    T_base_link_to_imu = np.eye(4, 4)
    T_base_link_to_imu[0:3, 3] = [-2.71/2.0-0.05, 0.32, 0.93]
    T_velo_to_imu=dataset.calib.T_velo_imu

    # tf_static
    transforms = [
        ('base_link', imu_frame_id, T_base_link_to_imu),
        (imu_frame_id, velo_frame_id, T_velo_to_imu)
    ]
    save_static_transforms(bag, transforms, dataset.timestamps)

    # tf_dynamic
    save_dynamic_tf(bag, dataset)

    # imu
    save_imu_data(bag, dataset, imu_frame_id, imu_topic)

    # gps
    save_gps_fix_data(bag, dataset, imu_frame_id, gps_fix_topic)
    save_gps_vel_data(bag, dataset, imu_frame_id, gps_vel_topic)

    # lidar
    save_velo_data(bag, dataset, velo_frame_id, velo_topic)

    print("## OVERVIEW ##")
    print(bag)
    bag.close()

if __name__ == "__main__":

    basedir = '/home/gio/slam_ws/src/kitti_raw_to_rosbag'

    date = '2011_09_30'
    drive = '0018'

    # dataset = pykitti.raw(basedir, date, drive)
    dataset = pykitti.raw(basedir, date, drive, dataset='extract')
    # dataset.calib:         Calibration data are accessible as a named tuple
    # dataset.timestamps:    Timestamps are parsed into a list of datetime objects
    # dataset.oxts:          List of OXTS packets and 6-dof poses as named tuples
    # dataset.velo:          Returns a generator that loads velodyne scans as [x,y,z,reflectance]
    # dataset.get_velo(idx): Returns the velodyne scan at idx

    test_point_format(dataset)
    # make_bag(dataset)

