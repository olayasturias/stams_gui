#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import rospy
import sys
import tf
import numpy as np
import datetime
from sensor_msgs.msg import PointCloud

sys_byteorder = ('>', '<')[sys.byteorder == 'little']

ply_dtypes = dict([
    (b'int8', 'i1'),
    (b'char', 'i1'),
    (b'uint8', 'u1'),
    (b'uchar', 'b1'),
    (b'uchar', 'u1'),
    (b'int16', 'i2'),
    (b'short', 'i2'),
    (b'uint16', 'u2'),
    (b'ushort', 'u2'),
    (b'int32', 'i4'),
    (b'int', 'i4'),
    (b'uint32', 'u4'),
    (b'uint', 'u4'),
    (b'float32', 'f4'),
    (b'float', 'f4'),
    (b'float64', 'f8'),
    (b'double', 'f8')
])

valid_formats = {'ascii': '', 'binary_big_endian': '>',
                 'binary_little_endian': '<'}


def read_ply(filename):
    """ Read a .ply (binary or ascii) file and store the elements in pandas DataFrame
    Parameters
    ----------
    filename: str
        Path to the filename
    Returns
    -------
    data: dict
        Elements as pandas DataFrames; comments and ob_info as list of string
    """

    with open(filename, 'rb') as ply:

        if b'ply' not in ply.readline():
            raise ValueError('The file does not start whith the word ply')
        # get binary_little/big or ascii
        fmt = ply.readline().split()[1].decode()
        # get extension for building the numpy dtypes
        ext = valid_formats[fmt]

        line = []
        dtypes = defaultdict(list)
        count = 2
        points_size = None
        mesh_size = None
        has_texture = False
        while b'end_header' not in line and line != b'':
            line = ply.readline()

            if b'element' in line:
                line = line.split()
                name = line[1].decode()
                size = int(line[2])
                if name == "vertex":
                    points_size = size
                elif name == "face":
                    mesh_size = size

            elif b'property' in line:
                line = line.split()
                # element mesh
                if b'list' in line:

                    if b"vertex_indices" in line[-1]:
                        mesh_names = ["n_points", "v1", "v2", "v3"]
                    else:
                        has_texture = True
                        mesh_names = ["n_coords"] + ["v1_u", "v1_v", "v2_u", "v2_v", "v3_u", "v3_v"]

                    if fmt == "ascii":
                        # the first number has different dtype than the list
                        dtypes[name].append(
                            (mesh_names[0], ply_dtypes[line[2]]))
                        # rest of the numbers have the same dtype
                        dt = ply_dtypes[line[3]]
                    else:
                        # the first number has different dtype than the list
                        dtypes[name].append(
                            (mesh_names[0], ext + ply_dtypes[line[2]]))
                        # rest of the numbers have the same dtype
                        dt = ext + ply_dtypes[line[3]]

                    for j in range(1, len(mesh_names)):
                        dtypes[name].append((mesh_names[j], dt))
                else:
                    if fmt == "ascii":
                        dtypes[name].append(
                            (line[2].decode(), ply_dtypes[line[1]]))
                    else:
                        dtypes[name].append(
                            (line[2].decode(), ext + ply_dtypes[line[1]]))
            count += 1

        # for bin
        end_header = ply.tell()

    data = {}

    if fmt == 'ascii':
        top = count
        bottom = 0 if mesh_size is None else mesh_size

        names = [x[0] for x in dtypes["vertex"]]

        data["points"] = pd.read_csv(filename, sep=" ", header=None, engine="python",
                                     skiprows=top, skipfooter=bottom, usecols=names, names=names)

        for n, col in enumerate(data["points"].columns):
            data["points"][col] = data["points"][col].astype(
                dtypes["vertex"][n][1])

        if mesh_size :
            top = count + points_size

            names = np.array([x[0] for x in dtypes["face"]])
            usecols = [1, 2, 3, 5, 6, 7, 8, 9, 10] if has_texture else [1, 2, 3]
            names = names[usecols]

            data["mesh"] = pd.read_csv(
                filename, sep=" ", header=None, engine="python", skiprows=top, usecols=usecols, names=names)

            for n, col in enumerate(data["mesh"].columns):
                data["mesh"][col] = data["mesh"][col].astype(
                    dtypes["face"][n + 1][1])

    else:
        with open(filename, 'rb') as ply:
            ply.seek(end_header)
            points_np = np.fromfile(ply, dtype=dtypes["vertex"], count=points_size)
            if ext != sys_byteorder:
                points_np = points_np.byteswap().newbyteorder()
            data["points"] = pd.DataFrame(points_np)
            if mesh_size:
                mesh_np = np.fromfile(ply, dtype=dtypes["face"], count=mesh_size)
                if ext != sys_byteorder:
                    mesh_np = mesh_np.byteswap().newbyteorder()
                data["mesh"] = pd.DataFrame(mesh_np)
                data["mesh"].drop('n_points', axis=1, inplace=True)

    return data


class SavePointCloud():
    def __init__(self):
        # Creating this object once. Otherwise waitForTransform takes too long
        self.tflistener = tf.TransformListener()
        rospy.Subscriber('/tritech_profiler/scan', PointCloud, self.pcCallback, queue_size=1)
        self.total_cloud = PointCloud()
        self.total_cloud.header.frame_id = 'world'

        now = datetime.datetime.now()
        self.plystr = str(now) + '.ply'

    def pcCallback(self, data):
        success = self.tflistener.waitForTransform('/world','/sonar',
                                                    rospy.Time().now(),
                                                    rospy.Duration(5.0))

        transformed_data = self.tflistener.transformPointCloud('/world',data)

        print transformed_data.points[0]

        self.total_cloud.points +=transformed_data.points
        print self.total_cloud

        rospcd = self.PCtoPLY(self.total_cloud)

        file = open(self.plystr,'w')
        file.write(rospcd)
        file.close()

    def PCtoPLY(self, msg):

        datastr = 'ply' + '\n'
        datastr += 'format'   + ' ' + 'ascii 1.0'     + '\n'
        datastr += 'element vertex'+ ' ' + str(len(msg.points))  + '\n'
        datastr += 'property float32' + ' x' + '\n'
        datastr += 'property float32' + ' y'+ '\n'
        datastr += 'property float32'+ ' z' + '\n'
        # datastr += 'property list' + '\n'
        datastr += 'end_header'+'\n'

        pointstr = ''
        for p in msg.points:
            pointstr += str(p.x) + ' ' + str(p.y) + ' ' + str(p.z) + '\n'
        pointstr = pointstr.decode('utf-8').encode('ascii','replace')
        datastr += pointstr

        return datastr

    def PCtoPCD(self, msg):

        datastr = ''
        datastr += 'VERSION'   + ' ' + '.7'                                + '\n'
        datastr += 'FIELDS'    + ' ' + 'x y z'                             + '\n'
        datastr += 'SIZE'      + ' ' + '8 8 8'                             + '\n'
        datastr += 'TYPE'      + ' ' + 'F F F'                             + '\n'
        datastr += 'COUNT'     + ' ' + '1 1 1'                             + '\n'
        datastr += 'WIDTH'     + ' ' + '3'                                 + '\n'
        datastr += 'HEIGHT'    + ' ' + str(len(msg.points))                + '\n'
        datastr += 'VIEWPOINT' + ' ' + '0 0 0 1 0 0 0' + '\n'
        datastr += 'POINTS'    + ' ' + str(len(msg.points))                + '\n'
        datastr += 'DATA'      + ' ' + 'binary_compressed'                 + '\n'

        pointstr = ''
        for p in msg.points:
            pointstr += str(p.x) + ' ' + str(p.y) + ' ' + str(p.z) + '\n'
        pointstr = pointstr.decode('utf-8').encode('ascii','replace')
        datastr += pointstr

        return datastr


def main():
    rospy.init_node('pctoply', anonymous=True)
    # Create application object
    #total_cloud = np.empty()
    SavePointCloud()

    rospy.spin()

if __name__ == '__main__':
    main()
