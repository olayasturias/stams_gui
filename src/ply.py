#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import rospy
import sys
import pypcd
from pypcd import numpy_pc2
import numpy as np
import pandas as pd
from collections import OrderedDict


from sensor_msgs.msg import PointCloud2
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


def write_ply(filename, points=None, mesh=None, as_text=False):
    """
    Parameters
    ----------
    filename: str
        The created file will be named with this
    points: ndarray
    mesh: ndarray
    as_text: boolean
        Set the write mode of the file. Default: binary
    Returns
    -------
    boolean
        True if no problems
    """
    if not filename.endswith('ply'):
        filename += '.ply'

    # open in text mode to write the header
    with open(filename, 'w') as ply:
        header = ['ply']

        if as_text:
            header.append('format ascii 1.0')
        else:
            header.append('format binary_' + sys.byteorder + '_endian 1.0')

        if points is not None:
            header.extend(describe_element('vertex', points))
        if mesh is not None:
            mesh = mesh.copy()
            mesh.insert(loc=0, column="n_points", value=3)
            mesh["n_points"] = mesh["n_points"].astype("u1")
            header.extend(describe_element('face', mesh))

        header.append('end_header')

        for line in header:
            ply.write("%s\n" % line)

    if as_text:
        if points is not None:
            points.to_csv(filename, sep=" ", index=False, header=False, mode='a',
                          encoding='ascii')
        if mesh is not None:
            mesh.to_csv(filename, sep=" ", index=False, header=False, mode='a',
                        encoding='ascii')

    else:
        with open(filename, 'ab') as ply:
            if points is not None:
                points.to_records(index=False).tofile(ply)
            if mesh is not None:
                mesh.to_records(index=False).tofile(ply)

    return True


def describe_element(name, df):
    """ Takes the columns of the dataframe and builds a ply-like description
    Parameters
    ----------
    name: str
    df: pandas DataFrame
    Returns
    -------
    element: list[str]
    """
    property_formats = {'f': 'float', 'u': 'uchar', 'i': 'int'}
    element = ['element ' + name + ' ' + str(len(df))]

    if name == 'face':
        element.append("property list uchar int vertex_indices")

    else:
        for i in range(len(df.columns)):
            # get first letter of dtype to infer format
            f = property_formats[str(df.dtypes[i])[0]]
            element.append('property ' + f + ' ' + df.columns.values[i])

    return element
def PCtoPC2(msg):
    rospc2 = PointCloud2
    rospc2.header = msg.header
    rospc2.width = 3
    rospc2.point_step = 8

    count = 0
    cloud = []
    for p in msg.points:
        count += 1
        cloud.append(p.x)
        cloud.append(p.y)
        cloud.append(p.z)
    rospc2.data = cloud
    rospc2.height = count
    return rospc2
def PCtoPCD(msg):

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

    for p in msg.points:
        datastr += str(p.x) + ' ' + str(p.y) + ' ' + str(p.z) + '\n'

    return datastr


def PCtoPLY(msg):

    datastr = 'ply' + '\n'
    datastr += 'format'   + ' ' + 'ascii 1.0'     + '\n'
    datastr += 'element vertex'+ ' ' + str(len(msg.points))  + '\n'
    datastr += 'property float32' + ' x' + '\n'
    datastr += 'property float32' + ' y'+ '\n'
    datastr += 'property float32'+ ' z' + '\n'
    # datastr += 'property list' + '\n'
    datastr += 'end_header'

    for p in msg.points:
        datastr += '\n'
        datastr += str(p.x) + ' ' + str(p.y) + ' ' + str(p.z)
    return datastr

def from_msg(msg, squeeze=True):
    """ from pointcloud2 msg
    squeeze: fix when clouds get 1 as first dim
    """
    md = {'version': .7,
          'fields': [],
          'size': [],
          'count': [],
          'width': 0,
          'height': 1,
          'viewpoint': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
          'points': 0,
          'type': [],
          'data': 'binary_compressed'}
    for field in msg.fields:
        md['fields'].append(field.name)
        t, s = pc2_type_to_pcd_type[field.datatype]
        md['type'].append(t)
        md['size'].append(s)
        # TODO handle multicount correctly
        if field.count > 1:
            warnings.warn('fields with count > 1 are not well tested')
        md['count'].append(field.count)
    pc_data = np.squeeze(numpy_pc2.pointcloud2_to_array(msg))
    md['width'] = len(pc_data)
    md['points'] = len(pc_data)
    pc = PointCloud(md, pc_data)
    return pc

def pcCallback(data):

    rospcd = PCtoPLY(data)
    print rospcd
    file = open('output.ply','w')
    file.write(rospcd)
    file.close()

    #"read pointcloud, append to array"
    cloud = []
    for p in data.points:
        cloud.append([p.x, p.y, p.z])
    p = pcl.PointCloud()
    p.from_array(cloud)

    # pc = PointCloud.from_msg(data)
    # pc.save('foo.pcd', compression='binary_compressed')
    # print 'saved'
    rospcd = PCtoPCD(data)
    file = open('output.pcd','w')
    file.write(rospcd)
    file.close()
    #pc.save('foo.pcd', compression='binary_compressed')
    print 'saved'


def main():
    rospy.init_node('pctoply', anonymous=True)
    # Create application object
    #total_cloud = np.empty()
    rospy.Subscriber('/tritech_profiler/scan', PointCloud, pcCallback)

    rospy.spin()

if __name__ == '__main__':
    main()
