from pandas import DataFrame, read_csv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

class V4LOG_to_py:
    """ *V4LOG_to_py* reads a CSV file and converts it to a readable format by python

    ** Attributes **:

      .. data:: filename
      Name of the CSV file that is being read

      .. data:: numcols
      Number of columns that the file has in total

      .. data:: sensor_raw
      Dataframe obtained from reading the useful columns from the csv

      .. data:: cylindric_cartesian
      Dataframe with the cylindrical and cartessian coordinates obtained from *sensor_raw*

    """
    def __init__(self, filename):
      with open (filename, 'rb') as csvinput:
        reader = csv.reader(csvinput)

        for row in reader:
          try:
            self.numcols = len(reader.next())
          except:
            pass

    def generate_useful_data_header(self):
      """ Generate the header for reading only the useful input data that we need
      from the CSV. This should remain equal so only need to be called once.
      """

      # This variable is for the complete header of the file
      self.names = ['SPf','DateTime','Node','Rangescale','Gain','LeftLim','RightLim','steps','pings']

      # This variable is the header name of the columns that will be useful for the data acquisition algorithm
      self.useful_data = ['LeftLim','RightLim','steps','pings']
      pointheader = []
      self.datan = self.numcols-len(self.names)-5
      # Give all the unlabeled columns (which correspond to points) a name
      for k in range (self.datan):
          string = 'point_%d' % k
          self.names.append(string)
          self.useful_data.append(string)

      # The 5 last columns have this special labels
      self.names.append('|')
      self.names.append('Code')
      self.names.append('Startms')
      self.names.append('Endms')
      self.names.append('status')

    def int_gradian_to_float_rad(self,x):
      """ Conversion from degree format of tritech to radians
      """
      x_float = float(x)
      x_float_gradian = x_float*(0.9/16)*(np.pi/180)
      return x_float_gradian

    def read_useful_csv(self,filename):
      """ Reading only the useful columns from the CSV file
      """
      self.sensor_raw = pd.read_csv(filename, header = 0,names = self.names, usecols = self.useful_data,
                                    converters={'LeftLim': self.int_gradian_to_float_rad,
                                                'RightLim': self.int_gradian_to_float_rad,
                                                'steps': self.int_gradian_to_float_rad})

    def convert_cylindrical_cartesian(self):
      """ Take the Dataframe obtained from reading the csv and convert it to a table
      with cylindrical coordinates and cartesian coordinates

      .. data:: converted_point
      Here the distance in meters for each point is saved according to linear movement equation

      .. data:: angle
      Angle associated to each point

      .. data:: X
      cartesian axis X which correspond to each point

      .. data:: Y
      cartesian axis X which correspond to each point

      .. data:: scan
      Number of scan (each 360 we have a new scan). Used for asigning a Z coordinate
      in case of not having GPS data

      .. data:: npoints_total_table
      Number of points (necessary to save them then in a column table)

      """
      converted_point = []
      angle = []
      X = []
      Y = []
      scan = []
      alpha_ini = 0
      npoints_total_table = []
      npoints_total = 0
      angletest = 45

      for index, row in self.sensor_raw.iterrows():
        # this loop covers each row
        barrido = index
        npoints = self.datan
        #npoints = int(row['pings'])
        for k in range (npoints):
          # this loop covers each element in a row

          # choose point by name (e.g. point_29)
          point_str = 'point_%d' % k

          # extract point, convert it to distance using sound velocity in water
          point = float(row[point_str])*1450/2000
          converted_point.append(point)

          # angle associated to point (converted to radians)
          ang = abs(float(row['steps']))*(float(k)+ alpha_ini)
          angle.append(ang)

          # scan number, can be used as height value (z coordinate)
          scan.append(barrido)

          # cartesian coordinates
          xpoint = point*np.cos(ang)
          ypoint = point*np.sin(ang)
          X.append(xpoint)
          Y.append(ypoint)

          # table with indexes, necessary to create the Dataframe later
          npoints_total_table.append(npoints_total)
          npoints_total += 1
        alpha_ini = float(angle[npoints-1])

      # Create a column labeled table with the distance obtained by sensor
      cylindric_coord1 = pd.DataFrame(converted_point[:len(npoints_total_table)], index=[npoints_total_table], columns=['Distance'])
      # Create a column labeled table with the angles from where measurement was taken
      cylindric_coord2 = pd.DataFrame(angle[:len(npoints_total_table)], index=[npoints_total_table], columns=['Angle'])
      # Colum with x cartesian coordinates
      x_coord = pd.DataFrame(X[:len(npoints_total_table)], index=[npoints_total_table], columns=['X'])
      # Cloumn with Y cartesian coordinates
      y_coord = pd.DataFrame(Y[:len(npoints_total_table)], index=[npoints_total_table], columns=['Y'])
      # Create a column labeled table with the number of scan for each measurement
      cylindric_coord3 = pd.DataFrame(scan[:len(npoints_total_table)], index=[npoints_total_table], columns=['Z'])

      # Concatenate previous tables to obtain the table with the cylindrical coordinates.
      axis = [cylindric_coord1,cylindric_coord2, x_coord, y_coord, cylindric_coord3]
      self.cylindric_cartesian = pd.concat(axis, axis=1)

      return int(xpoint)




def main():
  converter = V4LOG_to_py('PS.CSV')
  converter.generate_useful_data_header()
  converter.read_useful_csv('PS.CSV')
  converter.convert_cylindrical_cartesian()

  threedee = plt.figure().gca(projection='3d')
  threedee.scatter(converter.cylindric_cartesian['X'],
                   converter.cylindric_cartesian['Y'],
                   converter.cylindric_cartesian['Z'])

  plt.show()




if __name__ == '__main__':
    main()
