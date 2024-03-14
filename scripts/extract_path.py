
import numpy as np
import csv
import re


class coordinates:
    def __init__(self, file: str):
        self.file = file

    def read_csv(self):
        read_file = open(self.file, 'r')

        csv_reader = csv.reader(read_file)

        edges = []

        for row in csv_reader:
            edges.append(row)

        read_file.close()

        pattern = r'\d+'

        xy = []
        for edge in edges:
            coordinate = re.findall(pattern, edge[0])
            coordinate = list(map(int, coordinate))
            xy.append(coordinate)

        xy = np.array(xy)
        xy[:,1] = 800 - xy[:,1]

        return xy
    
    def normalize_coordinates_to_window(self, coordinates: np.ndarray, window_width: int, window_height: int, offset: tuple = (0,0)):
        '''
        This attribute maps the coordinates of the edges to the window size
        with the offset from the origin
        '''
        width = coordinates[:,0].max() - coordinates[:,0].min()
        height = coordinates[:,1].max() - coordinates[:,1].min()

        points = coordinates.astype(float)

        points[:,0] = (points[:,0] - points[:,0].min()) / width * window_width + offset[0]
        points[:,1] = (points[:,1] - points[:,1].min()) / height * window_height + offset[1]
        
        return points
        



if __name__ == '__main__':

    coo = coordinates('edges.csv')

    points = coo.read_csv()

    print(points)
    import matplotlib.pyplot as plt

    # line plot
    plt.plot(points[:, 0], points[:, 1], 'o-')
    plt.show()

    # normalize coordinates
    points = coo.normalize_coordinates_to_window(points, window_width=100, window_height=100, offset=(5,5))
    # line plot
    plt.plot(points[:, 0], points[:, 1], 'o-')
    plt.show()
