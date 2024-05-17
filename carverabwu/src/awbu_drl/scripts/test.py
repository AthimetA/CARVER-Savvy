import sys
from PyQt5 import QtWidgets
import pyqtgraph as pg
import numpy as np

class RobotPlotter(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()
        
        # Initialize some example data for the robot
        self.robot_x = np.array([0])
        self.robot_y = np.array([0])

        # Initialize some example data for the obstacles
        self.obstacle_x = np.array([1, 2, 3])
        self.obstacle_y = np.array([1, 2, 3])

    def initUI(self):
        # Create a central widget and set a layout
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QtWidgets.QVBoxLayout(self.central_widget)

        # Create a QTabWidget
        self.tabs = QtWidgets.QTabWidget()
        self.layout.addWidget(self.tabs)

        # Create the plot tab
        self.plot_tab = QtWidgets.QWidget()
        self.tabs.addTab(self.plot_tab, "Plot")

        # Create a layout for the plot tab
        self.plot_layout = QtWidgets.QVBoxLayout(self.plot_tab)

        # Create a plot window
        self.win = pg.GraphicsLayoutWidget(show=True, title="Robot and Obstacle Position")
        self.plot_layout.addWidget(self.win)
        
        # Create a plot item
        self.plot = self.win.addPlot(title="Robot and Obstacle X-Y Position")
        self.plot.setLabel('left', 'Y Position')
        self.plot.setLabel('bottom', 'X Position')
        self.plot.showGrid(x=True, y=True)
        
        # Create a scatter plot item for the robot's position
        self.robot_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 0, 0, 120), name="Robot")
        self.plot.addItem(self.robot_scatter)
        
        # Create a scatter plot item for the obstacles' positions
        self.obstacle_scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(0, 255, 0, 120), name="Obstacles")
        self.plot.addItem(self.obstacle_scatter)

        # Optionally, add more tabs with different content
        self.additional_tab = QtWidgets.QWidget()
        self.tabs.addTab(self.additional_tab, "Additional Tab")
        self.additional_layout = QtWidgets.QVBoxLayout(self.additional_tab)
        self.additional_label = QtWidgets.QLabel("This is an additional tab.")
        self.additional_layout.addWidget(self.additional_label)
        
        # Set up a timer to update the plot periodically
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)  # Update every 50 ms

    def update(self):
        # Example update logic for the robot: incrementing position
        new_robot_x = self.robot_x[-1] + 0.1 * (np.random.random() - 0.5)
        new_robot_y = self.robot_y[-1] + 0.1 * (np.random.random() - 0.5)

        # Update the robot's position data
        self.robot_x = np.append(self.robot_x, new_robot_x)
        self.robot_y = np.append(self.robot_y, new_robot_y)

        # Update the robot scatter plot
        self.robot_scatter.setData(self.robot_x, self.robot_y)

        # Update the obstacles scatter plot
        self.obstacle_scatter.setData(self.obstacle_x, self.obstacle_y)

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = RobotPlotter()
    main.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
