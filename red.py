import random
import time
from collections import deque, defaultdict
import heapq
import PySimpleGUI as sg

class Node:
    def __init__(self, value):
        self.value = value
        self.next = None

class DroneSimulator:
    def __init__(self, grid_size, start_index, target_index, obstacles):
        self.grid_size = grid_size
        self.drone_position = start_index
        self.target_position = target_index
        self.obstacles = list(obstacles)
        self.path = self.calculate_shortest_path()
        self.speed = 0
        self.total_time = 0
        self.altitude_variations = [start_index[2]]
        self.timestamp_queue = deque()
        self.stop_queue = []
        self.spatial_index = self.build_spatial_index()
        self.camera_images = {}
        self.distance_traveled = 0
        self.battery_level = 100
        self.path_graph = self.build_path_graph()
        self.travel_path = self.build_travel_path()

    def build_spatial_index(self):
        spatial_index = {}
        for obstacle in self.obstacles:
            x, y, z = obstacle
            if x not in spatial_index:
                spatial_index[x] = {}
            if y not in spatial_index[x]:
                spatial_index[x][y] = set()
            spatial_index[x][y].add(z)
        return spatial_index

    def calculate_shortest_path(self):
        distances = {self.drone_position: 0}
        queue = [(0, self.drone_position)]
        previous = {}
        while queue:
            current_distance, current_position = heapq.heappop(queue)
            if current_position == self.target_position:
                path = []
                while current_position in previous:
                    path.append(current_position)
                    current_position = previous[current_position]
                path.append(self.drone_position)
                return path[::-1]
            if current_position in distances and current_distance > distances[current_position]:
                continue
            distances[current_position] = current_distance
            neighbors = self.get_neighbors(current_position)
            for neighbor in neighbors:
                if neighbor not in self.obstacles:
                    new_distance = current_distance + self.get_distance(current_position, neighbor)
                    if neighbor not in distances or new_distance < distances[neighbor]:
                        heapq.heappush(queue, (new_distance, neighbor))
                        previous[neighbor] = current_position
        return []

    def get_neighbors(self, position):
        x, y, z = position
        neighbors = []
        if x > 0:
            neighbors.append((x - 1, y, z))
        if x < self.grid_size - 1:
            neighbors.append((x + 1, y, z))
        if y > 0:
            neighbors.append((x, y - 1, z))
        if y < self.grid_size - 1:
            neighbors.append((x, y + 1, z))
        if z > 0:
            neighbors.append((x, y, z - 1))
        if z < self.grid_size - 1:
            neighbors.append((x, y, z + 1))
        return neighbors

    def get_distance(self, position1, position2):
        x1, y1, z1 = position1
        x2, y2, z2 = position2
        return abs(x2 - x1) + abs(y2 - y1) + abs(z2 - z1)

    def move_to_next_position(self):
        if self.path:
            next_position = self.path.pop(0)
            distance = self.get_distance(self.drone_position, next_position)
            self.distance_traveled += distance
            self.drone_position = next_position
            self.battery_level -= 0.2
            self.altitude_variations.append(next_position[2])
            if self.total_time > 0:
                self.speed = self.distance_traveled / self.total_time
            if next_position in self.obstacles:
                self.capture_image(next_position)
            self.add_to_travel_path(next_position)

    def simulate(self):
        while self.drone_position != self.target_position and self.battery_level > 0:
            self.move_to_next_position()
            self.total_time += 1
            time.sleep(0.5)

    def capture_image(self, position):
        if position in self.obstacles:
            image = f"Captured image at obstacle position {position}"
            self.camera_images[position] = image



    def display_simulation(self):
        # Create layout for the GUI
        layout = [
            [sg.Text(f"Distance Traveled: {self.distance_traveled} units")],
            [sg.Text(f"Battery Level: {self.battery_level}%")],
            [sg.Text(f"Time taken: {self.total_time:.2f} seconds")],
            [sg.Text(f"Drone speed: {self.speed:.2f} units/second")],
            [sg.Text(f"Altitude Variations:")],
            [sg.Multiline("\n".join(f"Index {index}: {altitude} units" for index, altitude in enumerate(self.altitude_variations)), size=(30, 10), key='altitude_variations', disabled=True)],
            [sg.Text(f"Path Graph:")],
            [sg.Multiline(self.get_formatted_path_graph(), size=(30, 10), key='path_graph', disabled=True)],
            [sg.Text(f"Path Followed:")],
            [sg.Multiline(self.get_formatted_path_followed(), size=(30, 10), key='path_followed', disabled=True)],
        ]

        # Create the window
        window = sg.Window('Drone Simulator', layout)

        # Event loop
        while True:
            event, values = window.read()

            if event == sg.WINDOW_CLOSED:
                break

        # Close the window
        window.close()

    def get_formatted_path_graph(self):
        if self.path_graph is None:
            return "No path graph found."
        formatted_path_graph = []
        current = self.path_graph
        while current:
            formatted_path_graph.append(f"Node {current.value}: {self.get_neighbors(current.value)}")
            current = current.next
        return "\n".join(formatted_path_graph)

    def get_formatted_path_followed(self):
        path = []
        current = self.travel_path
        while current:
            path.append(str(current.value))
            current = current.next
        return "\n".join(path)

    def build_path_graph(self):
        head = None
        prev = None
        for obstacle in self.obstacles:
            node = Node(obstacle)
            if prev:
                prev.next = node
            else:
                head = node
            prev = node
        return head

    def build_travel_path(self):
        head = Node(self.drone_position)
        current = head
        for position in self.path[1:]:
            node = Node(position)
            current.next = node
            current = node
        return head

    def add_to_travel_path(self, position):
        new_node = Node(position)
        current = self.travel_path
        while current.next:
            current = current.next
        current.next = new_node

    def print_altitude_variations(self):
        print("Altitude Variations:")
        for index, altitude in enumerate(self.altitude_variations):
            print(f"Index {index + 1}: {altitude} units")

    def print_camera_images(self):
        print("Obstacle Coordinates:")
        for obstacle in obstacles:
            print(obstacle)

    def print_path_graph(self):
        print("Path Graph:")
        if self.path_graph is None:
            print("No path graph found.")
        else:
            current = self.path_graph
            while current:
                print(f"Node {current.value}: {self.get_neighbors(current.value)}")
                current = current.next

    def print_path_followed(self):
        print("Path Followed:")
        path = []
        current = self.travel_path
        while current:
            path.append(str(current.value))
            current = current.next
        print("\n".join(path))

# Create layout for the GUI
layout = [
    [sg.Text('Grid Size:'), sg.Input(size=(10, 1), key='grid_size')],
    [sg.Text('Start Position (x, y, z):'), sg.Input(size=(20, 1), key='start_position')],
    [sg.Text('Target Position (x, y, z):'), sg.Input(size=(20, 1), key='target_position')],
    [sg.Text('Number of Obstacles:'), sg.Input(size=(10, 1), key='num_obstacles')],
    [sg.Button('Next')],
]

# Create the window
window = sg.Window('Drone Simulator', layout)

# Event loop
while True:
    event, values = window.read()

    if event == sg.WINDOW_CLOSED:
        break

    if event == 'Next':
        # Get user input values
        grid_size = int(values['grid_size'])
        start_index = tuple(map(int, values['start_position'].split(',')))
        target_index = tuple(map(int, values['target_position'].split(',')))
        num_obstacles = int(values['num_obstacles'])

        # Close the initial window
        window.close()

        # Create layout for obstacle inputs
        obstacle_layout = [
            [sg.Text(f'Obstacle {i + 1} (x, y, z):'), sg.Input(size=(20, 1), key=f'obstacle_{i}')] for i in range(num_obstacles)
        ]
        obstacle_layout.append([sg.Button('Submit')])

        obstacle_window = sg.Window('Input Obstacles', obstacle_layout)

        while True:
            event, values = obstacle_window.read()

            if event == sg.WINDOW_CLOSED:
                break

            if event == 'Submit':
                obstacles = [tuple(map(int, values[f'obstacle_{i}'].split(','))) for i in range(num_obstacles)]
                obstacle_window.close()
                break

        simulator = DroneSimulator(grid_size, start_index, target_index, obstacles)
        simulator.simulate()
        simulator.display_simulation()



# Close the window
window.close()