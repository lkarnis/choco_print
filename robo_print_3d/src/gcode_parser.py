#!/usr/bin/env python3
import os
import math
import matplotlib.pyplot as plt

class GCodeParser:
    def __init__(self, path="."):
        self.path = path
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.e = False

    def parse_file(self, filename):
        gcode_path = os.path.join(self.path, filename)
        with open(gcode_path, "r") as gcode:
            with open("coordinates.txt", "w") as coordinates_file, open("extruder_commands.txt", "w") as extruder_commands_file:
                previous_e = None
                for line in gcode:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    parsed_line = self.parse_line(line)
                    if parsed_line:
                        x, y, z, e = parsed_line
                        if x is not None:
                            self.x = round(x, 6)
                        if y is not None:
                            self.y = round(y, 6)
                        if z is not None:
                            self.z = round(z, 6)
                        if x is not None or y is not None or z is not None:
                            coordinates_file.write(f"{self.x} {self.y} {self.z}\n")
                        if e is not None and e != previous_e:
                            extruder_commands_file.write(f"{self.x} {self.y} {self.z} {1 if e else 0}\n")
                            previous_e = e

    def parse_line(self, line):
        command = line.split(' ')
        if command[0] in ['G0', 'G1']:
            x = None
            y = None
            z = None
            e = None
            for com in command[1:]:
                if com[0] == 'X':
                    x = round(float(com[1:]) / 1000.0, 6)
                elif com[0] == 'Y':
                    y = round(float(com[1:]) / 1000.0, 6)
                elif com[0] == 'Z':
                    z = round(float(com[1:]) / 1000.0, 6)
                elif com[0] in ['F', 'E']:
                    e = True if float(com[1:]) > 0 else False
            return x, y, z, e
            
    def remove_close_points(self, threshold=0.0006):
        with open("coordinates.txt", "r") as coordinates_file, open("filtered_coordinates.txt", "w") as filtered_file:
            previous_point = None
            for line in coordinates_file:
                x, y, z = map(float, line.strip().split())
                if previous_point is not None:
                    dx, dy, dz = x - previous_point[0], y - previous_point[1], z - previous_point[2]
                    if math.sqrt(dx * dx + dy * dy + dz * dz) < threshold:
                        continue
                filtered_file.write(f"{x} {y} {z}\n")
                previous_point = (x, y, z)

    def draw_graphs(self):
        z_coordinates = {}
        with open("filtered_coordinates.txt", "r") as coordinates_file:
            for line in coordinates_file:
                x, y, z = map(float, line.strip().split())
                if z not in z_coordinates:
                    z_coordinates[z] = [[], []]
                z_coordinates[z][0].append(x)
                z_coordinates[z][1].append(y)
                

        for z in z_coordinates:
            if z == 0.0:
                continue
            plt.figure()
            plt.plot(z_coordinates[z][0], z_coordinates[z][1], '-b')  
            plt.scatter(z_coordinates[z][0], z_coordinates[z][1], color='r')  
            plt.title(f"Graf za z={z}")
            plt.xlabel('x')
            plt.ylabel('y')
            plt.grid(True)
            plt.show()
            
        z_coordinates = {}
        with open("filtered_coordinates.txt", "r") as coordinates_file:
            for line in coordinates_file:
                x, y, z = map(float, line.strip().split())
                if z not in z_coordinates:
                    z_coordinates[z] = [[], []]
                z_coordinates[z][0].append(x)
                z_coordinates[z][1].append(y)
                

        for z in z_coordinates:
            if z == 0.0:
                continue
            plt.figure()
            plt.plot(z_coordinates[z][0], z_coordinates[z][1], '-b')  
            plt.scatter(z_coordinates[z][0], z_coordinates[z][1], color='r')  
            plt.title(f"Graf za z={z}")
            plt.xlabel('x')
            plt.ylabel('y')
            plt.grid(True)
            plt.show()

if __name__ == "__main__":
    parser = GCodeParser()
    directory = os.path.dirname(os.path.abspath(__file__))

    while True:
        filename = input("Unesite ime GCode datoteke: ")
        gcode_path = os.path.join(directory, filename)
    
        if os.path.isfile(gcode_path):
            parser.parse_file(gcode_path)
            parser.remove_close_points() 
            parser.draw_graphs()
            print("Gcode je parsiran!")
            break
        else:    
            print("Neispravno ime datoteke. Upiši ponovno:")

