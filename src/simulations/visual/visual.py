from src.vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from src.vehicle_model.suspension_model.suspension_elements._1_elements.node import Node
from src.vehicle_model.suspension_model.suspension_data import SuspensionData
from src.vehicle_model.suspension_model.suspension import Suspension

import pyvista as pv
import numpy as np
import yaml


class VisualModel:
    def __init__(self, model_path: str):
        self.model_path = model_path
        self.plotter = pv.Plotter()
        
        # Process inputs from config file
        with open("./src/simulations/visual/visual_inputs/visual.yml") as f:
            try:
                self.config : dict[str, dict[str, dict]] = yaml.safe_load(f)
            except yaml.YAMLError as error:
                print("Failed to import yaml file. Reason:\n")
                print(error)

        slider_spacing = self.config["Sliders"]["Spacing"]
        slider_vert_pos = self.config["Sliders"]["Vertical Position"]
        self.show_axes = self.config["Axes"]["Show"]
        self.show_bars = self.config["Stabars"]["Show"]
        self.show_ground = self.config["Ground"]["Show"]
        self.show_links = self.config["Links"]["Show"]
        self.show_nodes = self.config["Nodes"]["Show"]
        self.show_tires = self.config["Tires"]["Show"]
        self.show_vehicle_CG = self.config["Vehicle CG"]["Show"]
        self.CG_node_radius = self.config["Vehicle CG"]["Radius"]
        self.show_N_lines = self.config["N Lines"]["Show"]
        self.N_line_radius = self.config["N Lines"]["Radius"]
        self.node_radius = self.config["Nodes"]["Radius"]
        self.link_radius = self.config["Links"]["Radius"]

        if self.show_axes:
            self.plotter.add_axes()

        # Define and store suspension
        sus_data = SuspensionData(path=self.model_path)
        self.sus = Suspension(sus_data=sus_data)

        self.heave_val = 0.0
        self.pitch_val = 0.0
        self.roll_val = 0.0

        # Ground parameters
        FL_cp = self.sus.FL_quarter_car.tire.contact_patch
        FR_cp = self.sus.FR_quarter_car.tire.contact_patch
        RL_cp = self.sus.RL_quarter_car.tire.contact_patch
        RR_cp = self.sus.RR_quarter_car.tire.contact_patch
        
        self.center = ((FL_cp + FR_cp + RL_cp + RR_cp) / 4).position
        self.direction = [0, 0, 1]
        self.i_size = abs((FL_cp - RL_cp).position[0]) * 1.25
        self.j_size = abs((FL_cp - FR_cp).position[1]) * 1.25

        # Slider spacing and sizing
        sp = slider_spacing
        ss = (1 - sp * 4) / 3

        # Add sliders
        self.plotter.add_slider_widget(self.on_heave_change, 
                                       [-5, 5], 
                                       title='Heave (in)', 
                                       value=0.0, 
                                       pointa=(sp, slider_vert_pos), 
                                       pointb=(sp + ss, slider_vert_pos))
        self.plotter.add_slider_widget(self.on_pitch_change, 
                                       [-10, 10], 
                                       title='Pitch (deg)', 
                                       value=0.0, 
                                       pointa=(2 * sp + ss, .925), 
                                       pointb=(2 * sp + 2 * ss, slider_vert_pos))
        self.plotter.add_slider_widget(self.on_roll_change, 
                                       [-10, 10], 
                                       title='Roll (deg)', 
                                       value=0.0, 
                                       pointa=(3 * sp + 2 * ss, .925), 
                                       pointb=(3 * sp + 3 * ss, slider_vert_pos))
        self.update_plot()
        self.plotter.show()

    def on_heave_change(self, value):
        self.heave_val = value
        self.update_plot()

    def on_pitch_change(self, value):
        self.pitch_val = value
        self.update_plot()

    def on_roll_change(self, value):
        self.roll_val = value
        self.update_plot()

    def update_plot(self):
        self.plotter.clear_actors()

        # Reset and apply suspension settings
        sus_data = SuspensionData(path=self.model_path)
        self.sus = Suspension(sus_data=sus_data)
        
        self.sus.heave(heave=self.heave_val * 0.0254, update_state=False)
        self.sus.pitch(pitch=self.pitch_val, update_state=False)
        self.sus.roll(roll=self.roll_val, update_state=False)
        
        # Transform literally everything :))
        
        if self.show_nodes:
            for key, node in self.sus.FL_nodes.items():
                self.sus.FL_nodes[key] = self.sus._sprung_to_global(node=node)
            for key, node in self.sus.FR_nodes.items():
                self.sus.FR_nodes[key] = self.sus._sprung_to_global(node=node)
            for key, node in self.sus.RL_nodes.items():
                self.sus.RL_nodes[key] = self.sus._sprung_to_global(node=node)
            for key, node in self.sus.RR_nodes.items():
                self.sus.RR_nodes[key] = self.sus._sprung_to_global(node=node)
            for i, node in enumerate(self.sus.tire_nodes):
                self.sus.tire_nodes[i] = self.sus._sprung_to_global(node=node)

        if self.show_vehicle_CG:
            self.sus.CG_node = self.sus._sprung_to_global(node=self.sus.CG_node)
        
        if self.show_links:
            for key, link in self.sus.FL_links.items():
                trans_inboard = self.sus._sprung_to_global(node=link.inboard_node)
                trans_outboard = self.sus._sprung_to_global(node=link.outboard_node)
                self.sus.FL_links[key] = Link(inboard_node=trans_inboard, outboard_node=trans_outboard)
            for key, link in self.sus.FR_links.items():
                trans_inboard = self.sus._sprung_to_global(node=link.inboard_node)
                trans_outboard = self.sus._sprung_to_global(node=link.outboard_node)
                self.sus.FR_links[key] = Link(inboard_node=trans_inboard, outboard_node=trans_outboard)
            for key, link in self.sus.RL_links.items():
                trans_inboard = self.sus._sprung_to_global(node=link.inboard_node)
                trans_outboard = self.sus._sprung_to_global(node=link.outboard_node)
                self.sus.RL_links[key] = Link(inboard_node=trans_inboard, outboard_node=trans_outboard)
            for key, link in self.sus.RR_links.items():
                trans_inboard = self.sus._sprung_to_global(node=link.inboard_node)
                trans_outboard = self.sus._sprung_to_global(node=link.outboard_node)
                self.sus.RR_links[key] = Link(inboard_node=trans_inboard, outboard_node=trans_outboard)
        
        if self.show_bars:
            for key, link in self.sus.Fr_stabar_links.items():
                trans_inboard = self.sus._sprung_to_global(node=link.inboard_node)
                trans_outboard = self.sus._sprung_to_global(node=link.outboard_node)
                self.sus.Fr_stabar_links[key] = Link(inboard_node=trans_inboard, outboard_node=trans_outboard)
            for key, link in self.sus.Rr_stabar_links.items():
                trans_inboard = self.sus._sprung_to_global(node=link.inboard_node)
                trans_outboard = self.sus._sprung_to_global(node=link.outboard_node)
                self.sus.Rr_stabar_links[key] = Link(inboard_node=trans_inboard, outboard_node=trans_outboard)

        # Now plot that shit

        if self.show_nodes:
            for _, node in self.sus.FL_nodes.items():
                sphere = pv.Sphere(radius=self.node_radius, center=node.position)
                self.plotter.add_mesh(sphere, color='red')
            for _, node in self.sus.FR_nodes.items():
                sphere = pv.Sphere(radius=self.node_radius, center=node.position)
                self.plotter.add_mesh(sphere, color='red')
            for _, node in self.sus.RL_nodes.items():
                sphere = pv.Sphere(radius=self.node_radius, center=node.position)
                self.plotter.add_mesh(sphere, color='red')
            for _, node in self.sus.RR_nodes.items():
                sphere = pv.Sphere(radius=self.node_radius, center=node.position)
                self.plotter.add_mesh(sphere, color='red')
            for node in self.sus.tire_nodes:
                sphere = pv.Sphere(radius=self.node_radius, center=node.position)
                self.plotter.add_mesh(sphere, color='red')
                
        if self.show_links:
            for _, link in self.sus.FL_links.items():
                cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=self.link_radius, height=link.length)
                self.plotter.add_mesh(cylinder, color='gray')
            for _, link in self.sus.FR_links.items():
                cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=self.link_radius, height=link.length)
                self.plotter.add_mesh(cylinder, color='gray')
            for _, link in self.sus.RL_links.items():
                cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=self.link_radius, height=link.length)
                self.plotter.add_mesh(cylinder, color='gray')
            for _, link in self.sus.RR_links.items():
                cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=self.link_radius, height=link.length)
                self.plotter.add_mesh(cylinder, color='gray')

        if self.show_tires:
            for tire in self.sus.tires:
                tube = pv.CylinderStructured(radius=[tire.outer_diameter / 2, tire.inner_diameter / 2],
                                            height=tire.width,
                                            center=self.sus._sprung_to_global(node=Node(position=tire.center)).position,
                                            direction=self.sus._sprung_to_global(Node(position=tire.direction), align_axes=False).position)
                self.plotter.add_mesh(tube, color='black', opacity=0.5)
        
        if self.show_vehicle_CG:
            sphere = pv.Sphere(radius=self.CG_node_radius, center=self.sus.CG_node.position)
            self.plotter.add_mesh(sphere, color='black')
        
        if self.show_N_lines:
            for link in self.sus.FL_N_lines:
                cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=self.link_radius, height=link.length)
                self.plotter.add_mesh(cylinder, color='blue')

            for link in self.sus.FR_N_lines:
                cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=self.link_radius, height=link.length)
                self.plotter.add_mesh(cylinder, color='blue')
            
            for link in self.sus.RL_N_lines:
                cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=self.link_radius, height=link.length)
                self.plotter.add_mesh(cylinder, color='blue')

            for link in self.sus.RR_N_lines:
                cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=self.link_radius, height=link.length)
                self.plotter.add_mesh(cylinder, color='blue')

        if self.show_bars:
            for _, link in self.sus.Fr_stabar_links.items():
                cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=self.link_radius, height=link.length)
                self.plotter.add_mesh(cylinder, color='green')
            for _, link in self.sus.Rr_stabar_links.items():
                cylinder = pv.Cylinder(center=link.center, direction=link.direction, radius=self.link_radius, height=link.length)
                self.plotter.add_mesh(cylinder, color='green')

        if self.show_ground:
            ground = pv.Plane(center=self.center,
                            direction=self.direction,
                            i_size=self.i_size,
                            j_size=self.j_size)
            self.plotter.add_mesh(ground, opacity=0.8)

        self.plotter.render()