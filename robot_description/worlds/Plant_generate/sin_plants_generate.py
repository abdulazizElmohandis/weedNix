import random
import math

def generate_new_coordinates(prev_x, base_y, randomize=True, use_sin=True, amplitude=1.0, frequency=1.0):
    """
    Generate new coordinates for a plant in a row.
    
    Parameters:
      - prev_x: previous X coordinate.
      - base_y: the base Y value for the row.
      - randomize: if False, use uniform placement (delta_x = 0.6, no offsets);
                   if True, generate random delta_x and add small offsets to X and Y.
      - use_sin: if True, calculate Y using a sine wave.
      - amplitude: amplitude of the sine wave.
      - frequency: frequency of the sine wave.
      
    Returns:
      - new_x, new_y: the new coordinates.
    """
    if randomize:
        # Increase X by a random delta between 0.21 and 0.74
        delta_x = round(random.uniform(0.31, 0.9), 3)
        new_x = round(prev_x + delta_x, 3)
        # Add a small random adjustment to new_x (to slightly modify the position)
        x_offset = round(random.uniform(-0.05, 0.05), 3)
        new_x = round(new_x + x_offset, 3)
        # Generate a small y offset
        y_offset = round(random.uniform(0.0053, 0.2530) * random.choice([-1, 1]), 3)
        if use_sin:
            new_y = round(base_y + amplitude * math.sin(frequency * new_x) + y_offset, 3)
        else:
            new_y = round(base_y + y_offset, 3)
    else:
        # Uniform placement: fixed delta_x and no offsets.
        delta_x = 0.7
        new_x = round(prev_x + delta_x, 3)
        if use_sin:
            new_y = round(base_y + amplitude * math.sin(frequency * new_x), 3)
        else:
            new_y = round(base_y, 3)
    return new_x, new_y

# Template for generating the plant model
template = """
    <model name='{}'>
      <link name='big_plant_22::link_0'>
        <pose frame=''>0 0 0.111949 0 -0 0</pose>
        <inertial>
          <mass>1e-08</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
          <pose frame=''>0 0 0 0 -0 0</pose>
        </inertial>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
        <visual name='visual'>
          <pose frame=''>0 0 0 0 -0 0</pose>
          <geometry>
            <mesh>
              <uri>model://big_plant/mesh/big_plant.stl</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <material>
            <lighting>1</lighting>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.01 1 0.01 1</specular>
            <emissive>0 1 0 1</emissive>
            <shader type='vertex'>
              <normal_map>__default__</normal_map>
            </shader>
          </material>
          <cast_shadows>1</cast_shadows>
          <transparency>0</transparency>
        </visual>
        <gravity>1</gravity>
      </link>
      <static>1</static>
      <allow_auto_disable>1</allow_auto_disable>
      <pose frame=''>{}</pose>
    </model>"""

######################################
# Section 1: Uniformly spaced plants with sine wave shape
######################################
num_rows_reg = 3        # Number of rows
row_length_reg = 14     # Length of each row
row_spacing_reg = 2     # Vertical spacing between rows
original_x_reg = -16    # Starting X position for the first row
original_y_reg = 4     # Starting Y position for the first row
original_name_reg = "Stander_Sin"

# Sine wave parameters for uniform section
amplitude_uniform = 0.4
frequency_uniform = 0.7

regular_lines = []

for row in range(num_rows_reg):
    base_y = original_y_reg + row * row_spacing_reg
    current_x = original_x_reg
    plant_num = 1
    while current_x - original_x_reg < row_length_reg:
        name = f"{original_name_reg}_R{row+1}_P{plant_num}"
        current_x, new_y = generate_new_coordinates(current_x, base_y, randomize=False, use_sin=True,
                                                     amplitude=amplitude_uniform, frequency=frequency_uniform)
        pose = f"{current_x} {new_y} 0 0 -0 0"
        regular_lines.append(template.format(name, pose).strip())
        plant_num += 1

######################################
# Section 2: Randomly spaced plants with sine wave shape and slight coordinate adjustments
######################################
num_rows_rand = 3        # Number of rows
row_length_rand = 14     # Length of each row
row_spacing_rand = 2     # Vertical spacing between rows
original_x_rand = 3      # Starting X position for the first row
original_y_rand = 4    # Starting Y position for the first row
original_name_rand = "Random_sin"

# Sine wave parameters for random section
amplitude_random = 0.4
frequency_random = 0.7

random_lines = []

for row in range(num_rows_rand):
    base_y = original_y_rand + row * row_spacing_rand
    current_x = original_x_rand
    plant_num = 1
    while current_x - original_x_rand < row_length_rand:
        name = f"{original_name_rand}_R{row+1}_P{plant_num}"
        current_x, new_y = generate_new_coordinates(current_x, base_y, randomize=True, use_sin=True,
                                                     amplitude=amplitude_random, frequency=frequency_random)
        pose = f"{current_x} {new_y} 0 0 -0 0"
        random_lines.append(template.format(name, pose).strip())
        plant_num += 1

######################################
# Merge results and save to file with professional comments
######################################
output_text = "<!-- add regular Sin -->\n"
output_text += "".join(regular_lines)
output_text += "\n<!-- end regular Sin -->\n\n"
output_text += "<!-- add random Sin -->\n"
output_text += "".join(random_lines)
output_text += "\n<!-- end random Sin -->\n"

with open("sin_wave_plants.sdf", "w", encoding="utf-8") as file:
    file.write(output_text)

print("File has been saved as sin_wave_plants.sdf")
