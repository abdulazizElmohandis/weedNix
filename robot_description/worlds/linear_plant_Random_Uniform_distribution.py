import random

def generate_new_coordinates(prev_x, base_y, randomize=True):
    """
    Generate new coordinates for a plant in a row.
    
    Parameters:
      prev_x (float): The previous x-coordinate.
      base_y (float): The fixed base y-coordinate for the row.
      randomize (bool): If True, applies random variation to both x and y.
                        If False, x increases by a fixed value and y remains unchanged.
    
    Returns:
      tuple: A tuple containing the new x-coordinate and the plant's y-coordinate.
             Note that base_y remains constant for the row.
    """
    if randomize:
        delta_x = round(random.uniform(0.45, 1.274), 3)
        new_x = round(prev_x + delta_x, 3)
        y_offset = round(random.uniform(0.0053, 0.25) * random.choice([-1, 1]), 3)
        plant_y = round(base_y + y_offset, 3)
    else:
        delta_x = 0.7
        new_x = round(prev_x + delta_x, 3)
        plant_y = round(base_y, 3)
    return new_x, plant_y

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
# Section 1: Uniformly Spaced Plants
######################################
num_rows_reg = 3        # Number of rows
row_length_reg = 14     # Length of each row
row_spacing_reg = 2     # Vertical spacing between rows
original_x_reg = -16    # Starting x-coordinate for the first row
original_y_reg = -5     # Starting y-coordinate for the first row
original_name_reg = "Stander"

regular_lines = []

for row in range(num_rows_reg):
    base_y = original_y_reg + row * row_spacing_reg  # Fixed base y-coordinate for each row
    current_x = original_x_reg
    plant_num = 1
    while current_x - original_x_reg < row_length_reg:
        name = f"{original_name_reg}_G2_R{row+1}_P{plant_num}"
        new_x, plant_y = generate_new_coordinates(current_x, base_y, randomize=False)
        pose = f"{new_x} {plant_y} 0 0 -0 0"
        regular_lines.append(template.format(name, pose).strip())
        current_x = new_x   # Update x-coordinate only; base_y remains unchanged for the row
        plant_num += 1

######################################
# Section 2: Randomly Spaced Plants
######################################
num_rows_rand = 3        # Number of rows
row_length_rand = 14     # Length of each row
row_spacing_rand = 2     # Vertical spacing between rows
original_x_rand = 3      # Starting x-coordinate for the first row
original_y_rand = -5     # Starting y-coordinate for the first row
original_name_rand = "Random"

random_lines = []

for row in range(num_rows_rand):
    base_y = original_y_rand + row * row_spacing_rand  # Fixed base y-coordinate for each row
    current_x = original_x_rand
    plant_num = 1
    while current_x - original_x_rand < row_length_rand:
        name = f"{original_name_rand}_G2_R{row+1}_P{plant_num}"
        new_x, plant_y = generate_new_coordinates(current_x, base_y, randomize=True)
        pose = f"{new_x} {plant_y} 0 0 -0 0"
        random_lines.append(template.format(name, pose).strip())
        current_x = new_x   # Update x-coordinate only; base_y remains unchanged for the row
        plant_num += 1

######################################
# Merge Results and Save to File
######################################
output_text = "<!-- add regular lines -->\n"
output_text += "".join(regular_lines)
output_text += "\n<!-- end regular lines -->\n\n"
output_text += "<!-- add random lines -->\n"
output_text += "".join(random_lines)
output_text += "\n<!-- end random lines -->\n"

with open("plants.sdf", "w", encoding="utf-8") as file:
    file.write(output_text)

print("File has been saved as plants.sdf")
