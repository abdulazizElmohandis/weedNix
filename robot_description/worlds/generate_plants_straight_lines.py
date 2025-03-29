import random

def generate_new_coordinates(prev_x, base_y):
    """
    Generate new (X, Y) coordinates while maintaining:
    - The average spacing between plants.
    - The min/max gap between consecutive plants.
    - The min/max deviation from the row line.
    """
    # Generate a new X offset within the predefined range (rounded to 3 decimals)
    delta_x = round(random.uniform(0.21, 0.74), 3)
    new_x = round(prev_x + delta_x, 3)
    
    # Generate a new Y deviation from the base Y value
    y_offset = round(random.uniform(0.0053, 0.2530) * random.choice([-1, 1]), 3)
    new_y = round(base_y + y_offset, 3)
    
    return new_x, new_y

# XML template for the plant models
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
    </model>
"""

# Configuration parameters
num_rows = 3          # Number of parallel rows
row_length = 50       # Desired row length (distance between the first and last plant in a row)
row_spacing = 3       # Distance between rows
original_x = -16      # X starting position of the first row
original_y = -9       # Y starting position of the first row
original_name = "big_plant"  # Base plant name

output = []

# Generate rows of plants
for row in range(num_rows):
    base_y = original_y + row * row_spacing  # Base Y value for the current row (remains constant within a row)
    current_x = original_x  # X starting position for the row
    plant_num = 1           # Counter for plants in the row
    
    # Generate plants until the row reaches the desired length
    while current_x - original_x < row_length:
        name = f"{original_name}_Row{row+1}_Plant{plant_num}"
        # Compute new X and Y coordinates
        current_x, current_y = generate_new_coordinates(current_x, base_y)
        pose = f"{current_x} {current_y} 0 0 -0 0"
        output.append(template.format(name, pose))
        plant_num += 1

# Combine all outputs into a single string
output_text = "".join(output)

# Save the generated plant models to a file
filename = "plants.sdf"
with open(filename, "w", encoding="utf-8") as file:
    file.write(output_text)

print(f"File saved successfully: {filename}")
