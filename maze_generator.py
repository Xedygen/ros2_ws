import numpy as np
import random
import os

# --- Configuration ---
cells_x = 64  # Number of cells in X direction
cells_y = 64  # Number of cells in Y direction
cell_size = 4.0  # Size of each cell in meters (scaled 4x from original 1m)
wall_thickness = 0.04  # Wall thickness in meters (scaled 4x from 0.01m)
wall_height = 4.0  # Wall height in meters (scaled 4x from 1m)

# Calculate maze dimensions
maze_width = cells_x * cell_size
maze_height = cells_y * cell_size

output_dir = os.path.join(os.getcwd(), "worlds")
os.makedirs(output_dir, exist_ok=True)

# Initialize visited array
visited = [[False] * cells_x for _ in range(cells_y)]

# Store wall segments: (x_pos, y_pos, is_horizontal)
walls = []

def carve(start_r, start_c):
    """Carves out paths using the Recursive Backtracker algorithm."""
    stack = [(start_r, start_c)]
    visited[start_r][start_c] = True
    
    while stack:
        r, c = stack[-1]
        neighbors = []
        
        # Check unvisited neighbors and track which wall would be removed
        if r > 0 and not visited[r - 1][c]:  # North
            neighbors.append((r - 1, c, 'north'))
        if r < cells_y - 1 and not visited[r + 1][c]:  # South
            neighbors.append((r + 1, c, 'south'))
        if c > 0 and not visited[r][c - 1]:  # West
            neighbors.append((r, c - 1, 'west'))
        if c < cells_x - 1 and not visited[r][c + 1]:  # East
            neighbors.append((r, c + 1, 'east'))
        
        if neighbors:
            nr, nc, direction = random.choice(neighbors)
            visited[nr][nc] = True
            # Note: We don't add walls here; we'll add all walls and skip carved ones
            stack.append((nr, nc))
        else:
            stack.pop()

def generate_all_walls():
    """Generate all possible walls in the grid."""
    horizontal_walls = set()
    vertical_walls = set()
    
    # Horizontal walls (between rows)
    for r in range(cells_y + 1):
        for c in range(cells_x):
            horizontal_walls.add((r, c))
    
    # Vertical walls (between columns)
    for r in range(cells_y):
        for c in range(cells_x + 1):
            vertical_walls.add((r, c))
    
    return horizontal_walls, vertical_walls

def get_walls_to_keep():
    """Get only the walls that should remain (not carved paths)."""
    h_walls_keep = set()
    v_walls_keep = set()
    
    # Add horizontal walls where there's NO path between cells
    for r in range(1, cells_y):
        for c in range(cells_x):
            # Keep wall if either cell is unvisited OR both are visited but no path exists
            # In our maze, if both cells are visited, there's a path, so keep wall if not both visited
            if not (visited[r-1][c] and visited[r][c]):
                h_walls_keep.add((r, c))
    
    # Add vertical walls where there's NO path between cells
    for r in range(cells_y):
        for c in range(1, cells_x):
            if not (visited[r][c-1] and visited[r][c]):
                v_walls_keep.add((r, c))
    
    # Add all border walls
    # Top border (r=0)
    for c in range(cells_x):
        h_walls_keep.add((0, c))
    # Bottom border (r=cells_y)
    for c in range(cells_x):
        h_walls_keep.add((cells_y, c))
    # Left border (c=0)
    for r in range(cells_y):
        v_walls_keep.add((r, 0))
    # Right border (c=cells_x)
    for r in range(cells_y):
        v_walls_keep.add((r, cells_x))
    
    return h_walls_keep, v_walls_keep

def wall_to_sdf_coords(r, c, is_horizontal):
    """Convert grid coordinates to SDF world coordinates."""
    # Center the maze at origin
    offset_x = -maze_width / 2
    offset_y = -maze_height / 2
    
    if is_horizontal:
        # Horizontal wall spans along X axis
        x = offset_x + (c + 0.5) * cell_size
        y = offset_y + r * cell_size
        size_x = cell_size
        size_y = wall_thickness
    else:
        # Vertical wall spans along Y axis
        x = offset_x + c * cell_size
        y = offset_y + (r + 0.5) * cell_size
        size_x = wall_thickness
        size_y = cell_size
    
    return x, y, size_x, size_y

def generate_sdf():
    """Generate the complete SDF file."""
    # Run maze generation FIRST
    carve(0, 0)
    
    sdf_content = f'''<?xml version="1.0"?>
<sdf version='1.8'>
  <world name='generated_maze'>
    <physics name='1ms' type='ignored'>
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <plugin name="gz::sim::systems::Physics" filename="gz-sim-physics-system" />
    <plugin name="gz::sim::systems::UserCommands" filename="gz-sim-user-commands-system" />
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="gz-sim-scene-broadcaster-system" />
    <plugin name="gz::sim::systems::Contact" filename="gz-sim-contact-system" />
    
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-{maze_width*0.6} -{maze_height*0.6} {maze_width*0.9} 0 0.7 0.8</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
    
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 40 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>4000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic' />
    
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{maze_width*2} {maze_height*2}</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>{maze_width*2} {maze_height*2}</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
      <pose>0 0 0 0 0 0</pose>
    </model>
    
    <model name='maze_walls'>
      <pose>0 0 {wall_height/2} 0 0 0</pose>
      <static>true</static>
'''
    
    # Generate maze walls based on visited cells
    h_walls, v_walls = get_walls_to_keep()
    
    wall_id = 0
    
    # Add horizontal walls
    for r, c in sorted(h_walls):
        x, y, sx, sy = wall_to_sdf_coords(r, c, True)
        wall_id += 1
        sdf_content += f'''      <link name='wall{wall_id}'>
        <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
        <collision name='wall{wall_id}_collision'>
          <geometry>
            <box>
              <size>{sx:.2f} {sy:.2f} {wall_height:.2f}</size>
            </box>
          </geometry>
        </collision>
        <visual name='wall{wall_id}_visual'>
          <geometry>
            <box>
              <size>{sx:.2f} {sy:.2f} {wall_height:.2f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0 0.1 0.2 1</diffuse>
            <specular>0 0.01 0.05 1</specular>
          </material>
        </visual>
      </link>
'''
    
    # Add vertical walls
    for r, c in sorted(v_walls):
        x, y, sx, sy = wall_to_sdf_coords(r, c, False)
        wall_id += 1
        sdf_content += f'''      <link name='wall{wall_id}'>
        <pose>{x:.2f} {y:.2f} 0 0 0 0</pose>
        <collision name='wall{wall_id}_collision'>
          <geometry>
            <box>
              <size>{sx:.2f} {sy:.2f} {wall_height:.2f}</size>
            </box>
          </geometry>
        </collision>
        <visual name='wall{wall_id}_visual'>
          <geometry>
            <box>
              <size>{sx:.2f} {sy:.2f} {wall_height:.2f}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0 0.1 0.2 1</diffuse>
            <specular>0 0.01 0.05 1</specular>
          </material>
        </visual>
      </link>
'''
    
    sdf_content += '''    </model>
  </world>
</sdf>'''
    
    return sdf_content, wall_id

# Generate the SDF
sdf_content, num_walls = generate_sdf()

# Save to file
output_path = os.path.join(output_dir, "generated_maze.sdf")
with open(output_path, 'w') as f:
    f.write(sdf_content)

print(f"Generated maze with {num_walls} walls")
print(f"Maze size: {cells_x}x{cells_y} cells ({maze_width}m x {maze_height}m)")
print(f"Saved to: {output_path}")