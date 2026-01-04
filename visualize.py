import matplotlib.pyplot as plt
from matplotlib import colors
import matplotlib.animation as animation
from matplotlib.patches import Patch
from matplotlib.collections import LineCollection
import numpy as np
import os
import sys
import random

# === Configuration ===
MAZE_DIM = 16        # Internal maze size (16x16)
BORDER_WIDTH = 1     # Width of the surrounding border
FRAME_DELAY_MS = 50  # Animation speed
PATH_FILE = 'path_log.txt'
MAZE_SEED = 42       # Fixed seed to keep walls consistent if using dummy data

# Calculated total grid size
TOTAL_GRID_SIZE = MAZE_DIM + (BORDER_WIDTH * 2)

# --- Color Definitions (Flat UI Palette) ---
COLOR_PALETTE = [
    '#ecf0f1',  # 0: Clouds (Free Space/Unvisited)
    '#2c3e50',  # 1: Wet Asphalt (Outer Border)
    '#2ecc71',  # 2: Emerald (Forward Path)
    '#e74c3c'   # 3: Alizarin (Backtrack)
]

# Define boundaries for 4 discrete states
BOUNDS = [-0.5, 0.5, 1.5, 2.5, 3.5]
COLOR_MAP = colors.ListedColormap(COLOR_PALETTE)
NORM = colors.BoundaryNorm(BOUNDS, COLOR_MAP.N)

# --- Global State ---
visited_coords = set()

def simulate_maze_structure(rows, cols, seed=None):
    """
    Runs the Recursive Backtracker algorithm to generate:
    1. A list of internal walls to DRAW.
    2. A path log (if we need dummy data).
    """
    if seed is not None:
        random.seed(seed)
        
    # Track visited cells for generation
    visited_gen = set()
    stack = []
    
    # Track walls: Start with ALL walls, remove them as we carve.
    # Walls are defined as:
    # H-Walls: ((r, c), (r+1, c)) -> Wall between row r and r+1
    # V-Walls: ((r, c), (r, c+1)) -> Wall between col c and c+1
    # We will simply store all existing walls and remove them.
    
    # Actually, easier approach for plotting:
    # Store "removed" walls during carve. Then iterate all possible walls,
    # and if not in "removed", add to "walls_to_draw".
    removed_h = set() # (r, c) means wall between r and r+1
    removed_v = set() # (r, c) means wall between c and c+1
    
    # Start at (0,0) (Bottom-Right in our visualizer logic)
    start_pos = (0, 0)
    stack.append(start_pos)
    visited_gen.add(start_pos)
    
    # For dummy path generation
    dummy_path = [start_pos]
    
    while stack:
        r, c = stack[-1]
        
        # Neighbors: (r, c, wall_type, wall_coords_to_remove)
        potential_neighbors = []
        
        # North (r+1)
        if r < rows - 1 and (r + 1, c) not in visited_gen:
            potential_neighbors.append(((r + 1, c), 'h', (r, c)))
        # South (r-1)
        if r > 0 and (r - 1, c) not in visited_gen:
            potential_neighbors.append(((r - 1, c), 'h', (r - 1, c)))
        # East (c+1) - Visually Left due to invert
        if c < cols - 1 and (r, c + 1) not in visited_gen:
            potential_neighbors.append(((r, c + 1), 'v', (r, c)))
        # West (c-1) - Visually Right due to invert
        if c > 0 and (r, c - 1) not in visited_gen:
            potential_neighbors.append(((r, c - 1), 'v', (r, c - 1)))
            
        if potential_neighbors:
            # Choose random neighbor
            (nr, nc), w_type, w_coords = random.choice(potential_neighbors)
            
            # Carve (remove wall)
            if w_type == 'h':
                removed_h.add(w_coords)
            else:
                removed_v.add(w_coords)
            
            visited_gen.add((nr, nc))
            stack.append((nr, nc))
            dummy_path.append((nr, nc))
        else:
            stack.pop()
            if stack:
                dummy_path.append(stack[-1])
                
    # --- Construct list of walls to draw ---
    walls_to_draw = []
    
    # Horizontal walls (between rows)
    # Total rows=MAZE_DIM. Internal boundaries are 0..MAZE_DIM-2
    for r in range(rows - 1):
        for c in range(cols):
            if (r, c) not in removed_h:
                # Wall exists between row r and r+1
                # Coordinates for plotting:
                # Grid Y boundary is r + 1 (since 0 is first cell) + BORDER offset
                # Let's align with matplotlib grid:
                # Cell (r,c) center is at index r+BORDER, c+BORDER.
                # Boundary between r and r+1 is at y = r + BORDER + 0.5
                y = r + BORDER_WIDTH + 0.5
                x_start = c + BORDER_WIDTH - 0.5
                x_end = c + BORDER_WIDTH + 0.5
                walls_to_draw.append([(x_start, y), (x_end, y)])

    # Vertical walls (between cols)
    for r in range(rows):
        for c in range(cols - 1):
            if (r, c) not in removed_v:
                # Wall exists between col c and c+1
                x = c + BORDER_WIDTH + 0.5
                y_start = r + BORDER_WIDTH - 0.5
                y_end = r + BORDER_WIDTH + 0.5
                walls_to_draw.append([(x, y_start), (x, y_end)])
                
    return walls_to_draw, dummy_path

# --- Initialization ---

# 1. Generate Maze Structure
walls_segments, generated_dummy_path = simulate_maze_structure(MAZE_DIM, MAZE_DIM, seed=MAZE_SEED)

def load_path_coordinates(filename):
    """Reads path file or returns generated dummy path."""
    coords = []
    if not os.path.exists(filename):
        print(f"Note: '{filename}' not found. Using generated dummy path...")
        return generated_dummy_path
        
    try:
        with open(filename, 'r') as f:
            for line in f:
                clean_line = line.strip()
                if clean_line and '#' not in clean_line:
                    parts = clean_line.split(',')
                    if len(parts) >= 2:
                        coords.append((int(float(parts[0])), int(float(parts[1]))))
    except ValueError as e:
        print(f"Error parsing coordinate file: {e}")
        sys.exit(1)
    return coords

def initialize_grid():
    """Creates the grid base."""
    # Outer border is 1 (Wall color), Inner is 0 (Free color)
    grid = np.ones((TOTAL_GRID_SIZE, TOTAL_GRID_SIZE), dtype=int)
    start_idx = BORDER_WIDTH
    end_idx = TOTAL_GRID_SIZE - BORDER_WIDTH
    grid[start_idx:end_idx, start_idx:end_idx] = 0
    return grid

# --- Setup Plot ---
path_coords = load_path_coordinates(PATH_FILE)
grid_data = initialize_grid()

fig, ax = plt.subplots(figsize=(9, 9), facecolor='#fdfdfd')
ax.set_title(f"Maze Navigation Log\n({MAZE_DIM}x{MAZE_DIM} Grid - Start @ Bottom Right)", fontsize=14, pad=15)

# Plot Heatmap
img = ax.imshow(grid_data, cmap=COLOR_MAP, norm=NORM, origin='lower')

# Plot Internal Walls
# We use LineCollection for efficiency and cleaner rendering
wall_collection = LineCollection(walls_segments, colors='#2c3e50', linewidths=2)
ax.add_collection(wall_collection)

# Orientation
ax.invert_xaxis() 

# Styling
ax.set_xticks([])
ax.set_yticks([])
for spine in ax.spines.values():
    spine.set_visible(False)

# Custom Legend
legend_elements = [
    Patch(facecolor=COLOR_PALETTE[0], edgecolor='#bdc3c7', label='Free Space'),
    Patch(facecolor=COLOR_PALETTE[1], edgecolor='#bdc3c7', label='Outer Border'),
    Patch(facecolor=COLOR_PALETTE[1], label='Internal Wall'), # Hack to show line color
    Patch(facecolor=COLOR_PALETTE[2], edgecolor='none', label='Forward Path'),
    Patch(facecolor=COLOR_PALETTE[3], edgecolor='none', label='Backtracking')
]
ax.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=5, frameon=False)

# --- Animation ---
def update_frame(frame_idx):
    if frame_idx >= len(path_coords):
        return img,

    internal_coord = path_coords[frame_idx]
    internal_r, internal_c = internal_coord

    if not (0 <= internal_r < MAZE_DIM and 0 <= internal_c < MAZE_DIM):
        return img,

    # Map to grid indices
    grid_r = internal_r + BORDER_WIDTH
    grid_c = internal_c + BORDER_WIDTH

    if internal_coord not in visited_coords:
        grid_data[grid_r, grid_c] = 2 # Forward
        visited_coords.add(internal_coord)
    else:
        grid_data[grid_r, grid_c] = 3 # Backtrack
    
    img.set_data(grid_data)
    return img,

ani = animation.FuncAnimation(
    fig, 
    update_frame, 
    frames=len(path_coords) + 20, 
    interval=FRAME_DELAY_MS, 
    blit=True,
    repeat=False
)

# Start Fullscreen
try:
    fig.canvas.manager.full_screen_toggle()
except AttributeError:
    pass

plt.tight_layout()
plt.show()