import matplotlib.pyplot as plt
from matplotlib import colors
import matplotlib.animation as animation
from matplotlib.patches import Patch
import numpy as np
import os
import sys

# === Configuration ===
MAZE_DIM = 16      # Internal maze size
BORDER_WIDTH = 1   # Width of the surrounding border
FRAME_DELAY_MS = 300 # Slower speed to see backtracking clearly
PATH_FILE = 'path_log.txt'

# Calculated total grid size (18x18)
TOTAL_GRID_SIZE = MAZE_DIM + (BORDER_WIDTH * 2)

# --- Color & State Definitions ---
# 0 = Free Space (White)
# 1 = Wall/Border (Black)
# 2 = Current Path Head (Red) - First visit
# 3 = Backtracked/Visited (Blue) - Second visit+
COLOR_MAP = colors.ListedColormap(['white', 'black', 'red', 'blue'])
# Define boundaries for 4 discrete states (0, 1, 2, 3)
BOUNDS = [-0.5, 0.5, 1.5, 2.5, 3.5]
NORM = colors.BoundaryNorm(BOUNDS, COLOR_MAP.N)

# --- Global State Tracking ---
# A set to store unique coordinates (r,c) we have already seen
visited_coords = set()

def load_path_coordinates(filename):
    """Reads a text file of 'row,col' coordinates into a list."""
    coords = []
    if not os.path.exists(filename):
        print(f"Error: Path file '{filename}' not found.")
        sys.exit(1)
    try:
        with open(filename, 'r') as f:
            for line in f:
                clean_line = line.strip()
                if clean_line and '#' not in clean_line:
                    r_str, c_str = clean_line.split(',')
                    coords.append((int(r_str), int(c_str)))
    except ValueError as e:
        print(f"Error parsing coordinate file: {e}")
        sys.exit(1)
    return coords

def initialize_grid():
    """Creates the 18x18 grid with black borders and white interior."""
    # Start with a grid completely filled with walls (1 = black)
    grid = np.ones((TOTAL_GRID_SIZE, TOTAL_GRID_SIZE), dtype=int)
    # Calculate inner area indices
    start_idx = BORDER_WIDTH
    end_idx = TOTAL_GRID_SIZE - BORDER_WIDTH
    # Fill the inner 16x16 area with free space (0 = white)
    grid[start_idx:end_idx, start_idx:end_idx] = 0
    return grid

# --- Main Setup ---
path_coords = load_path_coordinates(PATH_FILE)
grid_data = initialize_grid()

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_title(f"DFS Path Animation ({MAZE_DIM}x{MAZE_DIM} + Border)")

# Initial plot
img = ax.imshow(grid_data, cmap=COLOR_MAP, norm=NORM, origin='upper')

# Add grid lines and hide ticks
ax.grid(which='major', axis='both', linestyle='-', color='gray', linewidth=0.5)
ax.set_xticks(np.arange(-0.5, TOTAL_GRID_SIZE, 1))
ax.set_yticks(np.arange(-0.5, TOTAL_GRID_SIZE, 1))
ax.set_xticklabels([])
ax.set_yticklabels([])

# Add a Legend
legend_elements = [
    Patch(facecolor='white', edgecolor='gray', label='Free'),
    Patch(facecolor='black', edgecolor='gray', label='Wall/Border'),
    Patch(facecolor='red', edgecolor='gray', label='Current Head (First Visit)'),
    Patch(facecolor='blue', edgecolor='gray', label='Backtracked (Re-visited)')
]
ax.legend(handles=legend_elements, loc='upper right', bbox_to_anchor=(1.35, 1.0))


# --- Animation Function ---
def update_frame(frame_idx):
    # Stop updating if we run out of coordinates (handles the extra pause frames)
    if frame_idx >= len(path_coords):
        return img,

    # Get internal coordinate (0-15)
    internal_coord = path_coords[frame_idx]
    internal_r, internal_c = internal_coord

    # Safety check boundaries
    if not (0 <= internal_r < MAZE_DIM and 0 <= internal_c < MAZE_DIM):
        return img,

    # Offset to grid coordinate (1-16)
    grid_r = internal_r + BORDER_WIDTH
    grid_c = internal_c + BORDER_WIDTH

    # --- Core Logic ---
    if internal_coord not in visited_coords:
        # First time seeing this coordinate: Turn RED (2)
        grid_data[grid_r, grid_c] = 2
        visited_coords.add(internal_coord)
    else:
        # Seen before (backtracking): Turn BLUE (3)
        grid_data[grid_r, grid_c] = 3
    # ------------------
    
    img.set_data(grid_data)
    return img,

# Create animation
ani = animation.FuncAnimation(
    fig, 
    update_frame, 
    frames=len(path_coords) + 5, # Add extra frames to pause at end
    interval=FRAME_DELAY_MS, 
    blit=True,
    repeat=False
)

plt.tight_layout()
plt.show()