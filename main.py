import io
import time
import os
import tempfile
import matplotlib
matplotlib.use('Agg')  # Disable GUI windows for thread safety on server
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field
from planner import DijkstraPlanner

app = FastAPI(
    title="Robot Path Planning API",
    description="Dijkstra Grid-Based Planning with JSON analytics, Matplotlib plots, and GIF animations"
)

# Input data schema for Swagger UI with default values
class PathRequest(BaseModel):
    sx: float = Field(-5.0, description="Start X coordinate (m)")
    sy: float = Field(-5.0, description="Start Y coordinate (m)")
    gx: float = Field(50.0, description="Goal X coordinate (m)")
    gy: float = Field(50.0, description="Goal Y coordinate (m)")
    resolution: float = Field(2.0, description="Grid resolution / cell size (m)")
    robot_radius: float = Field(1.0, description="Robot safety radius for obstacle inflation (m)")
    ox: list[float] = Field([20.0, 20.0, 20.0, 20.0, 40.0, 40.0, 40.0], description="Obstacles X coordinates array")
    oy: list[float] = Field([0.0, 10.0, 20.0, 30.0, 20.0, 30.0, 40.0], description="Obstacles Y coordinates array")
    
    # Map boundaries to ensure points fit safely inside the generated grid
    min_x: float = Field(-15.0, description="Map minimum X bound")
    min_y: float = Field(-15.0, description="Map minimum Y bound")
    max_x: float = Field(65.0, description="Map maximum X bound")
    max_y: float = Field(65.0, description="Map maximum Y bound")


# ENDPOINT 1: Structured JSON Data
@app.post("/plan/text", summary="Get readable JSON path calculation")
def plan_path_text(req: PathRequest):
    start_time = time.time()
    
    bounds = (req.min_x, req.min_y, req.max_x, req.max_y)
    planner = DijkstraPlanner(req.ox, req.oy, req.resolution, req.robot_radius, map_bounds=bounds)
    
    rx, ry, path_cost, _ = planner.planning(req.sx, req.sy, req.gx, req.gy)
    execution_time = time.time() - start_time

    if not rx:
        return {
            "status": "NOT_FOUND",
            "meta": {"execution_time_sec": round(execution_time, 4)},
            "message": "Path impossible. Goal is blocked or out of reach."
        }

    waypoints = [{"x": round(x, 2), "y": round(y, 2)} for x, y in zip(rx, ry)]

    return {
        "status": "SUCCESS",
        "summary": {
            "total_distance_meters": round(path_cost, 2),
            "number_of_waypoints": len(waypoints),
            "execution_time_seconds": round(execution_time, 4)
        },
        "initial_conditions": {
            "start": {"x": req.sx, "y": req.sy},
            "goal": {"x": req.gx, "y": req.gy},
            "grid_resolution": req.resolution,
            "robot_safety_radius": req.robot_radius
        },
        "path": waypoints
    }


# ENDPOINT 2: Static PNG Plot
@app.post("/plan/plot", summary="Get a static PNG plot of the path")
def plan_path_plot(req: PathRequest):
    bounds = (req.min_x, req.min_y, req.max_x, req.max_y)
    planner = DijkstraPlanner(req.ox, req.oy, req.resolution, req.robot_radius, map_bounds=bounds)
    rx, ry, _, _ = planner.planning(req.sx, req.sy, req.gx, req.gy)

    plt.figure(figsize=(10, 8))
    
    # 1. Plot original obstacle centers (black X marks)
    plt.plot(req.ox, req.oy, "xk", markersize=8, label="Obstacles (Centers)")
    
    # 2. Draw the grid cells inflated by the robot's radius (light red dots)
    cx, cy = [], []
    for ix in range(planner.x_width):
        for iy in range(planner.y_width):
            if planner.obstacle_map[ix][iy]:
                cx.append(planner.calc_position(ix, planner.min_x))
                cy.append(planner.calc_position(iy, planner.min_y))
    plt.plot(cx, cy, ".r", alpha=0.15, label="Inflated Zone (Safety Buffer)")

    # 3. Plot Start, Goal and Shortest Path Trajectory
    plt.plot(req.sx, req.sy, "og", markersize=10, label="Start (Robot)")
    plt.plot(req.gx, req.gy, "bs", markersize=10, label="Goal (Finish)")
    
    if rx:
        plt.plot(rx, ry, "-b", linewidth=3, label="Shortest Path (Dijkstra)")
        plt.title(f"Dijkstra Path Planning\nPath Length: {len(rx)} nodes", fontsize=14)
    else:
        plt.title("Robot Path Planning\n[PATH NOT FOUND]", fontsize=14, color="red")

    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.legend(loc="upper left")
    plt.grid(True)
    plt.axis("equal")

    img_buf = io.BytesIO()
    plt.savefig(img_buf, format='png', bbox_inches='tight')
    plt.close()
    img_buf.seek(0)

    return StreamingResponse(img_buf, media_type="image/png")


# ENDPOINT 3: Animated GIF of Algorithm execution with a 2-second freeze at the end
@app.post("/plan/animate", summary="Get an animated GIF of the algorithm searching")
def plan_path_animate(req: PathRequest):
    bounds = (req.min_x, req.min_y, req.max_x, req.max_y)
    planner = DijkstraPlanner(req.ox, req.oy, req.resolution, req.robot_radius, map_bounds=bounds)
    rx, ry, _, history = planner.planning(req.sx, req.sy, req.gx, req.gy)

    if not history:
        raise HTTPException(status_code=400, detail="Search history is empty. Check your bounds.")

    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Pre-render static elements to optimize animation build
    ax.plot(req.ox, req.oy, "xk", markersize=8)
    ax.plot(req.sx, req.sy, "og", markersize=10)
    ax.plot(req.gx, req.gy, "bs", markersize=10)
    
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.grid(True)
    ax.axis("equal")

    # Frame reduction for speed optimization
    step_size = max(1, len(history) // 60)
    animation_frames = history[::step_size]
    if history and animation_frames[-1] != history[-1]:
        animation_frames.append(history[-1])

    # Freeze effect: Remember where search ends, then add 30 duplicate frames (2 seconds hold at 15 fps)
    final_search_frame_idx = len(animation_frames) - 1
    animation_frames.extend([history[-1]] * 30)

    scat = ax.scatter([], [], c='cyan', marker='x', alpha=0.6, s=15, label="Explored Nodes")
    path_line, = ax.plot([], [], "-r", linewidth=2, label="Final Path")
    ax.legend(loc="upper left")

    def update(frame_idx):
        if frame_idx <= final_search_frame_idx:
            current_history = animation_frames[:frame_idx+1]
            scat.set_offsets(current_history)
            ax.set_title(f"Dijkstra Exploring Nodes... (Frame {frame_idx}/{final_search_frame_idx})")
        else:
            current_history = animation_frames[:final_search_frame_idx+1]
            scat.set_offsets(current_history)
        
        # Path line flashes on and stays solid through the end-hold padding frames
        if frame_idx >= final_search_frame_idx and rx:
            path_line.set_data(rx, ry)
            ax.set_title("Dijkstra Completed! (Showing Final Path)")
            
        return scat, path_line

    ani = FuncAnimation(fig, update, frames=len(animation_frames), interval=50, blit=False)

    # Use NamedTemporaryFile to bypass strict string path checks in newer Matplotlib versions
    with tempfile.NamedTemporaryFile(suffix=".gif", delete=False) as tmp:
        tmp_path = tmp.name

    try:
        writer = PillowWriter(fps=15)
        ani.save(tmp_path, writer=writer)
        plt.close(fig)

        def iterfile():
            with open(tmp_path, mode="rb") as f:
                yield from f
            os.unlink(tmp_path)  # Erase the temp file from the Mac disk seamlessly

        return StreamingResponse(iterfile(), media_type="image/gif")

    except Exception as e:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)
        plt.close(fig)
        raise HTTPException(status_code=500, detail=f"Animation compilation error: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)