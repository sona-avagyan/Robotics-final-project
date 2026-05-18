# Robot Path Planning API (Dijkstra Algorithm)

A production-ready robotics final project that implements a grid-based **Dijkstra Path Planning** microservice using **FastAPI**. The core algorithmic module is cleanly isolated from the web presentation layers and features an autonomous **Obstacle Inflation** mechanism to ensure collision-free navigation for mobile robots.

## 🚀 Key Features

* **JSON Path Analytics (`/plan/text`)**: Calculates and returns precise trajectory waypoints, global path cost (total distance in meters), and computation execution metrics.
* **Static Plot Visualizer (`/plan/plot`)**: Dynamically generates and streams a clean Matplotlib PNG graph marking the Start, Goal, optimal trajectory path, and the inflated inflation safety buffer zones.
* **Search Execution Animator (`/plan/animate`)**: Generates an animated GIF mapping the wave-front expansion of explored nodes. It features a custom backend 2-second "freeze frame" hold at the final state for enhanced trajectory inspection before looping.

## 🛠️ Algorithmic & Implementation Details

1.  **Safety Buffer Inflation**: To eliminate "corner clipping" and unsafe diagonal passes through tight obstacle edges, the system automatically swells obstacle dimensions based on the robot's physical constraints using the geometric formula:
    $$R_{safe} = R_{robot} + \frac{\text{Resolution}}{\sqrt{2}}$$
2.  **Thread-Safe Server Graphics**: Matplotlib is explicitly forced onto the headless, non-interactive `Agg` backend (`matplotlib.use('Agg')`). This guarantees thread safety and prevents internal server crashes on host environments like macOS.
3.  **Strict Path I/O Safe Streaming**: Built-in compatibility layers process and compile GIF data structures using secure OS temporary paths, ensuring stability under newer Python runtimes (Python 3.13+) while automatically reclaiming storage space post-transmission.

## 📦 Installation & Setup

### 1. Clone & Navigate to Repository
Bash
cd "robotics final project"

### 2. Install Required Dependencies
Ensure you have Python 3.10 or higher installed. Run the following command to download all backend and mathematical frameworks:

Bash
pip install -r requirements.txt

### 3. Spin Up the Web Server
Execute the server using Uvicorn:

Bash
python3 main.py
The server will initialize and begin listening for requests locally at http://127.0.0.1:8000.

## 📖 Interactive API Documentation (Swagger UI)
FastAPI generates automated, interactive documentation out-of-the-box.

Open your web browser and navigate to: http://127.0.0.1:8000/docs

Expand any planning endpoint (e.g., POST /plan/animate).

Click "Try it out", modify any coordinates or resolution attributes within the JSON payload body, and hit "Execute".

## 📊 Visual Reference Examples
Static Evaluation Map (/plan/plot)
Displays the generated matrix trajectory layout cleanly circumventing obstacles while highlighting safety boundaries.

## Dynamic Search Animation (/plan/animate)
An animated simulation outlining real-time Dijkstra node explorations and final route confirmation with a 2-second review delay.
