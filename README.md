# Lane Tracking System

This project implements a real-time lane detection and tracking system using OpenCV and Kalman Filter. It processes video frames to detect lane lines and tracks them across frames, making it suitable for applications in autonomous driving, driver assistance systems, and road safety research.

## Features
- Lane detection using Canny edge detection and Hough Transform (standard and probabilistic)
- Lane tracking using a Kalman Filter for robust estimation
- Real-time video processing
- Modular code structure for easy extension
- Supports both straight and curved (overtaking) lane detection
- Robust to missing or noisy lane markings

## How It Works
1. **Lane Detection**: The `LaneDetector` class processes each video frame to detect lane lines using edge detection and Hough Transform. It identifies the left and right lane boundaries, supporting both straight and curved lanes by clustering and fitting polynomials to detected line segments.
2. **Lane Tracking**: The `LaneTracker` class uses a Kalman Filter to track the detected lanes across frames, providing smooth and robust lane position estimates even when detections are noisy or missing.
3. **Visualization**: Detected and tracked lanes are drawn on the video frames for visualization. The system can highlight both straight and curved lanes, making it suitable for real-world road scenarios.

## Application
- Autonomous vehicles
- Advanced driver-assistance systems (ADAS)
- Road lane departure warning systems
- Traffic monitoring and analysis
- Research and prototyping for computer vision in transportation

## Requirements
- Python 3.7+
- OpenCV
- NumPy
- SciPy

## Installation
Install the required packages using pip:

```bash
pip install opencv-python numpy scipy
```

## Usage
1. Place your test video in the project directory (default: `Lane Detection Test Video 01.mp4`).
2. Run the main script:

```bash
python main.py --path "Lane Detection Test Video 01.mp4"
```

You can specify a different video path with the `--path` argument.

## File Structure
- `main.py`: Main script to run lane detection and tracking on a video.
- `detect.py`: Contains the `LaneDetector` class for lane detection (supports both straight and curved lanes).
- `track.py`: Contains the `LaneTracker` class for lane tracking using Kalman Filter.
- `Lane Detection Test Video 01.mp4`: Example test video.

## Customization
- **Parameter Tuning**: You can adjust parameters in `detect.py` (e.g., Canny thresholds, Hough Transform settings) and `track.py` (e.g., Kalman Filter noise scales) to suit different road conditions, camera angles, or video qualities.
- **Curved Lane Support**: The detection algorithm is designed to handle overtaking and curved lanes by fitting polynomials to detected line segments. You can further enhance this by increasing the number of points used for fitting or by visualizing the full curve.
- **Visualization**: You can modify the drawing code in `main.py` to highlight lane areas, add overlays, or display additional information (e.g., lane curvature, vehicle offset).

## Notes
- Press `q` to quit the video window.
- The system is robust to missing or partial lane markings, but performance may vary with extreme lighting or weather conditions.
- For best results, use high-quality road videos with clear lane markings.

## Troubleshooting
- If you encounter errors related to missing packages, ensure you have installed all requirements with the correct Python version.
- If the video does not display, check your OpenCV installation and ensure your system supports GUI windows.
- For performance issues, try reducing the video resolution or adjusting detection parameters.

## License
This project is provided for educational and research purposes. Feel free to use and modify it for your own experiments or learning.
