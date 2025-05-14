<div align="center">
  <h1>🤖 AI Shell Game Player 🥚</h1>
</div>

## 🌟 Game Overview

This project implements an AI system that plays the classic "Shell Game" (also known as the "Cup and Ball Game"). The system uses computer vision to:
1.  **Detect** and track three white/off-white cups on a surface.
2.  **Detect** a colored ball (e.g., green).
3.  **Track** which cup the ball is under as the cups are moved around.
4.  **Identify** the final location of the ball after the shuffling stops.
5.  **Communicate** with an ESP32-controlled robotic arm to lift the cup that it believes contains the ball.

The system is designed to be robust to varying lighting conditions by using adaptive white thresholding for cup detection and handles temporary pauses if a cup goes out of frame.

## ✨ Key Features

*   **Cup Detection & Tracking:**
    *   Detects three white/off-white cups using HSV color segmentation and ellipse fitting.
    *   Employs adaptive white thresholding to adjust to lighting changes.
    *   Uses OpenCV's `MultiTracker` (CSRT tracker) to track the cups once initially detected.
*   **Ball Detection:**
    *   Detects a colored ball (e.g., green) using HSV color segmentation.
*   **Ball-Under-Cup Logic:**
    *   Determines which cup the ball is under based on proximity or the last known state if the ball is hidden.
    *   Tracks the last moved cup to infer the ball's location when it's hidden.
*   **Motion Detection:**
    *   Tracks the movement of each cup.
    *   The game ends if no cup has moved for a set `motion_timeout`.
*   **Robotic Arm Integration:**
    *   Communicates with an ESP32 via serial port.
    *   Sends commands to the ESP32 to move a robotic arm to the predicted cup and lift it using a magnet.
*   **Game State Management:**
    *   Pauses the game if a cup goes out of frame and resumes when all cups are visible.
    *   Handles game end conditions and reset functionality.
*   **Visual Feedback:**
    *   Displays the camera feed with detected cups, ball, tracking boxes, and game status information.
*   **IP Camera Input:**
    *   Uses an IP camera stream (e.g., from a smartphone app like IP Webcam) for video input.

## 🛠️ Tech Stack

*   **Programming Language:** Python
*   **Computer Vision:** OpenCV
*   **Robotics Control:** ESP32 (via Serial Communication with PySerial)
*   **Libraries:** NumPy, time, serial

## 🔩 Hardware Requirements

1.  **Computer:** To run the Python script.
2.  **IP Camera:** A smartphone with an app like "IP Webcam" or any other IP camera.
3.  **ESP32 Microcontroller:** To control the robotic arm.
4.  **Robotic Arm:**
    *   3-DOF or 4-DOF arm capable of reaching and lifting cups.
    *   Servos for arm joints.
    *   An electromagnet or other lifting mechanism for the cups.
5.  **Cups:** Three identical white or off-white cups.
6.  **Ball:** A small, brightly colored ball (e.g., green) that fits under the cups.
7.  **Power Supply:** For the ESP32 and robotic arm.
8.  **USB Cable:** For ESP32 programming and serial communication from the PC.

## 💾 Software & Installation

1.  **Python 3.x:** Install from [python.org](https://www.python.org/).
2.  **Python Libraries:**
    ```bash
    pip install opencv-python numpy pyserial
    ```
    *(Note: `cv2.legacy` trackers are used. Ensure your OpenCV version includes this, or adapt to newer tracker APIs if necessary.)*
3.  **ESP32 Setup:**
    *   Arduino IDE or PlatformIO for programming the ESP32.
    *   An ESP32 sketch that:
        *   Initializes servo motors and the electromagnet.
        *   Listens for serial commands in the format: `"servo1_angle,servo2_angle,servo3_angle,magnet_state\n"` (e.g., `"180,120,120,0"` or `"180,90,0,1"` for home position).
        *   Controls the arm and magnet based on received commands.
4.  **IP Camera App:** (e.g., IP Webcam for Android) installed on your smartphone.

## ⚙️ Configuration (Python Script)

Before running, adjust these parameters at the top of the Python script:

*   **`url`**: IP camera stream URL (e.g., `"http://192.168.49.1:8080/video"`).
*   **`esp32_serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)`**:
    *   Change `'/dev/ttyUSB0'` (Linux) or `'COMx'` (Windows) to your ESP32's serial port.
*   **Color Ranges (HSV):**
    *   `white_lower`, `white_upper`: For cup detection. These are adaptive but initial values can be tuned.
    *   `ball_lower`, `ball_upper`: For ball detection (currently set for green).
*   **`adaptive_white_threshold`**, **`adaptive_learning_rate`**: Parameters for dynamic white detection.
*   **`motion_threshold`**: Minimum pixel movement to register cup motion.
*   **`motion_timeout`**: Seconds of no cup motion before the game ends.
*   **Robotic Arm Positions (within `if game_ended:` block):**
    *   `cup_pos`: Servo angles for reaching each cup (Left, Middle, Right).
    *   `lift_pos`: Servo angles for lifting the cup.
    *   `home_cmd`: Servo angles for the arm's home position.
    *   **These angles are CRITICAL and must be calibrated precisely for your specific robotic arm.**

## ▶️ How to Run

1.  **Hardware Setup:**
    *   Set up the playing surface with good, consistent lighting.
    *   Assemble and calibrate your robotic arm.
    *   Connect the robotic arm (servos, electromagnet) to the ESP32.
    *   Connect the ESP32 to your PC via USB.
2.  **Software Setup:**
    *   Install all prerequisite software and libraries.
    *   Upload the control sketch to your ESP32.
    *   Start your IP camera stream and ensure it's accessible from your PC.
3.  **Configure Script:** Modify the Python script parameters as described above, especially the IP camera URL, ESP32 serial port, and **robotic arm servo angles**.
4.  **Run the Python Script:**
    ```bash
    python your_shell_game_script_name.py
    ```
5.  An OpenCV window will appear:
    *   Place the three white cups and the ball in the camera's view.
    *   The system will attempt to detect the cups. Once three cups are consistently detected, tracking will begin.
    *   Play the shell game by moving the cups.
    *   When you stop moving the cups, the `motion_timeout` will eventually trigger the game end.
    *   The system will then send commands to the robotic arm to lift the predicted cup.
6.  **Controls:**
    *   Press 'q' to quit the application.
    *   Press 'r' to reset the game (re-detect cups and restart tracking).

## 🔁 Workflow Logic

1.  **Initialization:**
    *   Connect to the IP camera and ESP32 serial port.
    *   Initialize game state variables.
2.  **Cup Detection (Initial):**
    *   On the first frame or after a reset, detect three white cups using adaptive HSV thresholding and ellipse fitting.
    *   If three cups are found, initialize `cv2.legacy.MultiTracker_create()` with CSRT trackers for each cup.
3.  **Main Loop:**
    *   Read a frame from the IP camera.
    *   **Cup Tracking:** Update cup positions using the multi-tracker.
    *   **Game Pause/Resume:** Check if any cup is out of frame. Pause game logic if a cup is missing and resume when all are back.
    *   **Ball Detection:** Detect the colored ball.
    *   **Ball-Under-Cup Logic:** Determine which cup (if any) the ball is currently under or was last seen under.
    *   **Motion Tracking:** Monitor each cup for movement. Update `last_motion_times`.
    *   **Game End Condition:** If `motion_timeout` is exceeded (no cup has moved), set `game_ended = True`.
4.  **Game Ended State:**
    *   Display "GAME ENDED" message.
    *   Determine the `wantedCup` (0 for Left, 1 for Middle, 2 for Right) based on the final ball location.
    *   If not already sent for this game end, send a sequence of commands to the ESP32 to:
        1.  Move arm to `cup_pos` (magnet off).
        2.  Activate magnet.
        3.  Move arm to `lift_pos` (magnet on).
        4.  (Optional: Lower cup back).
        5.  (Optional: Deactivate magnet).
        6.  Return arm to `home_cmd`.
    *   Allow game reset by pressing 'r'.
5.  **Display:** Show the camera feed with cup bounding boxes/ellipses, ball position, cup names, game status, and other debug information.

## 🔧 Calibration & Tuning

*   **Robotic Arm Servo Angles:** This is the **most critical** part to calibrate. The `cup_pos`, `lift_pos`, and `home_cmd` values in the script *must* be precisely determined for your specific arm's dimensions and servo ranges. Incorrect values can lead to improper operation or damage to the arm/cups.
*   **Color Ranges (HSV):** Adjust `white_lower`/`white_upper` and `ball_lower`/`ball_upper` based on your specific cups, ball color, and lighting conditions. Use an HSV color picker tool to find optimal ranges.
*   **`adaptive_white_threshold` & `adaptive_learning_rate`:** Tune for responsiveness to lighting changes without being too sensitive.
*   **`motion_threshold` & `motion_timeout`:** Adjust based on how quickly you want the game to end after shuffling stops.
*   **Camera Position & Lighting:** Ensure consistent, good lighting. The camera should have a clear, stable view of the entire playing area.

## ⚠️ Important Notes

*   The cup detection relies on the tops of the cups being somewhat elliptical and white/off-white.
*   The robotic arm control sequence is specific. Ensure your ESP32 sketch correctly interprets the comma-separated angle and magnet state commands.
*   The `cv2.legacy.TrackerCSRT_create()` tracker is used. Performance can vary. If issues arise, consider experimenting with other trackers available in OpenCV or newer tracking APIs.
*   Test the robotic arm movements thoroughly without the vision system first to ensure safety and correct calibration.
*   The system assumes three cups. Modifying for a different number would require changes to the logic and potentially the cup detection/naming.

---

This README provides a comprehensive guide to your AI Shell Game project. Good luck!
