# كتابة اسم الكوب

import cv2
import numpy as np
import time

# Initialize webcam with IP camera URL
url = "http://192.168.100.4:8080/video"
cap = cv2.VideoCapture(url)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Define flexible color range for white/off-white cups in HSV
# Wider range for hue (all colors), lower saturation (more white), higher value (brighter)
white_lower = np.array([0, 0, 180])       # Lower HSV - very wide hue, low saturation, high value
white_upper = np.array([180, 40, 255])    # Upper HSV - captures off-whites

# Define color range for the green ball
ball_lower = np.array([35, 100, 100])  # Lower HSV for green
ball_upper = np.array([85, 255, 255])  # Upper HSV for green

# Adaptive white detection parameters
adaptive_white_threshold = 200  # Initial brightness threshold
adaptive_learning_rate = 0.01   # How quickly we adjust to lighting changes

# Global variables for ball state
ball_position = None
ball_under_cup = None
last_nearest_cup = None
game_paused = False
missing_cup_index = None

# Add for motion tracking
last_motion_times = [time.time(), time.time(), time.time()]
last_cup_centers = [None, None, None]
motion_threshold = 10  # pixels
motion_timeout = 10     # seconds
game_ended = False
last_wanted_cup_idx = None  # Track the wanted cup after game ends
last_moved_cup_idx = None   # Track the last moved cup

# Track the last cup that had the ball under it before ending
last_ball_under_cup_idx = None

# Flag to indicate if any cup has the ball under it
cup_has_ball_flag = False

def update_white_threshold(frame):
    """Dynamically adjust white detection based on overall brightness"""
    global adaptive_white_threshold
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    current_brightness = np.mean(gray)
    # Slowly adapt the threshold towards current brightness
    adaptive_white_threshold = (1 - adaptive_learning_rate) * adaptive_white_threshold + \
                              adaptive_learning_rate * current_brightness * 0.9
    # Update the white range
    white_lower[2] = max(150, adaptive_white_threshold - 50)  # Minimum 150 for value
    white_upper[2] = 255

def detect_cups(frame):
    """Detect white/off-white cup tops using adaptive ellipse detection."""
    update_white_threshold(frame)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, white_lower, white_upper)
    
    # Apply morphological operations to clean up the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    cups = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Filter out small contours
            if len(contour) >= 5:  # Minimum points needed for ellipse fitting
                ellipse = cv2.fitEllipse(contour)
                (center, axes, angle) = ellipse
                major_axis = max(axes)
                minor_axis = min(axes)
                
                # More lenient aspect ratio for off-angle cups
                if 0.5 < (minor_axis/major_axis) < 1.5:
                    # Additional check for solidity (how convex the shape is)
                    hull = cv2.convexHull(contour)
                    hull_area = cv2.contourArea(hull)
                    if hull_area > 0:
                        solidity = float(area)/hull_area
                        if solidity > 0.8:  # Reject irregular shapes
                            cups.append(ellipse)
    
    # Sort cups by x-coordinate
    cups = sorted(cups, key=lambda x: x[0][0])
    return cups

def detect_ball(frame):
    """Detect ball position using color detection."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, ball_lower, ball_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
        if radius > 5:
            return (int(x), int(y))
    return None

def check_ball_under_cup(ball_pos, cup_ellipses):
    """Check if ball is under a cup based on position or last known state."""
    global ball_under_cup, last_nearest_cup, game_paused, missing_cup_index
    global last_moved_cup_idx, cup_has_ball_flag

    # Track the last moved cup before the ball is hidden
    if not hasattr(check_ball_under_cup, "last_ball_visible"):
        check_ball_under_cup.last_ball_visible = True
    if not hasattr(check_ball_under_cup, "last_moved_cup_before_hidden"):
        check_ball_under_cup.last_moved_cup_before_hidden = None

    if ball_pos is None:
        # When the ball just became hidden, store the last moved cup
        if check_ball_under_cup.last_ball_visible:
            check_ball_under_cup.last_moved_cup_before_hidden = last_moved_cup_idx
            check_ball_under_cup.last_ball_visible = False
        # Set flag based on whether a cup is considered to have the ball
        cup_has_ball_flag = check_ball_under_cup.last_moved_cup_before_hidden is not None
        return check_ball_under_cup.last_moved_cup_before_hidden
    else:
        check_ball_under_cup.last_ball_visible = True

    min_distance = float('inf')
    nearest_cup_idx = None
    for i, ellipse in enumerate(cup_ellipses):
        center = ellipse[0]
        distance = np.linalg.norm(np.array(ball_pos) - np.array(center))
        if distance < min_distance:
            min_distance = distance
            nearest_cup_idx = i

    last_nearest_cup = nearest_cup_idx

    if nearest_cup_idx is not None:
        major_axis = max(cup_ellipses[nearest_cup_idx][1]) / 2
        if min_distance < major_axis * 1.2:
            ball_under_cup = nearest_cup_idx
            cup_has_ball_flag = True
            return nearest_cup_idx

    cup_has_ball_flag = False
    return ball_under_cup

def check_cup_in_frame(box, frame_width, frame_height):
    """Check if a cup is within the frame boundaries."""
    x, y, w, h = box
    # More lenient check - allow partial visibility
    return (x + w/2 > 0 and y + h/2 > 0 and 
            x + w/2 < frame_width and y + h/2 < frame_height)

# Main loop
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame = cv2.resize(frame, (840, 680))
    frame_height, frame_width = frame.shape[:2]
    
    # Detect cups automatically on the first frame or if tracking fails
    if 'multi_tracker' not in globals() or not success:
        cups = detect_cups(frame)
        if len(cups) == 3:  # Now works with 2 or more cups
            multi_tracker = cv2.legacy.MultiTracker_create()
            for ellipse in cups:
                # Create a bounding box around the ellipse for tracking
                center, axes, angle = ellipse
                bbox = cv2.boundingRect(cv2.ellipse2Poly(
                    (int(center[0]), int(center[1])),
                    (int(axes[0]/2), int(axes[1]/2)),
                    int(angle), 0, 360, 1
                ))
                tracker = cv2.legacy.TrackerCSRT_create()
                multi_tracker.add(tracker, frame, bbox)
            print(f"{len(cups)} cups detected and trackers initialized!")
        else:
            print("Waiting for the 3 cups to be detected...")
            cv2.imshow("Shell Game", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue
    
    # Update cup positions
    success, boxes = multi_tracker.update(frame)
    
    # Check if any cup is out of frame
    if success and not game_paused:
        cups_in_frame = [check_cup_in_frame(box, frame_width, frame_height) for box in boxes]
        if not all(cups_in_frame):
            missing_cup_index = cups_in_frame.index(False)
            game_paused = True
            print(f"Game paused: Cup {missing_cup_index} is out of frame")
    
    # Check if all cups are back in frame
    if game_paused:
        cups_in_frame = [check_cup_in_frame(box, frame_width, frame_height) for box in boxes]
        if all(cups_in_frame):
            game_paused = False
            missing_cup_index = None
            print("Game resumed: All cups are back in frame")
    
    # Detect ball position
    ball_position = detect_ball(frame)
    
    # Draw ball if visible
    if ball_position:
        cv2.circle(frame, ball_position, 10, (0, 0, 255), 2)
        cv2.putText(frame, "Ball", (ball_position[0] - 20, ball_position[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    # Track cups and check ball location
    if success:
        cup_ellipses = []
        for i, (x, y, w, h) in enumerate(boxes):
            # Skip drawing if this is the missing cup during pause
            if game_paused and i == missing_cup_index:
                continue

            center = (int(x + w/2), int(y + h/2))
            axes = (int(w/2), int(h/2))
            angle = 0  # Approximate angle as we're using bounding box
            cup_ellipses.append((center, axes, angle))

            # --- Significant motion detection ---
            if last_cup_centers[i] is not None:
                dist = np.linalg.norm(np.array(center) - np.array(last_cup_centers[i]))
                if dist > motion_threshold:
                    last_motion_times[i] = time.time()
                    last_moved_cup_idx = i  # Track last moved cup
            last_cup_centers[i] = center

        # --- Draw all cups if game not ended ---
        if not game_ended:
            for i, (center, axes, angle) in enumerate(cup_ellipses):
                cv2.ellipse(frame, center, axes, angle, 0, 360, (0, 255, 0), 2)
                cv2.putText(frame, f"Cup {i}", (center[0] - 20, center[1] - max(axes) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # --- Real-time: Show which cup has the ball under it ---
        # Only if not paused and not ended
        if not game_paused and not game_ended:
            # Quantify cups as left, middle, right by x position
            cup_positions = []
            for i, (x, y, w, h) in enumerate(boxes):
                center_x = int(x + w/2)
                cup_positions.append((center_x, i))
            cup_positions_sorted = sorted(cup_positions, key=lambda tup: tup[0])
            cup_names = ["Left Cup", "Middle Cup", "Right Cup"]
            cup_idx_to_name = {}
            for idx, (_, i) in enumerate(cup_positions_sorted):
                cup_idx_to_name[i] = cup_names[idx]

            # Check which cup has the ball under it now
            current_ball_cup_idx = check_ball_under_cup(ball_position, cup_ellipses)
            if current_ball_cup_idx is not None:
                # Draw label above the cup
                x, y, w, h = [int(v) for v in boxes[current_ball_cup_idx]]
                center = (int(x + w/2), int(y + h/2))
                axes = (int(w/2), int(h/2))
                name = cup_idx_to_name.get(current_ball_cup_idx, f"Cup {current_ball_cup_idx}")
                cv2.putText(frame, f"{name} (Ball!)", (center[0] - 60, center[1] - max(axes) - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                # Optionally, highlight the cup
                cv2.ellipse(frame, center, axes, 0, 0, 360, (0, 255, 255), 3)

    # Determine which cup has the ball (only when game is not paused)
    if success and not game_paused:
        ball_under_cup_idx = check_ball_under_cup(ball_position, cup_ellipses)
        # Save the last valid cup index with the ball under it
        if ball_under_cup_idx is not None:
            last_ball_under_cup_idx = ball_under_cup_idx

    # --- End game if no cup has moved for 10 seconds ---
    if success and not game_paused and not game_ended:
        now = time.time()
        if all(now - t > motion_timeout for t in last_motion_times):
            game_ended = True
            print(f"Game ended: No cup has moved for {motion_timeout} seconds")
            last_wanted_cup_idx = last_moved_cup_idx

    # --- Show game ended message and highlight only the wanted cup ---
    if game_ended:
        # Quantify cups as left, middle, right by x position
        cup_positions = []
        for i, (x, y, w, h) in enumerate(boxes):
            center_x = int(x + w/2)
            cup_positions.append((center_x, i))
        cup_positions_sorted = sorted(cup_positions, key=lambda tup: tup[0])
        cup_names = ["Left Cup", "Middle Cup", "Right Cup"]
        cup_idx_to_name = {}
        for idx, (_, i) in enumerate(cup_positions_sorted):
            cup_idx_to_name[i] = cup_names[idx]

        idx_to_lr = {i: lr for lr, (_, i) in enumerate(cup_positions_sorted)}

        # If the ball is currently under a cup, use that cup as the detected one
        # Otherwise, use the last cup that had the ball under it before ending
        detected_cup_idx = None
        if ball_position is None:
            detected_cup_idx = last_ball_under_cup_idx
        else:
            # Ball is visible, check which cup (if any) it's under at end
            detected_cup_idx = check_ball_under_cup(ball_position, cup_ellipses)

        if detected_cup_idx is not None:
            wantedCup = idx_to_lr.get(detected_cup_idx, -1)
        else:
            wantedCup = -1
        print("wantedCup:", wantedCup)

        # Draw names above each cup (only after game end)
        for i, (x, y, w, h) in enumerate(boxes):
            center = (int(x + w/2), int(y + h/2))
            name = cup_idx_to_name.get(i, f"Cup {i}")
            cv2.putText(frame, name, (center[0] - 40, center[1] - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        # Only draw detection for the detected cup
        if detected_cup_idx is not None:
            i = detected_cup_idx
            x, y, w, h = [int(v) for v in boxes[i]]
            center = (int(x + w/2), int(y + h/2))
            axes = (int(w/2), int(h/2))
            angle = 0
            cv2.ellipse(frame, center, axes, angle, 0, 360, (0, 255, 0), 3)
            cv2.putText(frame, f"{cup_idx_to_name.get(i, f'Cup {i}')} (Ball)", (center[0] - 60, center[1] - max(axes) - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
          #  print(f"Ball is under {cup_idx_to_name.get(i, f'Cup {i}')}")
        cv2.putText(frame, "GAME ENDED: No cup motion", (100, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        cv2.imshow("Shell Game", frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        elif key & 0xFF == ord('r'):
            # Reset everything
            del multi_tracker
            last_motion_times = [time.time(), time.time(), time.time()]
            last_cup_centers = [None, None, None]
            game_ended = False
            last_wanted_cup_idx = None
            last_moved_cup_idx = None
            last_ball_under_cup_idx = None
            print("Reset: Re-detecting cups...")
        continue
    
    # Display game status and detection info
    if game_paused:
        cv2.putText(frame, "GAME PAUSED: Return all cups to frame", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, f"Cup {missing_cup_index} is missing", (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    # Display white detection threshold info
    cv2.putText(frame, f"White threshold: {int(white_lower[2])}-{white_upper[2]}", 
                (frame_width - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Print the flag value
   # print("cup_has_ball_flag:", cup_has_ball_flag)
    
    # Display frame
    cv2.imshow("Shell Game", frame)
    
    # Quit with 'q', reset with 'r'
    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break
    elif key & 0xFF == ord('r'):
        del multi_tracker
        print("Reset: Re-detecting cups...")

# Cleanup
cap.release()
cv2.destroyAllWindows()