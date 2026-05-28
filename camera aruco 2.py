import cv2
import cv2.aruco as aruco
import numpy as np
import os

def detect_markers_in_tile(img_tile, offset_x, offset_y, aruco_dict, parameters):
    """
    Detects ArUco markers within an image segment and translates their local coordinates back to the global image scale.
    Libraries used: cv2, cv2.aruco, numpy (np)
    """
    # Convert the specific tile to grayscale for detection
    gray = cv2.cvtColor(img_tile, cv2.COLOR_BGR2GRAY)
    
    # Initialize the detector and find markers in this tile
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(gray)

    global_corners = []
    found_ids = []

    # If markers are found, shift their local tile coordinates to match the full frame
    if ids is not None:
        for i in range(len(ids)):
            c = corners[i][0]
            c[:, 0] += offset_x  # Add horizontal offset
            c[:, 1] += offset_y  # Add vertical offset
            
            global_corners.append(np.array([c]))
            found_ids.append(ids[i][0])
            
    return global_corners, found_ids

def calibrate_camera_live(cap, aruco_dict):
    """
    Captures live frames of a ChArUco board to compute and save the camera's intrinsic calibration matrix and distortion coefficients.
    Libraries used: cv2, cv2.aruco, numpy (np)
    """
    # Define the ChArUco board layout (9x7 squares, specific marker sizes)
    board = aruco.CharucoBoard((9, 7), 0.019, 0.014, aruco_dict)
    
    # Apply aggressive tuning parameters for the calibration detector
    detector_params = aruco.DetectorParameters()
    detector_params.minMarkerPerimeterRate = 0.01 
    detector_params.adaptiveThreshWinSizeMin = 3
    detector_params.adaptiveThreshWinSizeMax = 30
    detector_params.adaptiveThreshWinSizeStep = 2
    
    charuco_detector = aruco.CharucoDetector(board, charucoParams=None, detectorParams=detector_params)

    all_charuco_corners = []
    all_charuco_ids = []
    image_size = None

    # Print console instructions for the user
    print("\n=== PHASE 0: CAMERA CALIBRATION ===")
    print("Hold the 7x9 ChArUco board in front of the camera.")
    print("Press 'c' to capture a calibration frame (do this at different angles/distances).")
    print("Press 'd' when done (at least 10 frames recommended).")
    print("=====================================\n")

    # Live capture loop for calibration frames
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        if image_size is None:
            image_size = (frame.shape[1], frame.shape[0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display_frame = frame.copy()

        # Detect the ChArUco board in the current frame
        charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)
        
        # Draw detected corners on the live feed for visual feedback
        if charuco_ids is not None and len(charuco_ids) > 0:
            aruco.drawDetectedCornersCharuco(display_frame, charuco_corners, charuco_ids)

        # Overlay UI instructions
        cv2.putText(display_frame, f"Frames Captured: {len(all_charuco_corners)}", 
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_frame, "'c': Capture | 'd': Done calibrating", 
                    (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("Calibration Phase", display_frame)

        key = cv2.waitKey(1) & 0xFF
        
        # Handle user input: capture frame, finish calibration, or quit
        if key == ord('c'):
            if charuco_ids is not None and len(charuco_ids) > 10:
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                print(f"Captured frame! Total frames: {len(all_charuco_corners)}")
            else:
                print("Not enough markers visible to capture a reliable frame.")
        elif key == ord('d'):
            if len(all_charuco_corners) >= 5:
                print("Calculating calibration parameters... please wait.")
                break
            else:
                print("Error: Need at least 5 frames to calibrate reliably. Keep capturing.")
        elif key == ord('q'):
            print("Calibration aborted.")
            cv2.destroyWindow("Calibration Phase")
            return None, None

    cv2.destroyWindow("Calibration Phase")
    
    # Extract object points and image points for the calibration calculation
    obj_points = []
    img_points = []
    for i in range(len(all_charuco_corners)):
        objp, imgp = board.matchImagePoints(all_charuco_corners[i], all_charuco_ids[i])
        obj_points.append(objp)
        img_points.append(imgp)

    # Calculate Camera Matrix and Distortion Coefficients
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)
    
    if ret:
        print(f"Calibration successful! RMS Error: {ret:.4f}")
        # Save matrices so calibration isn't required on subsequent runs
        np.savez("camera_matrix.npz", mtx=mtx, dist=dist)
        print("Saved to 'camera_matrix.npz'.")
        return mtx, dist
    else:
        print("Calibration failed.")
        return None, None

def main():
    """
    Manages the webcam feed to load camera calibration, detect team colors, track markers using overlapping tiles, and render 3D pose estimations.
    Libraries used: cv2, cv2.aruco, numpy (np), os
    """
    # Open the designated camera (index 1)
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Set up the ArUco dictionary and detection parameters
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
    parameters = aruco.DetectorParameters()
    parameters.minMarkerPerimeterRate = 0.01  
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 30
    parameters.adaptiveThreshWinSizeStep = 2
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

    # --- PHASE 0: LOAD OR PERFORM CALIBRATION ---
    # Check if a calibration file already exists to save time
    if os.path.exists("camera_matrix.npz"):
        print("Loading existing camera calibration...")
        data = np.load("camera_matrix.npz")
        mtx = data['mtx']
        dist = data['dist']
    else:
        mtx, dist = calibrate_camera_live(cap, aruco_dict)
        if mtx is None:
            return # Exit if calibration failed or aborted

    # Configure the 3D points for the game piece markers (Used for solvePnP)
    # IMPORTANT: Change this to the physical size of the ArUco markers on your crates/quadrants in meters.
    MARKER_SIZE_METERS = 0.05 
    
    # Define the 4 corners of the marker in 3D space
    marker_3d_points = np.array([
        [-MARKER_SIZE_METERS / 2,  MARKER_SIZE_METERS / 2, 0],
        [ MARKER_SIZE_METERS / 2,  MARKER_SIZE_METERS / 2, 0],
        [ MARKER_SIZE_METERS / 2, -MARKER_SIZE_METERS / 2, 0],
        [-MARKER_SIZE_METERS / 2, -MARKER_SIZE_METERS / 2, 0]
    ], dtype=np.float32)

    print("Starting live feed... Press 'q' to quit.")

    # --- SLIDING WINDOW CONFIGURATION ---
    # Divide the frame into a 2x3 grid with overlapping boundaries
    rows = 2
    cols = 3
    overlap = 50 

    # --- ITEM CONFIGURATION ---
    # Map specific ArUco IDs to physical game items and team colors
    item_map = { 
        36: {"color": "Blue", "type": "crate"},
        47: {"color": "Yellow", "type": "crate"},
        20: {"color": "Yellow", "type": "nest quadrant"},
        21: {"color": "Blue", "type": "nest quadrant"},
        22: {"color": "Yellow", "type": "cursor quadrant"},
        23: {"color": "Blue", "type": "cursor quadrant"}
    }

    team_color = None 

    # Main detection loop
    while True:
        ret, frame = cap.read()
        if not ret: break
        
        h, w = frame.shape[:2]
        tile_h = int(h / rows)
        tile_w = int(w / cols)
        unique_markers = {}

        # Scan the image using the sliding window approach to catch small markers
        for r in range(rows):
            for c in range(cols):
                # Calculate tile boundaries including the overlap
                y1 = max(0, r * tile_h - overlap)
                y2 = min(h, (r + 1) * tile_h + overlap)
                x1 = max(0, c * tile_w - overlap)
                x2 = min(w, (c + 1) * tile_w + overlap)

                tile = frame[y1:y2, x1:x2]
                
                # Detect markers inside the current tile
                corners, ids = detect_markers_in_tile(tile, x1, y1, aruco_dict, parameters)
                
                # Prevent duplicate detections caused by the overlap
                for i, marker_id in enumerate(ids):
                    if marker_id not in unique_markers:
                        unique_markers[marker_id] = corners[i]

        # ==========================================
        # PHASE 1: TEAM RECOGNITION
        # ==========================================
        # Wait until a mapped marker is shown to lock in the player's team color
        if team_color is None:
            cv2.putText(frame, "RECOGNITION PHASE: Show a team marker...", 
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            for m_id in unique_markers.keys():
                if m_id in item_map:
                    team_color = item_map[m_id]["color"]
                    print(f"Team Locked: {team_color} (Triggered by ID {m_id})")
                    break 

        # ==========================================
        # PHASE 2: TRACKING AND 3D POSE ESTIMATION
        # ==========================================
        else:
            # Change UI color depending on the locked team
            ui_color = (255, 0, 0) if team_color == "Blue" else (0, 255, 255)
            cv2.putText(frame, f"TEAM LOCKED: {team_color.upper()}", 
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, ui_color, 2)

            if unique_markers:
                # Prepare data structures for rendering and pose calculation
                final_ids = np.array([[k] for k in unique_markers.keys()], dtype=np.int32)
                final_corners = list(unique_markers.values())

                aruco.drawDetectedMarkers(frame, final_corners, final_ids)
                
                for i in range(len(final_ids)):
                    m_id = final_ids[i][0]
                    c = final_corners[i][0]
                    
                    # 1. Pixel Center Calculation for UI placement
                    cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))

                    # 2. 3D Pose Estimation using solvePnP
                    success, rvec, tvec = cv2.solvePnP(
                        marker_3d_points, c, mtx, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    # Determine physical position (Z is depth/distance from camera, X is left/right, Y is up/down)
                    if success:
                        cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, MARKER_SIZE_METERS)
                        x_m, y_m, z_m = tvec[0][0], tvec[1][0], tvec[2][0]
                    else:
                        x_m, y_m, z_m = 0.0, 0.0, 0.0

                    # Determine if the detected object belongs to the locked team
                    if m_id in item_map:
                        item = item_map[m_id]
                        relationship = "Friend" if item["color"] == team_color else "Foe"
                        label = f"{relationship} {item['type']}"
                        text_color = (0, 255, 0) if relationship == "Friend" else (0, 0, 255)
                    else:
                        label = "Other"
                        text_color = (255, 255, 255)

                    # Construct and draw the on-screen labels and coordinate data
                    text = f"{label} ID:{m_id}"
                    pose_text = f"Pos: [X:{x_m:.2f}m, Y:{y_m:.2f}m, Z:{z_m:.2f}m]"
                    
                    cv2.circle(frame, (cx, cy), 4, text_color, -1)
                    cv2.putText(frame, text, (cx - 20, cy - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
                    cv2.putText(frame, pose_text, (cx - 20, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Show the processed frame to the user
        cv2.imshow("Live Tiled Detection & 3D Pose", frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up hardware resources and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()