import cv2
import time
import pyautogui
import pykinect_azure as pykinect
from pykinect_azure.k4a._k4atypes import k4a_float3_t
import threading
from pykinect_azure.k4a.configuration import Configuration
import numpy as np

pyautogui.FAILSAFE = False

screen_width, screen_height = pyautogui.size()
screen_size = pyautogui.size()

SWIPE_SPEED_THRESHOLD = 1000
SWIPE_COOLDOWN = 1.3
CLICK_COOLDOWN = 1.0
SPACE_COOLDOWN = 5.0
kinect_field_of_view_ratio = 1
last_swipe_time = 0
last_click_time = 0
last_space_time = 0

click_lock = threading.Lock()

SMOOTHING_FACTOR = 0.7
MOVEMENT_THRESHOLD = 10

right_wrist_history = []


def perform_click():
    global last_click_time
    with click_lock:
        current_time = time.time()
        if current_time - last_click_time > CLICK_COOLDOWN:
            pyautogui.click()
            last_click_time = current_time


def map_hand_to_screen(hand_coords, depth_image_shape):
    depth_width, depth_height = depth_image_shape

    screen_x = screen_width - (hand_coords[0] / depth_width) * screen_width
    screen_y = (hand_coords[1] / depth_height) * screen_height

    return screen_x, screen_y


def update_cursor_position(mapped_x, mapped_y, previous_position):
    global smoothed_position, last_move_time
    if previous_position is None:
        smoothed_position = (mapped_x, mapped_y)
        last_move_time = time.time()
    else:
        smoothed_x = previous_position[0] + SMOOTHING_FACTOR * (
            mapped_x - previous_position[0]
        )
        smoothed_y = previous_position[1] + SMOOTHING_FACTOR * (
            mapped_y - previous_position[1]
        )
        smoothed_position = (smoothed_x, smoothed_y)
        if (
            np.sqrt(
                (smoothed_position[0] - previous_position[0]) ** 2
                + (smoothed_position[1] - previous_position[1]) ** 2
            )
            > MOVEMENT_THRESHOLD
        ):
            last_move_time = time.time()
        elif time.time() - last_move_time >= 1.5:
            perform_click()
            last_move_time = time.time()
    pyautogui.moveTo(int(smoothed_position[0]), int(smoothed_position[1]))
    return smoothed_position


def map_joints_to_color_space(joint, calibration):
    # Assuming joint.position is already an instance of _xyz, you can directly use its x, y, and z values
    source_point3d = k4a_float3_t(
        (joint.position.x, joint.position.y, joint.position.z)
    )

    source_camera = pykinect.K4A_CALIBRATION_TYPE_DEPTH
    target_camera = pykinect.K4A_CALIBRATION_TYPE_COLOR

    try:
        # Call the convert_3d_to_2d method with the correct type of argument.
        target_point2d = calibration.convert_3d_to_2d(
            source_point3d, source_camera, target_camera
        )

        # Adjust the mapping to use a smaller portion of the Kinect's field of view
        target_point2d.xy.x = target_point2d.xy.x / kinect_field_of_view_ratio
        target_point2d.xy.y = target_point2d.xy.y / kinect_field_of_view_ratio

        # Ensure the mapped coordinates are within the screen bounds
        target_point2d.xy.x = min(max(target_point2d.xy.x, 0), screen_width)
        target_point2d.xy.y = min(max(target_point2d.xy.y, 0), screen_height)

        return target_point2d
    except Exception as e:
        # Log any exceptions that occur during the conversion.
        print(f"Error converting 3D to 2D: {e}")
        return None


def is_cursor_over_button(cursor_position, button_position, button_size):
    x, y = cursor_position
    bx, by, bw, bh = button_position + button_size
    if bx <= x <= bx + bw and by <= y <= by + bh:
        return True
    else:
        return False


def process_hover_state(
    hovering,
    hover_start_time,
    cursor_position,
    button_position,
    button_size,
    hover_duration,
):
    if is_cursor_over_button(cursor_position, button_position, button_size):
        if not hovering:
            hover_start_time = time.time()
            hovering = True
        elif time.time() - hover_start_time >= hover_duration:
            perform_click()
            hovering = False
            hover_start_time = None
    else:
        hovering = False
        hover_start_time = None
    return hovering, hover_start_time


if __name__ == "__main__":
    pykinect.initialize_libraries(track_body=True)
    last_move_time = None

    device_config = Configuration()
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
    device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED
    device_config.camera_fps = pykinect.K4A_FRAMES_PER_SECOND_30
    device_config.synchronized_images_only = True

    device = pykinect.start_device(config=device_config)
    body_tracker = pykinect.start_body_tracker()
    calibration = device.get_calibration(
        device_config.depth_mode, device_config.color_resolution
    )

    # Obtain screen and camera resolution once, outside the loop
    # screen_size = pyautogui.size()
    screen_x, screen_y = screen_size.width, screen_size.height
    color_camera_resolution_width = (
        calibration._handle.color_camera_calibration.resolution_width
    )
    color_camera_resolution_height = (
        calibration._handle.color_camera_calibration.resolution_height
    )

    hover_start_time = None
    hovering = False
    hover_duration = 2.0  # 2 seconds to trigger click
    previous_position = None  # Stores previous cursor position
    cursor_update_threshold = 5  # Minimum distance to update cursor position

    button_position = (100, 100)  # Button top-left corner position (x, y)
    button_size = (50, 50)  # Button size (width, height)

    while True:
        capture = device.update()
        body_frame = body_tracker.update()

        # Check if there are any bodies in the current frame
        num_bodies = body_frame.get_num_bodies()
        if num_bodies > 0:
            for body_id in range(num_bodies):
                body = body_frame.get_body(body_id)

                if body.is_valid():
                    right_hand_joint = body.joints[pykinect.K4ABT_JOINT_WRIST_RIGHT]
                    left_hand_joint = body.joints[pykinect.K4ABT_JOINT_WRIST_LEFT]
                    head_joint = body.joints[pykinect.K4ABT_JOINT_HEAD]
                    right_hand_point_2d = map_joints_to_color_space(
                        right_hand_joint, calibration
                    )
                    left_hand_point_2d = map_joints_to_color_space(
                        left_hand_joint, calibration
                    )
                    head_point_2d = map_joints_to_color_space(head_joint, calibration)

                    if right_hand_point_2d:
                        mapped_x = screen_width - (
                            right_hand_point_2d.xy.x
                            * screen_x
                            / color_camera_resolution_width
                        )
                        mapped_y = (
                            right_hand_point_2d.xy.y
                            * screen_y
                            / color_camera_resolution_height
                        )
                        current_position = (mapped_x, mapped_y)

                        right_wrist_history.append((current_position, time.time()))

                        if len(right_wrist_history) >= 5:
                            dx = (
                                right_wrist_history[-1][0][0]
                                - right_wrist_history[0][0][0]
                            )
                            dy = (
                                right_wrist_history[-1][0][1]
                                - right_wrist_history[0][0][1]
                            )
                            dt = right_wrist_history[-1][1] - right_wrist_history[0][1]
                            speed = dx / dt
                            vertical_speed = dy / dt
                            current_time = time.time()
                            if current_time - last_swipe_time >= SWIPE_COOLDOWN:
                                if speed > SWIPE_SPEED_THRESHOLD:
                                    print("Swipe right detected")
                                    # press right arrow key
                                    pyautogui.press("left")
                                    last_swipe_time = current_time
                                elif speed < -SWIPE_SPEED_THRESHOLD:
                                    print("Swipe left detected")
                                    # press left arrow key
                                    pyautogui.press("right")
                                    last_swipe_time = current_time
                                elif vertical_speed > SWIPE_SPEED_THRESHOLD:
                                    print("Swipe down detected")
                                    # press down arrow key
                                    pyautogui.press("down")
                                    last_swipe_time = current_time
                                elif vertical_speed < -SWIPE_SPEED_THRESHOLD:
                                    print("Swipe up detected")
                                    # press up arrow key
                                    pyautogui.press("up")
                                    last_swipe_time = current_time
                            right_wrist_history = []

                        # Create a thread to update cursor position asynchronously
                        cursor_update_thread = threading.Thread(
                            target=update_cursor_position,
                            args=(mapped_x, mapped_y, previous_position),
                        )
                        cursor_update_thread.start()

                        previous_position = current_position

                    if left_hand_point_2d and head_point_2d:
                        mapped_y_left = (
                            left_hand_point_2d.xy.y
                            * screen_y
                            / color_camera_resolution_height
                        )
                        mapped_y_head = (
                            head_point_2d.xy.y
                            * screen_y
                            / color_camera_resolution_height
                        )
                        space_current_time = time.time()
                        # if both left wrist and right wrist are above head, perform spacebar press
                        if (
                            mapped_y_left < mapped_y_head
                            and mapped_y < mapped_y_head
                            and space_current_time - last_space_time >= SPACE_COOLDOWN
                        ):
                            pyautogui.press("space")
                            print("Spacebar pressed")
                            last_space_time = space_current_time

        # Only process hovering state if previous_position is not None
        if previous_position is not None:
            hovering, hover_start_time = process_hover_state(
                hovering,
                hover_start_time,
                previous_position,
                button_position,
                button_size,
                hover_duration,
            )

        # Draw the skeleton onto the color image if there's an available color frame
        ret_color, color_image = capture.get_color_image()
        if ret_color:
            color_image_with_skeleton = body_frame.draw_bodies(
                color_image, pykinect.K4A_CALIBRATION_TYPE_COLOR
            )
            cv2.imshow("Color image with skeleton", color_image_with_skeleton)

        # Press 'q' to stop
        if cv2.waitKey(1) == ord("q"):
            break

    device.close()
    cv2.destroyAllWindows()
