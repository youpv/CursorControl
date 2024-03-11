import cv2
import time
import pyautogui
import pykinect_azure as pykinect
import ctypes
from pykinect_azure.k4a._k4atypes import k4a_float3_t


pyautogui.FAILSAFE = False  # Disable the fail-safe feature

def map_hand_to_screen(hand_coords, depth_image_shape):
    # Normalize the hand coordinates based on the depth image dimensions and convert to screen coordinates
    screen_width, screen_height = pyautogui.size()
    depth_width, depth_height = depth_image_shape
    
    # Assuming hand_coords are the x, y values from the depth space 2D position, you will need to scale this correctly
    screen_x = (hand_coords[0] / depth_width) * screen_width
    screen_y = (hand_coords[1] / depth_height) * screen_height
    # Depending on the coordinate system of hand_coords, you might need to adjust y-coordinate as (1 - hand_coords[1])
    
    return screen_x, screen_y

def map_joints_to_color_space(joint, calibration):
    # Assuming joint.position is already an instance of _xyz, you can directly use its x, y, and z values
    source_point3d = k4a_float3_t((joint.position.x, joint.position.y, joint.position.z))

    source_camera = pykinect.K4A_CALIBRATION_TYPE_DEPTH
    target_camera = pykinect.K4A_CALIBRATION_TYPE_COLOR

    try:
        # Call the convert_3d_to_2d method with the correct type of argument.
        target_point2d = calibration.convert_3d_to_2d(
            source_point3d,
            source_camera,
            target_camera
        )
        return target_point2d
    except Exception as e:
        # Log any exceptions that occur during the conversion.
        print(f"Error converting 3D to 2D: {e}")
        return None

def is_cursor_over_button(cursor_position, button_position, button_size):
    # Check if cursor is within the button bounds
    x, y = cursor_position
    bx, by, bw, bh = button_position + button_size
    return bx <= x <= bx + bw and by <= y <= by + bh

# Function that returns the current hover state and updates the state if necessary
def process_hover_state(hovering, hover_start_time, cursor_position, button_position, button_size, hover_duration):
    if is_cursor_over_button(cursor_position, button_position, button_size):
        if not hovering:
            hover_start_time = time.time()
            hovering = True
        elif time.time() - hover_start_time >= hover_duration:
            pyautogui.click()
            hovering = False
            hover_start_time = None
    else:
        hovering = False
        hover_start_time = None
    return hovering, hover_start_time

if __name__ == "__main__":
    pykinect.initialize_libraries(track_body=True)

    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

    device = pykinect.start_device(config=device_config)
    body_tracker = pykinect.start_body_tracker()
    calibration = device.get_calibration(device_config.depth_mode, device_config.color_resolution)

    hover_start_time = None
    hovering = False
    hover_duration = 2.0  # 2 seconds to trigger click

    button_position = (100, 100)  # Button top-left corner position (x, y)
    button_size = (50, 50)  # Button size (width, height)



    while True:
        capture = device.update()
        body_frame = body_tracker.update()
        # Obtain screen width and height using pyautogui
        screen_size = pyautogui.size()
        screen_x, screen_y = screen_size.width, screen_size.height
    
        # Check if there are any bodies in the current frame
        num_bodies = body_frame.get_num_bodies()
        if num_bodies > 0:
            for body_id in range(num_bodies):
                body = body_frame.get_body(body_id)
                
                if body.is_valid():
                    right_hand_joint = body.joints[pykinect.K4ABT_JOINT_HAND_RIGHT]
                    right_hand_point_2d = map_joints_to_color_space(right_hand_joint, calibration)
                    if right_hand_point_2d:
                      mapped_x = right_hand_point_2d.xy.x * screen_x / calibration._handle.color_camera_calibration.resolution_width
                      mapped_y = right_hand_point_2d.xy.y * screen_y / calibration._handle.color_camera_calibration.resolution_height
                      pyautogui.moveTo(mapped_x, mapped_y)

        # Draw the skeleton onto the color image if there's an available color frame
        ret_color, color_image = capture.get_color_image()
        if ret_color:
            color_image_with_skeleton = body_frame.draw_bodies(color_image, pykinect.K4A_CALIBRATION_TYPE_COLOR)
            cv2.imshow('Color image with skeleton', color_image_with_skeleton)

        # Press 'q' to stop
        if cv2.waitKey(1) == ord('q'):
            break

    device.close()
    cv2.destroyAllWindows()