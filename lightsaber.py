import cv2
import pykinect_azure as pykinect
from pykinect_azure.k4a import _k4atypes


def draw_lightsaber(
    image, joint_start_pos, joint_end_pos, color=(0, 0, 255), thickness=30
):
    """
    Draws a lightsaber on the given image between two joint positions.

    :param image: Input image.
    :param joint_start_pos: Start position (x, y) of the lightsaber as a tuple.
    :param joint_end_pos: End position (x, y) of the lightsaber as a tuple.
    :param color: Color of the lightsaber as a BGR tuple (default is blue).
    :param thickness: Thickness of the lightsaber line.
    """
    cv2.line(image, joint_start_pos, joint_end_pos, color, thickness)


import numpy as np


def map_joints_to_color_space(joint, calibration):
    source_point3d = _k4atypes.k4a_float3_t(
        (joint.position.x, joint.position.y, joint.position.z)
    )
    source_camera = pykinect.K4A_CALIBRATION_TYPE_DEPTH
    target_camera = pykinect.K4A_CALIBRATION_TYPE_COLOR

    try:
        target_point2d = calibration.convert_3d_to_2d(
            source_point3d, source_camera, target_camera
        )
        return int(target_point2d.xy.x), int(target_point2d.xy.y)
    except Exception as e:
        print(f"Error converting 3D to 2D: {e}")
        return None


def calculate_lightsaber_end(
    hand_pos, forearm_pos, arm_orientation, length=500, offset_length=200
):
    """
    Calculates a more hand-like endpoint for the lightsaber, taking into account the orientation of the forearm.

    :param hand_pos: The position (x, y) of the hand joint.
    :param forearm_pos: The position (x, y) of the forearm joint (e.g., elbow).
    :param arm_orientation: The orientation of the arm required to adjust the offset direction.
    :param length: Length of the lightsaber extending from the hand joint.
    :param offset_length: Offset distance from the hand to simulate grip.
    :return: New endpoint (x, y) of the lightsaber.
    """
    # Convert to numpy arrays
    hand_vec = np.array(hand_pos)
    forearm_vec = np.array(forearm_pos)

    # Calculate direction vector from forearm to hand
    direction = hand_vec - forearm_vec
    direction_normalized = direction / np.linalg.norm(direction)

    # Rotate the direction by 90 degrees to simulate holding the lightsaber.
    # Rotation direction changes with arm orientation (left arm vs. right arm).
    if arm_orientation == "right":
        # Rotate clockwise
        rotate_matrix = np.array([[0, 1], [-1, 0]])
    else:
        # Rotate counterclockwise
        rotate_matrix = np.array([[0, -1], [1, 0]])

    rotated_direction = np.dot(rotate_matrix, direction_normalized)

    # Calculate the new end point as offset plus extension of the lightsaber
    offset_point = hand_vec + rotated_direction * offset_length
    new_end_point = offset_point + rotated_direction * length

    return tuple(offset_point.astype(int)), tuple(new_end_point.astype(int))


if __name__ == "__main__":
    pykinect.initialize_libraries(track_body=True)

    device_config = pykinect.default_configuration
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1080P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

    device = pykinect.start_device(config=device_config)
    body_tracker = pykinect.start_body_tracker()
    calibration = device.get_calibration(
        device_config.depth_mode, device_config.color_resolution
    )

    while True:
        capture = device.update()
        body_frame = body_tracker.update()
        ret_color, color_image = capture.get_color_image()

        if ret_color:
            num_bodies = body_frame.get_num_bodies()

            if num_bodies > 0:
                for body_id in range(num_bodies):
                    body = body_frame.get_body(body_id)

                    if body.is_valid():
                        right_wrist_joint = body.joints[
                            pykinect.K4ABT_JOINT_WRIST_RIGHT
                        ]
                        right_elbow_joint = body.joints[
                            pykinect.K4ABT_JOINT_ELBOW_RIGHT
                        ]

                        wrist_pos = map_joints_to_color_space(
                            right_wrist_joint, calibration
                        )
                        elbow_pos = map_joints_to_color_space(
                            right_elbow_joint, calibration
                        )

                        # Assuming wrist_pos as the hand position, and elbow_pos as the forearm position
                        if wrist_pos and elbow_pos:
                            # Determine the orientation of the arm (left or right arm)
                            # For this example, let's keep using the right arm.
                            arm_orientation = (
                                "right"  # Change to 'left' if using the left arm
                            )

                            handle_start_pos, lightsaber_end_pos = (
                                calculate_lightsaber_end(
                                    wrist_pos,
                                    elbow_pos,
                                    arm_orientation
                                )
                            )

                            # Drawing the lightsaber handle (shorter, and potentially darker or a different color)
                            draw_lightsaber(
                                color_image,
                                wrist_pos,
                                handle_start_pos,
                            )

                            # Drawing the lightsaber blade
                            draw_lightsaber(
                                color_image,
                                handle_start_pos,
                                lightsaber_end_pos,
                                color=(0, 255, 0),
                                thickness=6,
                            )

            cv2.imshow("Color image with lightsaber", color_image)

        if cv2.waitKey(1) == ord("q"):
            break

    device.close()
    cv2.destroyAllWindows()
