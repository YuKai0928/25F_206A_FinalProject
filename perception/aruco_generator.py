import cv2
import numpy as np
import matplotlib.pyplot as plt

def generate_aruco_marker(marker_id=0,
                          dictionary_name=cv2.aruco.DICT_4X4_50,
                          marker_size=300,
                          save_path=None):
    """
    Generate and optionally save a single ArUco marker image.

    Args:
        marker_id (int): ID of the marker within the chosen dictionary.
        dictionary_name: OpenCV ArUco dictionary (e.g. cv2.aruco.DICT_4X4_50).
        marker_size (int): Output image size in pixels (marker_size x marker_size).
        save_path (str or None): If given, saves the marker image to this path.

    Returns:
        marker_img (numpy.ndarray): Generated marker as a grayscale image.
    """
    # Get the predefined dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_name)

    # Create a blank image then draw the marker on it
    marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8)
    marker_img = cv2.aruco.generateImageMarker(
        aruco_dict, marker_id, marker_size, marker_img, 1
    )

    # Optionally save the file
    if save_path is not None:
        cv2.imwrite(save_path, marker_img)
        print(f"Saved ArUco marker (ID={marker_id}) to: {save_path}")

    return marker_img

def plot_marker(marker_img, title="ArUco Marker"):
    """
    Display the ArUco marker using matplotlib.
    """
    plt.figure()
    plt.imshow(marker_img, cmap='gray', vmin=0, vmax=255)
    plt.axis('off')
    plt.title(title)
    plt.show()


if __name__ == "__main__":
    for marker_id in range(10):
        marker_size = 400  # pixels
        save_path = f"aruco_id_{marker_id}_4x4_50.png"

        marker_img = generate_aruco_marker(
            marker_id=marker_id,
            dictionary_name=cv2.aruco.DICT_4X4_50,
            marker_size=marker_size,
            save_path=save_path
        )
        plot_marker(marker_img, title=f"ArUco 4x4_50, ID={marker_id}")