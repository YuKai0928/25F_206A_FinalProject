import socket
import struct
import cv2
import numpy as np

HOST = "0.0.0.0"   # Listen on all interfaces
PORT = 5000

def recv_exact(sock, size):
    """Receive exactly `size` bytes from the socket."""
    data = b""
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet:
            raise ConnectionError("Socket connection closed unexpectedly")
        data += packet
    return data

def main():
    # Create TCP server socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)
        print(f"Listening on {HOST}:{PORT} ...")

        conn, addr = server.accept()
        with conn:
            print(f"Connected by {addr}")

            # 1) Read 4 bytes for the length header (big-endian unsigned int)
            header = recv_exact(conn, 4)
            (img_len,) = struct.unpack(">I", header)
            print(f"Expecting {img_len} bytes of image data")

            # 2) Read the JPEG bytes
            img_bytes = recv_exact(conn, img_len)

    # 3) Convert JPEG bytes to OpenCV image
    jpg_array = np.frombuffer(img_bytes, dtype=np.uint8)
    image = cv2.imdecode(jpg_array, cv2.IMREAD_COLOR)

    if image is None:
        print("Failed to decode image")
        return

    # 4) Example CV algo – convert to grayscale and detect edges
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 200)

    # 5) Show result
    cv2.imshow("TM5 Camera - Original", image)
    cv2.imshow("TM5 Camera - Edges", edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()