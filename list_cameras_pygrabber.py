from pygrabber.dshow_graph import FilterGraph

def list_cameras():
    devices = FilterGraph().get_input_devices()
    return devices

if __name__ == "__main__":
    cameras = list_cameras()
    if cameras:
        print("Available cameras:")
        for i, camera in enumerate(cameras):
            print(f"Camera {i}: {camera}")
    else:
        print("No cameras detected.") 