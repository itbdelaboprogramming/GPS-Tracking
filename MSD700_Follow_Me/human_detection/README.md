# Human detection

This folder contain all the code to detect human from the frame image of a camera. 

## How to Use

1. Download/Pull the code from GitHub
2. Go to GPS-Tracking/MSD700_Follow_Me/human_detection directory

If you run this in Jetson device or any device that have Python installed globally, then you can skip this step and jum to step 6

3. Install [virtual environment](https://python.land/virtual-environments/virtualenv) for Python
4. Activate the virtual environment
```powershell
venv\Scripts\Activate.ps1
```
5. Install [opencv-python](https://pypi.org/project/opencv-python/)

6. Go back to GPS-Tracking/MSD700_Follow_Me/human_detection directory
7. Run `follow_me.py` script
If succcess, the terminal will print the information about darknet model and the device camera used. A pop up will appear in the screen and show the frame from the camera.
If you want to use another camera and not your default one, try to run the script with `-c <device-id>` flag. 
```powershell
python follow_me.py -c 4
```
This will use camera with device id 4. For reference when using Intel Realsense camera in itbdelabof3 computer, the device id is 8 while using it in Jetson Xavier NX the device id is 4
