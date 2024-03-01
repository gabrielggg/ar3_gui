# ar3_gui

You will be able to test and move the joints in any direction you want, using the buttons DIR1 and DIR2. This project can be used with any 6 axis robot arm, you just have to replace the urdf directory files with the files of your robot (STL's and .urdf).

To run it:

git clone https://github.com/gabrielggg/ar3_gui.git

cd ar3_gui

pip install -r requirements.txt

python viz.py urdf/ar3.urdf

![ar3_gui](https://github.com/gabrielggg/ar3_gui/assets/5673338/1ebe9edc-b3b8-4fae-8d95-67fab76d2ef1)

This project is based on this urdf parser:
https://github.com/clemense/yourdfpy/

Thanks to https://github.com/ongdexter/ar3_core for providing the urdf files.

