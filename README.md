# FSAE-Acc.-Sim

Simple and lightwieght simulation of the 75m acceleration test at FSAE events.

Requires matplotlib, tkinter and optionally pyinstaller to build a standalone .exe

pip install pyinstaller

pyinstaller --onefile --add-data "black.tcl;." sim_gui.py