# Mapping

Here you can find:
- CVS file (zipped) with sensor and position data - from a high definition XYZ:0.2x0.5x0.5mm run mapping an area of 90x64x23cm
- A PDF explaining the format of the CVS file
- Various Jupyter Notebooks for preparing and processing the mapping data
- A PDF showing the rig and sensor placements, origo and coordinate setup
- PDFs with section views generated from this data. Both in the regular Viridis colorscale, and in a custom "Zebra" colorscale alternating between black and white with 1k of sensor value spacing per color, i.e. 2k value difference before the same gradient repeats
- A JPG and a PNG showing the intended format of the section views/colorscale (we observed differences in how two PCs rendered the Zebra colorscale/legend on the right of the pages)

Example of early mapping logic:
(Note: movements have since been vastly improved with continous movement on the x-axis to almost completely remove oscillations)

https://youtu.be/Qj2UO2YyPS8

Example of autonomous homing/docking:
(Note: Extremely basic/crude movement logic but 100% success rate)

https://youtu.be/gWZDnJYmrWg

Example of manualhoming/docking:

https://youtu.be/5CPR1RUVa-0
