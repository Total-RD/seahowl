##########
User guide
##########


If SEAHOWL is used through its driver with input files, the following JSON files must be created:

* **main file:** main options for the simulation and links to turbine files
* **turbine file:** main options for the turbine and links to RNA, tower, and blade files
* **RNA file:** main properties of the RNA
* **tower file:** main properties of the tower
* **blade file:** main properties of the blade (and links to airfoil files in using built-in BEMT)

Each file name can be arbitrarily name since and all filepaths are user-defined and each filepath defined within a file should be relative to that file.
The only requirement are that the files are valid JSON and that all expected keys and values are defined.

***********
Input files
***********

Main input file
===============

The main input file (or *command* file) of SEAHOWL is a JSON file, which can be named arbitrarily.
In this file, the following is set: global numerical options, output options, environmental conditions, and the list of turbines with their positions and the filepath of their own JSON files.

.. literalinclude:: ../../../data/IEA15MW/main.json
   :language: json
   :linenos:
   :caption: Main input file example


Turbine input file
==================

The turbine input file contains information the rotor, the RNA, the tower, and the controller.
Some of the options of each component are set in this file (e.g. blade pitch and precone, elasto and aero discretization of blades and towers), while more detailed options of each components are described in other files that are referred here (such as for blades and tower material properties).

Rotor "type" can be "fea", "rigid", or "disk".
Controller "type" can be "DISCON" or "RPM".

.. literalinclude:: ../../../data/IEA15MW/turbine.json
   :language: json
   :linenos:
   :caption: Turbine input file example


RNA input file
==============

The RNA input file contains properties of the hub, the shaft, the nacelle, and the drivetrain.

.. literalinclude:: ../../../data/IEA15MW/rna.json
   :language: json
   :linenos:
   :caption: Rotor-Nacelle Assembly input file example


Tower input file
================

The tower input file contains the main properties of the tower such as its **height** (tower top), the **height of its base** (tower bottom), and **damping coefficients**. The rest of the properties are defined through reference points that are defined along the tower according to their "**fraction**", ranging from 0 (tower base) to 1 (tower top).
Each reference point is defined with with:

* elasto properties: the **lineic density**, the **fore-aft stiffness**, and the **side-side stiffness**.
* aero properties: the **diameter**, and the **drag coefficient**.

.. literalinclude:: ../../../data/IEA15MW/tower.json
   :language: json
   :linenos:
   :lines: -24
   :caption: Tower input file example (truncated)

Blade input file
================

The blade input file contains the main properties of the blade through reference points that are defined along the blade according to their "**fraction**", ranging from 0 (blade root) to 1 (blade tip). The global **damping coefficients** of the blade are also defined in this file.
Each reference point is defined with its **coordinates** (in IEC standard) along with:

* elasto properties: the **twist** along the main axis, the 6x6 **mass matrix**, and the 6x6 **stiffness matrix**.
* aero properties: the **airfoil filepath** (only used if the selected aerodynamic model is the built-in BEMT), the **chord length**, and the **aerodynamic offset** (if any).

.. literalinclude:: ../../../data/IEA15MW/blade.json
   :language: json
   :linenos:
   :lines: -51
   :caption: Blade input file example (truncated)

Airfoil input file
==================

.. literalinclude:: ../../../data/IEA15MW/airfoils/IEA-15-240-RWT_AeroDyn15_Polar_37.json
   :language: json
   :linenos:
   :lines: -15
   :caption: Airfoil input file example (truncated)
