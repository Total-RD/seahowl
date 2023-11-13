###########
Input files
###########


If SEAHOWL is used through its driver with input files, the following JSON files must be created:

* **main file:** main options for the simulation and links to turbine files
* **turbine file:** main options for the turbine and links to RNA, tower, and blade files
* **RNA file:** main properties of the RNA
* **tower file:** main properties of the tower
* **blade file:** main properties of the blade (and links to airfoil files in using built-in BEMT)

Each file name can be arbitrarily name since and all filepaths are user-defined and each filepath defined within a file should be relative to that file.
The only requirement are that the files are valid JSON and that all expected keys and values are defined.


***************
Main input file
***************

The main input file (or *command* file) of SEAHOWL is a JSON file, which can be named arbitrarily.
In this file, the following is set: global numerical options, output options, environmental conditions to use, and the list of turbines with their positions and the filepath of their own JSON files.

* **numerics**: (dict)

   * **dt**: (float) the time stepping value for the simulation [s].
   * **t_end**: (float) the ending time of the simulation [s].

* **outputs**: (dict)

   * **dt**: (float) the time stepping value for outputs of the simulation [s].
   * **VTK**: (bool) whether VTK will be part of outputs or not.
   * **log_level**: (string) global log level ("critical", "error", "warning", "info", "debug", "trace").

* **environment**: (dict)

   * **file**: file path of environment file (relative to this file path).

* **turbines**: (list of dict)

   * **file**: file path of turbine file (relative to this file path).
   * **translation**: (float array of length 3) translation of turbine in space [m].
   * **rotation**: rotation of turbine (yaw) [deg].


.. literalinclude:: ../../../data/IEA15MW/main.json
   :language: json
   :linenos:
   :caption: Main input file example

**********************
Environment input file
**********************

The environment input file contains information for the wind, wave, and current conditions.

.. literalinclude:: ../../../data/IEA15MW/environment.json
   :language: json
   :linenos:
   :caption: Environment input file example

Wind types
==========

Wind ramp
---------

.. code-block:: json

   "wind": {
      "type": "ramp",
      "options": {
         "reference_height": 150,
         "shear_coefficient": 0.12,
         "velocity_start": [12, 0, 0],
         "velocity_stop": [25, 0, 0],
         "time_start": 500,
         "time_stop": 1700
      }
   }

Wind InflowWind
---------------

(requires InflowWind dependency)

.. code-block:: json

   "wind": {
      "type": "inflowwind",
      "options": {
         "file_inflowwind": "./aerodyn/IEA-15-240-RWT_InflowWind.dat",
         "file_windwnd": "./aerodyn/long_step_wind.wnd"
      }
   }


******************
Turbine input file
******************

The turbine input file contains information for the rotor, the RNA, the tower, and the controller.
Some of the options of each component are set in this file (e.g. blade pitch and precone, elasto and aero discretization of blades and towers), while more detailed options of each components are described in other files that are referred here (such as for blades and tower material properties).

Rotor "type" can be "fea", "rigid", or "disk". **fpm**: (bool) if true, use Fully-Populated Matrix (FPM) elements (6x6 material properties).
Controller "type" can be "DISCON" or "RPM".

For discretization of blades and tower, it is possible to either use an ordered array of floats between 0 and 1 (with 0 and 1 included in the array as bounds) corresponding to the normalized abscissa of the reference points or only one integer corresponding to the number of elements to use for discretization.

* **rotor**: (dict)

   * **type**: (string) type of rotor ("fea", "rigid", "disk").
   * **options**: (dict) options specific to type of rotor.

      * **discretization**: (dict)

   * **elasto**: (array of floats) discretization fractions (between 0 and 1) for elasto part of blade.
   * **aero**: (array of floats) discretization fractions (between 0 and 1) for aero part of blade.

   * **blades**: (list)

      * **file**: file path of blade file (relative to this file path).
      * **initial_pitch**: initial pitch of blade [deg].
      * **precone**: (float) precone of blade [deg].

* **rna**: (dict)

   * **initial_pitch_collective**: initial collective pitch of blades [deg].
   * **file**: file path of RNA file (relative to this file path).

* **tower**: (dict)

   * **discretization**: (dict)

   * **elasto**: (array of floats) discretization fractions (between 0 and 1) for elasto part of blade.
   * **aero**: (array of floats) discretization fractions (between 0 and 1) for aero part of blade.

   * **file**: file path of tower file (relative to this file path).

* **controller**: (dict)

   * **type**: (string) type of controller.
   * **options**: (dict) options specific to type of controller.

* **floater**: (optional dict) uses floater if defined

   * **file**: file path of floater file


.. literalinclude:: ../../../data/IEA15MW/turbine.json
   :language: json
   :linenos:
   :caption: Turbine input file example


Rotor types
===========

Controller types
================

No Controller
-------------

.. code-block:: json

   "controller": {
      "type": ""
   }

Controller DISCON
-----------------

The controller for bladed-style DISCON routine (e.g. ROSCO controller), where the path to the controller options file (DISCON.IN) and path to library (libdiscon.so for Linux, or dll for Windows) must be provided.

.. code-block:: json

   "controller": {
      "type": "DISCON",
         "options": {
            "infile": "path/to/DISCON.IN",
            "libfile": "path/to/libdiscon.so"
         }
   }

Controller RPM
--------------

For this controller, only variable torque is applied and a target RPM is set as the maximum RPM allowed for the rotor.

.. code-block:: json

   "controller": {
      "type": "RPM",
      "options": {
         "target_rpm": 5.0
      }
   }


**************
RNA input file
**************

The RNA input file contains properties of the hub, the shaft, the nacelle, and the drivetrain.

* **shaft**: (dict)

   * **tilt**: (float) tilt of shaft [deg].
   * **distance_from_towertop**: distance of shaft frol towertop [m].

* **nacelle**: (dict)

   * **inertia**: (float) inertia of nacelle [kg.m2].
   * **mass**: (float) mass of nacelle [kg]
   * **CM**: (array of floats of length 3) center of mass offset from towertop [m].
   * **yaw_bearing_mass**: mass of yaw bearing [kg].

* **drivetrain**: (dict)

   * **generator_efficiency**: (float) generator efficiency [%].
   * **generator_inertia**: (float) generator inertia [kg.m2].
   * **gearbox_ratio**: (float) gearbox ratio [-].
   * **gearbox_efficiency**: (float) geabox efficiency [%].

* **hub**: (dict)

   * **radius**: (float) radius of hub [m].
   * **overhang**: (float) overhang of hub [m].
   * **inertia**: (float) inertia of hub [kg.m2].
   * **mass**: (float) mass of hub [kg].
   * **CM**: (float) offset of center of mass of hub [m].


.. literalinclude:: ../../../data/IEA15MW/rna.json
   :language: json
   :linenos:
   :caption: Rotor-Nacelle Assembly input file example


******************
Floater input file
******************

The floater input file contains information for the floater and its mooring system. It is only used if the "floater" key is in the turbine input file.

* **mass**: (float) total mass of floater [kg].
* **cog**: (array of floats of length 3) center of gravity of floater [m].
* **inertia**: (3x3 matrix of floats) inertia of floater [kg.m2].
* **type**: (string) type of floater (only "HydroChrono" available).
* **options**:

   * **file**: (string) file path of hydro .h5 file for HydroChrono,
   * **name**: (string) name of body/floater in .h5 file


.. literalinclude:: ../../../data/IEA15MW/floater.json
   :language: json
   :linenos:
   :caption: Floater input file example


****************
Tower input file
****************

The tower input file is a reference file that should be defined only once per tower type and not be changed by the user (unless the tower properties themselves change).
All the options such as discretization of the tower during runtime are defined in the turbine input file.
Only *reference* points are defined in this file, and interpolation between them wil be used if it does not match the chosen numerical discretization during the simulation.

The tower input file contains the main properties of the tower such as its height (tower top), the height of its base (tower bottom), and damping coefficients. The rest of the properties are defined through reference points along the tower according to their "fraction", ranging from 0 (tower base) to 1 (tower top).
Each reference point contains information about elasto properties (the lineic density, the fore-aft stiffness, and the side-side stiffness) and aero properties (the diameter, and the drag coefficient).

* **type**: (string) type of tower ("cylinder" or "anisotropic").
* **height**: (float) absolute height of towertop [m].
* **base_height**: (float) absolute height of towerbase [m].
* **damping_coefficients**: (array of floats of length 4) damping coefficients of blade.

If type is "cylinder":

* **options**:

   * **density**: (float) volumic density of tower material [kg/m3]
   * **young_modulus**: (float) Young's modulus of tower material [Pa]
   * **poisson_ratio**: (float) Poisson's ratio of tower material [-]

* **reference_points**: (list of dict) list of reference points.

   * **fraction**: (float) normalized abscissa along tower (starting from base) of reference point.
   * **diameter**: (float) outer diameter of tower at reference point [m].
   * **thickness**: (float) thickness of tower at reference point [m].
   * **drag_coefficient**: (float) drag coefficient of tower at reference point [-].

If type is "anisotropic"

* **reference_points**: (list of dict) list of reference points.

   * **fraction**: (float) normalized abscissa along tower (starting from base) of reference point.
   * **density**: (float) density of tower at reference point [kg/m].
   * **stiffness_foreaft**: (float) fore-aft stiffness of tower at reference point [N.m2].
   * **stiffness_sideside**: (float) side-side stiffness of tower at reference point [N.m2].
   * **stiffness_axial**: (float) axial stiffness of tower at reference point [N.m2].
   * **stiffness_torsion**: (float) torsional stiffness of tower at reference point [N.m2].
   * **diameter**: (float) diameter of tower at reference point [m].
   * **drag_coefficient**: (float) drag coefficient of tower at reference point [-].

.. literalinclude:: ../../../data/IEA15MW/tower.json
   :language: json
   :linenos:
   :lines: -24
   :caption: Tower input file example (truncated)


****************
Blade input file
****************

The blade input file is a reference file that should be defined only once per blade type and not be changed by the user (unless the blade properties themselves change).
All the options such as discretization of the blade during runtime are defined in the turbine input file.
Only *reference* points are defined in this file, and interpolation between them wil be used if it does not match the chosen numerical discretization during the simulation.

The reference points are defined along the blade according to their "fraction", ranging from 0 (blade root) to 1 (blade tip). The global **damping coefficients** of the blade are also defined in this file.
Each reference point is defined with its coordinates (in IEC standard) along with
elasto properties (the twist along the main axis, the 6x6 mass matrix, and the 6x6 stiffness matrix)
and the aero properties (the airfoil filepath, the chord length, and the aerodynamic offset if any).
The airfoil filepath is only used built-in BEMT is used for aerodynamics.

* **damping_coefficients**: (array of floats of length 4) damping coefficients of blade.
* **reference_points**: (list of dict) list of reference points.

   * **coordinates**: (array of floats of length 3) coordinates of reference point (IEC standard) [m].
   * **twist**: (float) structural twist of blade at reference point [deg].
   * **mass_matrix**: (6x6 matrix of floats) mass matrix of blade at reference point [kg/m].
   * **stiffness_matrix**: (6x6 matrix of floats) stiffness matrix of blade at reference point [N.m2].
   * **chord**: (float) chord length of blade at reference point [m].
   * **airfoil_file**: (string) file path of airfoil file (relative to this file path).


.. literalinclude:: ../../../data/IEA15MW/blade.json
   :language: json
   :linenos:
   :lines: -51
   :caption: Blade input file example (truncated)


******************
Airfoil input file
******************

.. literalinclude:: ../../../data/IEA15MW/airfoils/IEA-15-240-RWT_AeroDyn15_Polar_37.json
   :language: json
   :linenos:
   :lines: -15
   :caption: Airfoil input file example (truncated)
