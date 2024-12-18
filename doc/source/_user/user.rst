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
   * **duration**: (float) the duration of the simulation [s].
   * **statics**: (dict)

      * **linear_step**: (bool) linear statics step (true/false).
      * **linear_step**: (float) Number of nonlinear statics step.

   * **presimulation**: (dict)

      * **dt**: (float) the time stepping value for the presimulation [s].
      * **duration**: (float) the duration of the presimulation [s].
      * **presetup**: (bool) presetup during presimulation (true/false), e.g. mooring stretching, blade damping.
      * **fix_tower**: (float) fix tower bottoms during presimulation (true/false).

* **outputs**: (dict)

   * **dt**: (float) the time stepping value for outputs of the simulation [s].
   * **folder**: (str) Path to the folder for outputs.
   * **VTK**: (bool) output VTK (true/false).
   * **log_level**: (string) global log level ("critical", "error", "warning", "info", "debug", "trace").
   * **gui**: (bool) in situ visualization (true/false).

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

   * **initial_yaw**: initial yaw of the RNA [deg].
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

   * **inertia**: (3x3 matrix of floats) inertia of nacelle [kg-m2].
   * **mass**: (float) mass of nacelle [kg]
   * **position_from_towertop**: (array of floats of length 3) center of mass offset from towertop [m].
   * **yaw_bearing_mass**: mass of yaw bearing [kg].

* **drivetrain**: (dict)

   * **generator_efficiency**: (float) generator efficiency [%].
   * **generator_inertia**: (float) generator inertia [kg-m2].
   * **gearbox_ratio**: (float) gearbox ratio [-].
   * **gearbox_efficiency**: (float) geabox efficiency [%].

* **hub**: (dict)

   * **radius**: (float) radius of hub [m].
   * **overhang**: (float) overhang of hub [m].
   * **inertia**: (3x3 matrix of floats) inertia of hub [kg-m2].
   * **mass**: (float) mass of hub [kg].
   * **position_from_apex**: (array of floats of length 3) offset of center of mass of hub [m].


.. literalinclude:: ../../../data/IEA15MW/rna.json
   :language: json
   :linenos:
   :caption: Rotor-Nacelle Assembly input file example


******************
Floater input file
******************

The floater input file contains information for the floater and its mooring system. It is only used if the "floater" key is in the turbine input file.

* **position**: (array of floats of length 3) center of gravity of floater [m].
* **mass**: (float) total mass of floater [kg].
* **inertia**: (3x3 matrix of floats) inertia of floater [kg-m2].
* **damping_matrix**: (6x6 matrix of floats) viscous damping matrix of floater [N/(m/s),N/(rad/s),N-m/(m/s),N-m/(rad/s)].
* **type**: (string) type of floater (e.g. "HydroChrono").
* **options**: (dict)

   * **file**: (string) file path of hydro .h5 file for HydroChrono,

* **bodies**: (list of dict) list of bodies of the floater (linked to main body).

   * **name**: (string) name of body/floater (can match name in .h5 file if using HydroChrono)
   * **position**: (array of floats of length 3) center of gravity of floater [m].
   * **mass**: (float) total mass of floater [kg].
   * **inertia**: (3x3 matrix of floats) inertia of floater [kg-m2].

* **moorings**: (list of dict) list of moorings of the floater.

   * **connected_body_name**: (string) name of body where the mooring fairlead is connected.
   * **length**: (float) length of the mooring line [m].
   * **line_properties**: (string) path to mooring line properties file.
   * **fairlead_position**: (array of floats of length 3) position of fairlead [m].
   * **relative_fairlead**: (bool) fairlead position expressed relative to connected body position (true/false).
   * **anchor_position**: (array of floats of length 3) position of anchor [m].
   * **relative_anchor**: (bool) anchor position expressed relative to connected body position (true/false).
   * **rotation_axis**: (array of floats of length 3) axis around which to rotate the mooring [-].
   * **rotation_angle**: (float) angle to rotate the mooring [deg].
   * **discretization**: (dict)

      * **elasto**: (array of floats) discretization fractions (between 0 and 1) for elasto part of mooring [-].
      * **hydro**: (array of floats) discretization fractions (between 0 and 1) for hydro part of mooring [-].


.. literalinclude:: ../../../data/IEA15MW/floater.json
   :language: json
   :linenos:
   :caption: Floater input file example


****************
Tower input file
****************

The tower input file is a CSV or JSON reference file that should be defined only once per tower type and not be changed by the user (unless the tower properties themselves change).
All the options such as discretization of the tower during runtime are defined in the turbine input file.
Only *reference* points are defined in this file, and interpolation between them wil be used if it does not match the chosen numerical discretization during the simulation.

* **position_x**, **position_y**, **position_z**: (float) position of reference point [m]
* **diameter**: (float) diameter of tower at reference point [m]
* **thickness**: (float) thickness of tower at reference point [m]
* **density**: (float) linear density of tower at reference point [kg/m]
* **young_modulus**: (float) Young's modulus of tower material at reference point [Pa]
* **poisson_ratio**: (float) Poisson's ratio of tower material at reference point [-]
* **damping_x**, **damping_y**, **damping_z**, **damping_t**: (float) Damping coefficients at reference point [-]

.. literalinclude:: ../../../data/IEA15MW/tower.csv
   :language: json
   :linenos:
   :lines: -3
   :caption: Tower input file example (truncated)

Alternatively, the tower can also be defined in a JSON format as input as follows:

.. literalinclude:: ../../../data/IEA15MW/tower.json
   :language: json
   :linenos:
   :lines: -20
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

* **global_variables**: (dict) values to apply to all reference points (unless defined in reference point).

   * **damping_coefficients**: (array of floats of length 4) damping coefficients of blade.
   * **offset_gravity**: (array of floats of length 2) Gravity offset [m].
   * **offset_elastic**: (array of floats of length 2) Elastic offset [m].

* **reference_points**: (list of dict) list of reference points.

   * **coordinates**: (array of floats of length 3) coordinates of reference point (IEC standard) [m].
   * **twist**: (float) structural twist of blade at reference point [deg].
   * **mass_matrix**: (6x6 matrix of floats) mass matrix of blade at reference point [kg/m, (kg-m2)/m].
   * **stiffness_matrix**: (6x6 matrix of floats) stiffness matrix of blade at reference point [(N/m)/m, (N/rad)/m, (N-m/m)/m, (N-m/rad)/m].
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
