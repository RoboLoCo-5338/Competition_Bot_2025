# RoboLoCo Simulation

This document goes over the details and setup for the 3D simulation for the 2025 Competition robot.

## Prerequisites:
1) Install the [FRC Game Tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html)
2) Install the [WPILib software suite](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
3) Install [Git](https://git-scm.com/downloads)
4) Install [Git LFS](https://git-lfs.com/)

## Initial Setup:
1) If you do not already have the repository installed, perform the following: 
    1) Open Git Bash. Optionally, change the folder you'll work in by typing `cd <folder>`
    2) Enable Git LFS with command `git lfs install`
    3) Clone the 2025 Competition Bot repository with command `git clone https://github.com/RoboLoCo-5338/Competition_Bot_2025.git`. You'll have to log into your GitHub account to continue
    4) Enter the repo folder with `cd ./Competition_Bot_2025`
    5) Checkout the simulation branch with `git checkout advantagescope_sim`
2) Open up the repo project
    1) Open the "2025 WPILib VS Code"
    2) Click in the top left File->Open Folder
    3) Navigate to the "Competition_Bot_2025" folder
3) Click the WPILib logo in the top right and select "WPILib: Simulate Robot Code". This will first build the code, then sim it. If the code does not build properly, perform the same steps but press "WPILib: Build Robot Code" first.
4) A popup will show at the top with options for "Sim GUI" or "Use Real DriverStation"
    1) Selecting "Sim GUI" will open a window that has:
        1) Set Disabled, Autonomous, Teleoperated
        2) View LED state
        3) Connect and view controllers
        4) Simulated a controller with keyboard keys
        5) View simulated motor and encoder values
        6) Many additional things
    2) Selecting "Use Real DriverStation" will require you to connect a real DriverStation
        1) Open the "FRC Driver Station" program
        2) Select the gear icon on the left
        3) Under team number, put `127.0.0.1` (assuming the sim and driver station are on the same computer)
5) Open the "AdvantageScope (WPILib) 2025" program
    1) Click in the top left File->Connect to Simulator
6) Import the 3D CAD configuration
    1) Click in the top left Help->Use Custom Assets Folder
        1) Click in the top left Help->Use Custom Assets Folder
        2) Navigate to the competition robot folder
        3) In the folder, go to the resources/assets folder and open this folder
    2) Click the + in the top right and click "3D Field"
    3) In the bottom right, click "2025 Field (AndyMark)"
    4) On the left, go to the tab AdvantageKit->RealOutputs->Odometry
    5) Drag the "Robot" object underneath the 3D Field display
    6) Under the 3D Field display, click the arrow on the left of the "NT:/AdvantageKit/RealOutputs/Odometry/Robot" and click "USSRivets"
7) Select what you want to view for the simulation:
    1) To automatically set up the layout(skips steps 20-22):
        1) Click in the top left File->Import Layout
        2) Navigate to the competition bot folder
        3) Select SimLayout.json
        4) All the tabs should appear. For reference, they are:
            1) Line Graph: Allows you to chart values
            2) Odometry: 2d field odometry
            3) 3d Field: Allows for 3d visualization of the robot
            4) Mechanism: Allows for 2d visualization of a mechanism
            5) Controller: Visualization of driver(left) and operator(right) controller inputs
            6) Console: Shows Code Console
    2) For manual 2D mechanism simulation setup:
        1) Click the + in the top right and click "Mechanism"
        2) On the left, go to the tab AdvantageKit->RealOutputs->Elevator
        3) Drag the "Mechanism" object here underneath the Mechanism display
    3) For manual 2D field simulation setup:
        1) Click the + in the top right and click "Odometry"
        2) In the bottom right, click "2025 Field (AndyMark)"
        3) On the left, go to the tab AdvantageKit->RealOutputs->Odometry
        4) Drag the "Robot" object underneath the Odometry display
    4) For manual 3D field and mechanism simulation setup, drag the following onto the "Robot" object underneath the 3D Field in this exact order
          1) ElevatorStage1
          2) ElevatorStage2
          3) EndEffector
          4) EndEffectorFrontRoller
          5) EndEffectorBackRoller
          6) BackLeftSwerveModule
          7) BackLeftWheel
          8) BackRightSwerveModule
          9) BackRightWheel
          10) FrontLeftSwerveModule
          11) FrontLeftWheel
          12) FrontRightSwerveModule
          13) FrontRightWheel
      8) Ensure each of these appears as a "Component" next to their name. If it shows "Vision Target" instead, click the icon next to the name and switch to "Component"
      9) Right click the field and click which view you want to have (including the cameras on the robot!)
8) Start your simulation by selecting Autonomous or connect a controller and select Teleoperated in your Sim GUI or Driver Station!

## Known issues:
1) LED code is disabled as it causes the system to lock upe
2) Soft limits do not seem to work properly in simulation
3) The end effector starts up higher at the beginning of autonomous rather than hanging down
4) General simulation organization and naming issues

## Other things to explore:
1) Explore all the values you can monitor on the left side during operation
2) You can click on the left of lines of code next to the line numbers to create a "Breakpoint" to pause the code when reaching that line
3) Change the graphics fidelity: Help->Preferences->3D Mode (Default)
4) It's possible to [replay logs](https://docs.advantagekit.org/getting-started/traditional-replay) using the simulation to try to do minor modifications!
