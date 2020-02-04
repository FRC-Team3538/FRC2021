# FRC2020
### 2/3/20:
  1. Patched a Build Error and a Runtime Error
### 2/1/20:
  1. [Debugged Vision, Shooter, and Integration of These Systems](src/main/cpp/Robot.cpp)
     - [Tuned Shooter and Distancing](https://drive.google.com/open?id=1-B2QTZ6yiJMroI5dedXIeb6OXCrGLCgO)
     - Fixed Bugs With Auto Shooting and Feeding
  2. [Added Auto State Machines](src/main/cpp/auto)
     - Basic State Machine From Previous Years
     - Inherited Class For Ease and Consistency
  3. [Added and Tuned Auto Drive Forward and Turning](src/main/cpp/subsystem/Drivebase.cpp)
     - Tested Driving Forward and Turning Using Falcon Encoders
  
### 1/29/20:
  1. [Added Limelight Vision Tracking](src/main/cpp/subsystem/RJVisionPipeline.cpp) 
     - Two/Three Pointer Options
     - PNP
  2. [Added Auto Aim & Distancing](src/main/cpp/subsystem/Shooter.cpp)
     - Currently Editing Flywheel Speed Rather Than Hood Angle
     - PDF Loop

### 1/25/20:
  1. [Wrote Gearbox-Specific Drivetrain Code](src/main/cpp/subsystem/Drivebase.cpp)

### 1/18/20:
  1. Made Code Skeleton
     - [Imported Previous Year's Code Into 2020](src/main)
     - Cleaned and Gutted Unecessary and Irrelevant Code
  2. Added Color Wheel
     - [Tuned Color Sensing](src/main/cpp/subsystem/ColorWheel.cpp)
     - Integrated Color Detection
  3. Added Shooter Code
     - [Temporary Manual Hood Control](src/main/cpp/subsystem/Shooter.cpp)
