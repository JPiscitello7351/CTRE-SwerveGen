The first step is to create a subsystem for our drivetrain. A subsystem is a class that represents a mechanism on the robot. It should contain all the code for that mechanism, such as motors and sensors.

To create a new subsystem in VS Code with the WPILib extension:  
<br>
1. Right-click on the `src/main/include/subsystems` folder in the file explorer.  
<br>
2. Select `WPILib: Create a new class/command`.  
  ![Create Subsystem](create_subsystem_right_click.png)  
<br>
3. Choose `Subsystem` from the list.  
  ![alt text](Subystem_Selection.png)  
<br>
4. Name the new subsystem `Drivetrain`.  
  ![alt text](Drivetrain_Subsystem.png)  
<br>
5. You should see two folder now in one in `src/main/include/subsystems` called `Drivetrain.h`  and the other in `src/main/cpp/subsystems` called `Drivetrain.cpp`  
  ![alt text](Drivetrain_Files.png)
<br>
