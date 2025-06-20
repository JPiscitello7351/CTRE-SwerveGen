

# Class subsystems::CommandSwerveDrivetrain



[**ClassList**](annotated.md) **>** [**subsystems**](namespacesubsystems.md) **>** [**CommandSwerveDrivetrain**](classsubsystems_1_1CommandSwerveDrivetrain.md)



_Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily be used in command-based projects._ 

* `#include <CommandSwerveDrivetrain.h>`



Inherits the following classes: frc2::SubsystemBase,  [TunerSwerveDrivetrain](classTunerSwerveDrivetrain.md)
















## Public Types inherited from TunerSwerveDrivetrain

See [TunerSwerveDrivetrain](classTunerSwerveDrivetrain.md)

| Type | Name |
| ---: | :--- |
| typedef swerve::SwerveModuleConstants&lt; configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration &gt; | [**SwerveModuleConstants**](classTunerSwerveDrivetrain.md#typedef-swervemoduleconstants)  <br> |






































## Public Functions

| Type | Name |
| ---: | :--- |
|  void | [**AddVisionMeasurement**](#function-addvisionmeasurement-12) (frc::Pose2d visionRobotPose, units::second\_t timestamp) override<br>_Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting for measurement noise._  |
|  void | [**AddVisionMeasurement**](#function-addvisionmeasurement-22) (frc::Pose2d visionRobotPose, units::second\_t timestamp, std::array&lt; double, 3 &gt; visionMeasurementStdDevs) override<br>_Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting for measurement noise._  |
|  frc2::CommandPtr | [**ApplyRequest**](#function-applyrequest-12) (RequestSupplier request) <br>_Returns a command that applies the specified control request to this swerve drivetrain._  |
|  frc2::CommandPtr | [**ApplyRequest**](#function-applyrequest-22) (RequestSupplier request) <br>_Returns a command that applies the specified control request to this swerve drivetrain._  |
|   | [**CommandSwerveDrivetrain**](#function-commandswervedrivetrain-13) (swerve::SwerveDrivetrainConstants const & driveTrainConstants, ModuleConstants const &... modules) <br>_Constructs a CTRE SwerveDrivetrain using the specified constants._  |
|   | [**CommandSwerveDrivetrain**](#function-commandswervedrivetrain-23) (swerve::SwerveDrivetrainConstants const & driveTrainConstants, units::hertz\_t odometryUpdateFrequency, ModuleConstants const &... modules) <br>_Constructs a CTRE SwerveDrivetrain using the specified constants._  |
|   | [**CommandSwerveDrivetrain**](#function-commandswervedrivetrain-33) (swerve::SwerveDrivetrainConstants const & driveTrainConstants, units::hertz\_t odometryUpdateFrequency, std::array&lt; double, 3 &gt; const & odometryStandardDeviation, std::array&lt; double, 3 &gt; const & visionStandardDeviation, ModuleConstants const &... modules) <br>_Constructs a CTRE SwerveDrivetrain using the specified constants._  |
|  void | [**Periodic**](#function-periodic) () override<br> |
|  frc2::CommandPtr | [**SysIdDynamic**](#function-sysiddynamic) (frc2::sysid::Direction direction) <br>_Runs the SysId Dynamic test in the given direction for the routine specified by m\_sysIdRoutineToApply._  |
|  frc2::CommandPtr | [**SysIdQuasistatic**](#function-sysidquasistatic) (frc2::sysid::Direction direction) <br>_Runs the SysId Quasistatic test in the given direction for the routine specified by m\_sysIdRoutineToApply._  |


## Public Functions inherited from TunerSwerveDrivetrain

See [TunerSwerveDrivetrain](classTunerSwerveDrivetrain.md)

| Type | Name |
| ---: | :--- |
|   | [**TunerSwerveDrivetrain**](classTunerSwerveDrivetrain.md#function-tunerswervedrivetrain-13) (swerve::SwerveDrivetrainConstants const & driveTrainConstants, ModuleConstants const &... modules) <br>_Constructs a CTRE SwerveDrivetrain using the specified constants._  |
|   | [**TunerSwerveDrivetrain**](classTunerSwerveDrivetrain.md#function-tunerswervedrivetrain-23) (swerve::SwerveDrivetrainConstants const & driveTrainConstants, units::hertz\_t odometryUpdateFrequency, ModuleConstants const &... modules) <br>_Constructs a CTRE SwerveDrivetrain using the specified constants._  |
|   | [**TunerSwerveDrivetrain**](classTunerSwerveDrivetrain.md#function-tunerswervedrivetrain-33) (swerve::SwerveDrivetrainConstants const & driveTrainConstants, units::hertz\_t odometryUpdateFrequency, std::array&lt; double, 3 &gt; const & odometryStandardDeviation, std::array&lt; double, 3 &gt; const & visionStandardDeviation, ModuleConstants const &... modules) <br>_Constructs a CTRE SwerveDrivetrain using the specified constants._  |






















































## Public Functions Documentation




### function AddVisionMeasurement [1/2]

_Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting for measurement noise._ 
```C++
inline void subsystems::CommandSwerveDrivetrain::AddVisionMeasurement (
    frc::Pose2d visionRobotPose,
    units::second_t timestamp
) override
```





**Parameters:**


* `visionRobotPose` The pose of the robot as measured by the vision camera. 
* `timestamp` The timestamp of the vision measurement in seconds. 




        

<hr>



### function AddVisionMeasurement [2/2]

_Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate while still accounting for measurement noise._ 
```C++
inline void subsystems::CommandSwerveDrivetrain::AddVisionMeasurement (
    frc::Pose2d visionRobotPose,
    units::second_t timestamp,
    std::array< double, 3 > visionMeasurementStdDevs
) override
```



Note that the vision measurement standard deviations passed into this method will continue to apply to future measurements until a subsequent call to SetVisionMeasurementStdDevs or this method.




**Parameters:**


* `visionRobotPose` The pose of the robot as measured by the vision camera. 
* `timestamp` The timestamp of the vision measurement in seconds. 
* `visionMeasurementStdDevs` Standard deviations of the vision pose measurement. 




        

<hr>



### function ApplyRequest [1/2]

_Returns a command that applies the specified control request to this swerve drivetrain._ 
```C++
template<typename RequestSupplier>
inline frc2::CommandPtr subsystems::CommandSwerveDrivetrain::ApplyRequest (
    RequestSupplier request
) 
```



This captures the returned swerve request by reference, so it must live for at least as long as the drivetrain. This can be done by storing the request as a member variable of your drivetrain subsystem or robot.




**Parameters:**


* `request` Function returning the request to apply 



**Returns:**

Command to run 





        

<hr>



### function ApplyRequest [2/2]

_Returns a command that applies the specified control request to this swerve drivetrain._ 
```C++
template<typename RequestSupplier>
inline frc2::CommandPtr subsystems::CommandSwerveDrivetrain::ApplyRequest (
    RequestSupplier request
) 
```





**Parameters:**


* `request` Function returning the request to apply 



**Returns:**

Command to run 





        

<hr>



### function CommandSwerveDrivetrain [1/3]

_Constructs a CTRE SwerveDrivetrain using the specified constants._ 
```C++
template<std::same_as< SwerveModuleConstants >... ModuleConstants>
inline subsystems::CommandSwerveDrivetrain::CommandSwerveDrivetrain (
    swerve::SwerveDrivetrainConstants const & driveTrainConstants,
    ModuleConstants const &... modules
) 
```



This constructs the underlying hardware devices, so users should not construct the devices themselves. If they need the devices, they can access them through getters in the classes.




**Parameters:**


* `drivetrainConstants` Drivetrain-wide constants for the swerve drive 
* `modules` Constants for each specific module 




        

<hr>



### function CommandSwerveDrivetrain [2/3]

_Constructs a CTRE SwerveDrivetrain using the specified constants._ 
```C++
template<std::same_as< SwerveModuleConstants >... ModuleConstants>
inline subsystems::CommandSwerveDrivetrain::CommandSwerveDrivetrain (
    swerve::SwerveDrivetrainConstants const & driveTrainConstants,
    units::hertz_t odometryUpdateFrequency,
    ModuleConstants const &... modules
) 
```



This constructs the underlying hardware devices, so users should not construct the devices themselves. If they need the devices, they can access them through getters in the classes.




**Parameters:**


* `driveTrainConstants` Drivetrain-wide constants for the swerve drive 
* `odometryUpdateFrequency` The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0. 
* `modules` Constants for each specific module 




        

<hr>



### function CommandSwerveDrivetrain [3/3]

_Constructs a CTRE SwerveDrivetrain using the specified constants._ 
```C++
template<std::same_as< SwerveModuleConstants >... ModuleConstants>
inline subsystems::CommandSwerveDrivetrain::CommandSwerveDrivetrain (
    swerve::SwerveDrivetrainConstants const & driveTrainConstants,
    units::hertz_t odometryUpdateFrequency,
    std::array< double, 3 > const & odometryStandardDeviation,
    std::array< double, 3 > const & visionStandardDeviation,
    ModuleConstants const &... modules
) 
```



This constructs the underlying hardware devices, so users should not construct the devices themselves. If they need the devices, they can access them through getters in the classes.




**Parameters:**


* `driveTrainConstants` Drivetrain-wide constants for the swerve drive 
* `odometryUpdateFrequency` The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0. 
* `odometryStandardDeviation` The standard deviation for odometry calculation 
* `visionStandardDeviation` The standard deviation for vision calculation 
* `modules` Constants for each specific module 




        

<hr>



### function Periodic 

```C++
void subsystems::CommandSwerveDrivetrain::Periodic () override
```




<hr>



### function SysIdDynamic 

_Runs the SysId Dynamic test in the given direction for the routine specified by m\_sysIdRoutineToApply._ 
```C++
inline frc2::CommandPtr subsystems::CommandSwerveDrivetrain::SysIdDynamic (
    frc2::sysid::Direction direction
) 
```





**Parameters:**


* `direction` Direction of the SysId Dynamic test 



**Returns:**

Command to run 





        

<hr>



### function SysIdQuasistatic 

_Runs the SysId Quasistatic test in the given direction for the routine specified by m\_sysIdRoutineToApply._ 
```C++
inline frc2::CommandPtr subsystems::CommandSwerveDrivetrain::SysIdQuasistatic (
    frc2::sysid::Direction direction
) 
```





**Parameters:**


* `direction` Direction of the SysId Quasistatic test 



**Returns:**

Command to run 





        

<hr>

------------------------------
The documentation for this class was generated from the following file `src/main/include/subsystems/CommandSwerveDrivetrain.h`

