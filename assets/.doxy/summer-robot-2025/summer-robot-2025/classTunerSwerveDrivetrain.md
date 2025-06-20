

# Class TunerSwerveDrivetrain



[**ClassList**](annotated.md) **>** [**TunerSwerveDrivetrain**](classTunerSwerveDrivetrain.md)



_Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types._ 

* `#include <TunerConstants.h>`



Inherits the following classes: swerve::SwerveDrivetrain< hardware::TalonFX, hardware::TalonFX, hardware::CANcoder >


Inherited by the following classes: [subsystems::CommandSwerveDrivetrain](classsubsystems_1_1CommandSwerveDrivetrain.md)












## Public Types

| Type | Name |
| ---: | :--- |
| typedef swerve::SwerveModuleConstants&lt; configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration &gt; | [**SwerveModuleConstants**](#typedef-swervemoduleconstants)  <br> |




















## Public Functions

| Type | Name |
| ---: | :--- |
|   | [**TunerSwerveDrivetrain**](#function-tunerswervedrivetrain-13) (swerve::SwerveDrivetrainConstants const & driveTrainConstants, ModuleConstants const &... modules) <br>_Constructs a CTRE SwerveDrivetrain using the specified constants._  |
|   | [**TunerSwerveDrivetrain**](#function-tunerswervedrivetrain-23) (swerve::SwerveDrivetrainConstants const & driveTrainConstants, units::hertz\_t odometryUpdateFrequency, ModuleConstants const &... modules) <br>_Constructs a CTRE SwerveDrivetrain using the specified constants._  |
|   | [**TunerSwerveDrivetrain**](#function-tunerswervedrivetrain-33) (swerve::SwerveDrivetrainConstants const & driveTrainConstants, units::hertz\_t odometryUpdateFrequency, std::array&lt; double, 3 &gt; const & odometryStandardDeviation, std::array&lt; double, 3 &gt; const & visionStandardDeviation, ModuleConstants const &... modules) <br>_Constructs a CTRE SwerveDrivetrain using the specified constants._  |




























## Public Types Documentation




### typedef SwerveModuleConstants 

```C++
using TunerSwerveDrivetrain::SwerveModuleConstants =  swerve::SwerveModuleConstants<configs::TalonFXConfiguration, configs::TalonFXConfiguration, configs::CANcoderConfiguration>;
```




<hr>
## Public Functions Documentation




### function TunerSwerveDrivetrain [1/3]

_Constructs a CTRE SwerveDrivetrain using the specified constants._ 
```C++
template<std::same_as< SwerveModuleConstants >... ModuleConstants>
inline TunerSwerveDrivetrain::TunerSwerveDrivetrain (
    swerve::SwerveDrivetrainConstants const & driveTrainConstants,
    ModuleConstants const &... modules
) 
```



This constructs the underlying hardware devices, so users should not construct the devices themselves. If they need the devices, they can access them through getters in the classes.




**Parameters:**


* `drivetrainConstants` Drivetrain-wide constants for the swerve drive 
* `modules` Constants for each specific module 




        

<hr>



### function TunerSwerveDrivetrain [2/3]

_Constructs a CTRE SwerveDrivetrain using the specified constants._ 
```C++
template<std::same_as< SwerveModuleConstants >... ModuleConstants>
inline TunerSwerveDrivetrain::TunerSwerveDrivetrain (
    swerve::SwerveDrivetrainConstants const & driveTrainConstants,
    units::hertz_t odometryUpdateFrequency,
    ModuleConstants const &... modules
) 
```



This constructs the underlying hardware devices, so users should not construct the devices themselves. If they need the devices, they can access them through getters in the classes.




**Parameters:**


* `drivetrainConstants` Drivetrain-wide constants for the swerve drive 
* `odometryUpdateFrequency` The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0. 
* `modules` Constants for each specific module 




        

<hr>



### function TunerSwerveDrivetrain [3/3]

_Constructs a CTRE SwerveDrivetrain using the specified constants._ 
```C++
template<std::same_as< SwerveModuleConstants >... ModuleConstants>
inline TunerSwerveDrivetrain::TunerSwerveDrivetrain (
    swerve::SwerveDrivetrainConstants const & driveTrainConstants,
    units::hertz_t odometryUpdateFrequency,
    std::array< double, 3 > const & odometryStandardDeviation,
    std::array< double, 3 > const & visionStandardDeviation,
    ModuleConstants const &... modules
) 
```



This constructs the underlying hardware devices, so users should not construct the devices themselves. If they need the devices, they can access them through getters in the classes.




**Parameters:**


* `drivetrainConstants` Drivetrain-wide constants for the swerve drive 
* `odometryUpdateFrequency` The frequency to run the odometry loop. If unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0. 
* `odometryStandardDeviation` The standard deviation for odometry calculation in the form [x, y, theta]ᵀ, with units in meters and radians 
* `visionStandardDeviation` The standard deviation for vision calculation in the form [x, y, theta]ᵀ, with units in meters and radians 
* `modules` Constants for each specific module 




        

<hr>

------------------------------
The documentation for this class was generated from the following file `src/main/include/generated/TunerConstants.h`

