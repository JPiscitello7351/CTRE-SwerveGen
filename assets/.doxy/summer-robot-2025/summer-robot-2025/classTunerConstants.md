

# Class TunerConstants



[**ClassList**](annotated.md) **>** [**TunerConstants**](classTunerConstants.md)




























## Public Static Attributes

| Type | Name |
| ---: | :--- |
|  constexpr swerve::SwerveModuleConstants | [**BackLeft**](#variable-backleft)   = `/* multi line expression */`<br> |
|  constexpr swerve::SwerveModuleConstants | [**BackRight**](#variable-backright)   = `/* multi line expression */`<br> |
|  constexpr swerve::SwerveDrivetrainConstants | [**DrivetrainConstants**](#variable-drivetrainconstants)   = `/* multi line expression */`<br> |
|  constexpr swerve::SwerveModuleConstants | [**FrontLeft**](#variable-frontleft)   = `/* multi line expression */`<br> |
|  constexpr swerve::SwerveModuleConstants | [**FrontRight**](#variable-frontright)   = `/* multi line expression */`<br> |
|  const CANBus | [**kCANBus**](#variable-kcanbus)   = `{kCANBusName, "./logs/example.hoot"}`<br> |
|  constexpr units::meters\_per\_second\_t | [**kSpeedAt12Volts**](#variable-kspeedat12volts)   = `5.21\_mps`<br> |
















## Public Static Functions

| Type | Name |
| ---: | :--- |
|  [**subsystems::CommandSwerveDrivetrain**](classsubsystems_1_1CommandSwerveDrivetrain.md) | [**CreateDrivetrain**](#function-createdrivetrain) () <br> |


























## Public Static Attributes Documentation




### variable BackLeft 

```C++
constexpr swerve::SwerveModuleConstants TunerConstants::BackLeft;
```




<hr>



### variable BackRight 

```C++
constexpr swerve::SwerveModuleConstants TunerConstants::BackRight;
```




<hr>



### variable DrivetrainConstants 

```C++
constexpr swerve::SwerveDrivetrainConstants TunerConstants::DrivetrainConstants;
```




<hr>



### variable FrontLeft 

```C++
constexpr swerve::SwerveModuleConstants TunerConstants::FrontLeft;
```




<hr>



### variable FrontRight 

```C++
constexpr swerve::SwerveModuleConstants TunerConstants::FrontRight;
```




<hr>



### variable kCANBus 

```C++
const CANBus TunerConstants::kCANBus;
```




<hr>



### variable kSpeedAt12Volts 

```C++
constexpr units::meters_per_second_t TunerConstants::kSpeedAt12Volts;
```




<hr>
## Public Static Functions Documentation




### function CreateDrivetrain 

```C++
static subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain () 
```



Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot program. 


        

<hr>

------------------------------
The documentation for this class was generated from the following file `src/main/include/generated/TunerConstants.h`

