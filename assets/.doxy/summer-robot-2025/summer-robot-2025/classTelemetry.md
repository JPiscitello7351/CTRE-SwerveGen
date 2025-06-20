

# Class Telemetry



[**ClassList**](annotated.md) **>** [**Telemetry**](classTelemetry.md)










































## Public Functions

| Type | Name |
| ---: | :--- |
|  void | [**Telemeterize**](#function-telemeterize) (subsystems::CommandSwerveDrivetrain::SwerveDriveState const & state) <br> |
|   | [**Telemetry**](#function-telemetry) (units::meters\_per\_second\_t maxSpeed) <br> |




























## Public Functions Documentation




### function Telemeterize 

```C++
void Telemetry::Telemeterize (
    subsystems::CommandSwerveDrivetrain::SwerveDriveState const & state
) 
```



Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. 


        

<hr>



### function Telemetry 

```C++
Telemetry::Telemetry (
    units::meters_per_second_t maxSpeed
) 
```



Construct a telemetry object with the specified max speed of the robot.




**Parameters:**


* `maxSpeed` Maximum speed 




        

<hr>

------------------------------
The documentation for this class was generated from the following file `src/main/include/Telemetry.h`

