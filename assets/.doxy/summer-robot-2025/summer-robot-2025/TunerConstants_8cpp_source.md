

# File TunerConstants.cpp

[**File List**](files.md) **>** [**cpp**](dir_fdf2b31f12d3ebb2f617242d0514024b.md) **>** [**generated**](dir_548f091a957a3b24798cc181b1013b81.md) **>** [**TunerConstants.cpp**](TunerConstants_8cpp.md)

[Go to the documentation of this file](TunerConstants_8cpp.md)


```C++
#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
```


