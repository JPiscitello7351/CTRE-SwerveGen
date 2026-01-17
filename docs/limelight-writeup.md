This markdown file accompanies the branch `limelight-testing`. Specifically, commit:

06e2dbcdd0cfb13d7f4569575299789335ecd22a

Which has the two commands described below fully working. It's encouraged for
the reader to look at the resources secion, as well as read the source/header
files listed below in the implementation section.

The commit after will contain this write-up.

# Purpose

Develop and demonstrate proof-of-concept commands for using a limelight to
command the drivetrain of a swerve ARGOBOT to track an AprilTag. This prooves
that a LL4 (LimeLight 4) can be used similar to retro-reflective tape used in past seasons.

# Implementation

(Note this project was implemented using 2025 WPILib to avoid some breaking changes
that were out of scope for this test)

The relevant files for this experiment are:

- `src/main/cpp/commands/TranslateToTag.cpp/h`
- `src/main/cpp/commands/ServoToTag.cpp/h`
- `src/main/cpp/subsystems/VisionSubsystem.cpp/h`

Each of the header files has doxygen documentation and source files are
well-documented with comments. It's highly recommended those are read for
more information not covered here.

The two commands that demonstrate tag tracking are `TranslateToTag` and 
`ServoToTag`, which translate the robot, and rotate the robot respectively
to center a tag on the crosshairs of a limelight mounted at the front of a
swerve drive ARGOBOT angled up at 45 degrees.

The limelight configs used for this experiment can be found in:

`llconfigs/js-limelight-testing-2026-01-16-final.vpr`

## Operating Instructions

On the index 0 controller, the B and Y buttons will activate the commands that
were written. 

**Holding B** will activate the `ServoToTag` command
**Holding Y** will activate the `TranslateToTag` command

Read each commands documentation (found in header file) before running them
for more information. (Or don't)

While these commands are active there is **NO DRIVER STICK OVERRIDE** you will
have to release the button to stop the command from running.

Tap **Left Bumper** to tare field-centric control, drive is field-centric with
translation on left stick and rotation on right.

The robot will track a tag if you hold one in front of the camera while
the commands are active. Have fun!

## Recommendations

After testing, I think that a targetting implementation (this excludes robot localization)
will have three avenues for tracking targets for shooting:

- Treating the AprilTag like a retro-reflective target and using limelight pipelines,
  configuration, and targetting settings to adjust the output tx/ty values. 
- Same as above, but instead of interfaceing with tx/ty, interfacing with the
  `rawFiducialMarkers` and extracting each tag's individual ID and tx/ty data, then
  feeding this data to a custom algorithm.
- Using the robotSpace vector for the tag, you can determine the location of the tag
  relative to the robot in 3d space. Naturally, this is exactly what localization is,
  but it would be up to the implementer to determine if this data will be mixed with
  the MegaTag2 algo, the drivetrain, or other odometry solutions.

No idea which is better. Actually localizing the robot, or tracking the pose
of a *reliable* apriltag would allow for more advanced control. Again, this
does not necessarily take localization into account, just "dumb" tracking
of a single LimeLight target (AprilTag).

Another note is that tuning one or more of the like 10 available pipelines
for different tasks and switching on the fly seems like a good idea. One
pipeline could perform targetting when needed, then localization when not
actively aiming. (or maybe you always want to aiming).

## Other Observations

- Saw about a 2-3 millisecond latency reported by the limelight.
- LimeLight recommends only using the `wpiBlue` variant of the pose estimate
- LimeLight suggests simple solutions first, such as servoing over global pose estimation
  for an application like shooting/aiming.
- Multiple LimeLights may require tweaks to the approach this `VisionSubsystem` takes.

Setting speeds to zero when a target is not in view like:

```
IF (HAS_TARGETS)
{
    SET SPEED TO (TX * CONSTANT)
}
else
{
    SET SPEED TO ZERO
}
```

Will cause jitters in the drivetrain if the target is dropped for a frame. Probably
filter for this in a final implementation.

# References

[Limelight - Limelight 4 Quick Start](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-4)

[Limelight - FRC Programming Guide](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib)

[Limelight - Tracking AprilTags](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags)

[GitHub - LimeLight Helpers Repo](https://github.com/LimelightVision/limelightlib-wpicpp)
