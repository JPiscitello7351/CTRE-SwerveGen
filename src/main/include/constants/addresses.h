/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

namespace address {
    namespace argobotRed {
        namespace drive{
            constexpr int frontLeftDrive = 1;
            constexpr int frontLeftTurn = 2;
            constexpr int frontRightDrive = 3;
            constexpr int frontRightTurn = 4;
            constexpr int backRightDrive = 5;
            constexpr int backRightTurn = 6;
            constexpr int backLeftDrive = 7;
            constexpr int backLeftTurn = 8;
        }
    }

    namespace argobotBlue {
        namespace drive{
            constexpr int frontLeftDrive = address::argobotRed::drive::frontLeftDrive;
            constexpr int frontLeftTurn = address::argobotRed::drive::frontLeftTurn;
            constexpr int frontRightDrive = address::argobotRed::drive::frontRightDrive;
            constexpr int frontRightTurn = address::argobotRed::drive::frontRightTurn;
            constexpr int backRightDrive = address::argobotRed::drive::backRightDrive;
            constexpr int backRightTurn = address::argobotRed::drive::backRightTurn;
            constexpr int backLeftDrive = address::argobotRed::drive::backLeftDrive;
            constexpr int backLeftTurn = address::argobotRed::drive::backLeftTurn;
            
        }
    }
}