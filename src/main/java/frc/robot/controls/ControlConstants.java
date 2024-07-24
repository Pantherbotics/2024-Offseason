// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

/** Add your docs here. */
public class ControlConstants {

    public int kMoveYAxisPort;
    public int kMoveXAxisPort;
    public int kRotateAxisPort;

    public int kIntakeButtonPort;
    public int kShootButtonPort;
    public int kAmpButtonPort;
    public int kPassButtonPort;
    public int kClimbButtonPort;

    public ControlConstants(InputType type){
        switch (type) {
            case XBOX:
                kMoveYAxisPort = 1;
                kMoveXAxisPort = 0;
                kRotateAxisPort = 4;
                kIntakeButtonPort = 5;
                kShootButtonPort = 4;
                kAmpButtonPort = 6;
                kPassButtonPort = 7;
                kClimbButtonPort = 0;
            break;
            case JOYSTICK:
                kMoveYAxisPort = 1;
                kMoveXAxisPort = 0;
                kRotateAxisPort = 3;
                kIntakeButtonPort = 0;
                kShootButtonPort = 1;
                kAmpButtonPort = 2;
                kPassButtonPort = 3;
                kClimbButtonPort = 5;
            break;
            default:
                kMoveYAxisPort = 1;
                kMoveXAxisPort = 0;
                kRotateAxisPort = 4;
                kIntakeButtonPort = 5;
                kShootButtonPort = 4;
                kAmpButtonPort = 6;
                kPassButtonPort = 7;
                kClimbButtonPort = 0;
            break;
        }
        
    }

    public static enum InputType{
        JOYSTICK,
        XBOX,
        PLAYSTATION
    }



}
