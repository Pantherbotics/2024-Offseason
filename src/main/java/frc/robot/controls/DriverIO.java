// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controls.ControlConstants.InputType;

/** Add your docs here. */
public class DriverIO extends GenericHID{

    private final ControlConstants ports;

    public DriverIO(int port, InputType type){
        super(port);
        ports = new ControlConstants(type);
    }

    public static enum goal{
        SPEAKER,
        AMP,
        PASS
    }


    public double moveX(){
        return getRawAxis(ports.kMoveXAxisPort);
    }
    public double moveY(){
        return getRawAxis(ports.kMoveYAxisPort);
    }
    public double rotate(){
        return getRawAxis(ports.kRotateAxisPort);
    }
    public Trigger intake(){
        return new Trigger(()->getRawButton(ports.kIntakeButtonPort));
    }
    public Trigger shoot(){
        return new Trigger(()->getRawButton(ports.kShootButtonPort));
    }
    public Trigger climb(){
        return new Trigger(()->getRawButton(ports.kClimbButtonPort));
    }
    public void setRumble(double left, double right){
        setRumble(RumbleType.kLeftRumble, left);
        setRumble(RumbleType.kRightRumble, right);
    }
    public void setRumble(double amount){
        setRumble(RumbleType.kLeftRumble, amount);
        setRumble(RumbleType.kRightRumble, amount);
    }

}
