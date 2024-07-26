// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldPoses {

        
    private static final Pose2d ampPose = new Pose2d(1.8,7.65, Rotation2d.fromDegrees(90));
    private static final Pose2d speakerPose = new Pose2d(0,5.5, Rotation2d.fromDegrees(90));
    public static Pose2d kAmpPose = ampPose;
    public static Pose2d kSpeakerPose = speakerPose;

    private static boolean hasFlipppedPoses = false;

    public static void flipPoses(){
        if (!hasFlipppedPoses || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                if (allianceColor == Alliance.Red){
                    kAmpPose = GeometryUtil.flipFieldPose(ampPose);
                    kSpeakerPose = GeometryUtil.flipFieldPose(speakerPose);
                } else {
                    kAmpPose = ampPose;
                    kSpeakerPose = speakerPose;
                }
                hasFlipppedPoses = true;
            });
        }
    }


}


