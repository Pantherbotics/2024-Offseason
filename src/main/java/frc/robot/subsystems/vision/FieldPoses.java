// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.opencv.core.Rect2d;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class FieldPoses {

    private static final double FIELD_LENGTH =  16.54;
        
    private static final Pose2d ampPose = new Pose2d(1.8,7.65, Rotation2d.fromDegrees(90));
    private static final Pose2d speakerPose = new Pose2d(0,5.5, Rotation2d.fromDegrees(90));
    private static final Pose2d ampPassPose = new Pose2d(2,7, Rotation2d.fromDegrees(90));
    private static final Pose2d midPassPose = new Pose2d(7,6, Rotation2d.fromDegrees(90));
    public static boolean isRedAlliance = false;
    public static Pose2d kAmpPose = ampPose;
    public static Pose2d kSpeakerPose = speakerPose;
    public static Pose2d kAmpPassPose = ampPassPose;
    public static Pose2d kMidPassPose = midPassPose;

    private static final Rect2d shootRegion = new Rect2d(0,0, 5.86, 8.2);
    private static final Rect2d ampPassRegion = new Rect2d(5.86, 0, 4.82, 8.2);
    private static final Rect2d midPassRegion = new Rect2d(10.68, 0, 5.86, 8.2);
    public static Rect2d kShootRegion = shootRegion;
    public static Rect2d kAmpPassRegion = ampPassRegion;
    public static Rect2d kMidPassRegion = midPassRegion;

    private static final List<Pose2d> notePosesUnflipped = List.of(
        new Pose2d(2.9, 4.1, Rotation2d.fromDegrees(0)),
        new Pose2d(2.9, 5.55, Rotation2d.fromDegrees(0)),
        new Pose2d(2.9, 7, Rotation2d.fromDegrees(0)),
        new Pose2d(8.27, 7.46, Rotation2d.fromDegrees(0)),
        new Pose2d(8.27, 5.78, Rotation2d.fromDegrees(0)),
        new Pose2d(8.27, 4.1, Rotation2d.fromDegrees(0)),
        new Pose2d(8.27, 2.42, Rotation2d.fromDegrees(0)),
        new Pose2d(8.27, 0.74, Rotation2d.fromDegrees(0))
    );

    private static final List<Pose2d> notePosesFlipped = List.of(
        new Pose2d(13.64, 4.1, Rotation2d.fromDegrees(0)),
        new Pose2d(13.64, 5.55, Rotation2d.fromDegrees(0)),
        new Pose2d(13.64, 7, Rotation2d.fromDegrees(0)),
        new Pose2d(8.27, 7.46, Rotation2d.fromDegrees(0)),
        new Pose2d(8.27, 5.78, Rotation2d.fromDegrees(0)),
        new Pose2d(8.27, 4.1, Rotation2d.fromDegrees(0)),
        new Pose2d(8.27, 2.42, Rotation2d.fromDegrees(0)),
        new Pose2d(8.27, 0.74, Rotation2d.fromDegrees(0))
    );

    public static List<Pose2d> notePoses = notePosesUnflipped;


    private static boolean hasFlipppedPoses = false;



    private static Rect2d flipFieldRect(Rect2d rect){
        return new Rect2d(FIELD_LENGTH - rect.x - rect.width, rect.y, rect.width, rect.height); 
    }

    public static void flipPoses(){
        if (!hasFlipppedPoses || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                isRedAlliance = (allianceColor == Alliance.Red);
                if (isRedAlliance){
                    kAmpPose = GeometryUtil.flipFieldPose(ampPose);
                    kSpeakerPose = GeometryUtil.flipFieldPose(speakerPose);
                    kAmpPassPose = GeometryUtil.flipFieldPose(ampPassPose);
                    kMidPassPose = GeometryUtil.flipFieldPose(midPassPose);
                    kShootRegion = flipFieldRect(shootRegion);
                    kAmpPassRegion = flipFieldRect(ampPassRegion);
                    kMidPassRegion = flipFieldRect(midPassRegion);
                    notePoses = notePosesFlipped;
                } else {
                    kAmpPose = ampPose;
                    kSpeakerPose = speakerPose;
                    kAmpPassPose = ampPassPose;
                    kMidPassPose = midPassPose;
                    kShootRegion = shootRegion;
                    kAmpPassRegion = ampPassRegion;
                    kMidPassRegion = midPassRegion;
                    notePoses = notePosesUnflipped;
                }
                hasFlipppedPoses = true;
            });
        }
    }


}


