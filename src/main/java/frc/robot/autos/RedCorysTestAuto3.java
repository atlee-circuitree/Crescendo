// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualShoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightHelpers;

public class RedCorysTestAuto3 extends SequentialCommandGroup {
  
  Drivetrain m_drivetrain;
  Arm m_Arm;
  Intake m_Intake;
  RobotContainer m_RobotContainer;

  public RedCorysTestAuto3(Drivetrain Drivetrain, RobotContainer RobotContainer, Arm Arm, Intake Intake) {
  
    m_drivetrain = Drivetrain;
    m_Arm = Arm;
    m_Intake = Intake;
    m_RobotContainer = RobotContainer;

    addCommands(

      // Close Ring
      InitialPose("CorysTotallyCoolAuto1", true),
      new ManualShoot(Arm, 120),
      ChoreoPathingWithIntake("CorysTotallyCoolAuto1", true, m_Arm, m_Intake),
      new ManualShoot(Arm, 120),
      ChoreoPathing("CorysTotallyCoolAutoVersion2", true),
      DriveToNote(m_Arm, m_Intake),
      ChoreoPathingWithIntake("CorysTotallyCoolAutoVersion2(2)", true, m_Arm, m_Intake),
      new ManualShoot(m_Arm, 150),
      ChoreoPathing("CorysTotallyCoolAuto2", true),
      DriveToNote(m_Arm, m_Intake),
      ChoreoPathingWithIntake("CorysTotallyCoolAuto3", true, m_Arm, m_Intake),
      new ManualShoot(m_Arm, 120)
    

      
    );
 
  }

  private Command ChoreoPathing(String Trajectory, boolean IsRed) {

    return Choreo.choreoSwerveCommand(

      Choreo.getTrajectory(Trajectory), 
      () -> (m_drivetrain.getState().Pose), 
      Constants.AutoDrivePID, Constants.AutoDrivePID, Constants.AutoTurnPID, 
      (ChassisSpeeds speeds) -> m_drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> IsRed, 
      m_drivetrain

    );

  }

  private ParallelCommandGroup ChoreoPathingWithIntake(String Trajectory, boolean IsRed, Arm Arm, Intake Intake) {

    return new ParallelCommandGroup(
      
      ChoreoPathing(Trajectory, IsRed),
      new ManualIntake(Intake, Arm, 120)
      
    );

  }

  private ParallelRaceGroup DriveToNote(Arm Arm, Intake Intake) {

    return new ParallelCommandGroup( // Drive toward the center close note until the limelight loses detection

      new ManualIntake(Intake, Arm, 120), // Enables intake
      m_drivetrain.applyRequest(() -> m_RobotContainer.driveRobotCentric // Robot centric drive command that adjusts off of the lightlight
        .withVelocityX(1.8)  
        .withVelocityY(-LimelightHelpers.getTX("limelight-ri") / 30) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-ri") / 40))

    ).until(() -> -LimelightHelpers.getTX("limelight-ri") / 30 == 0);

  }

  private ParallelRaceGroup DriveToSpeaker(Arm Arm, Intake Intake) {

    return new ParallelCommandGroup( // Drive toward the center close note until the limelight loses detection

      new ManualIntake(Intake, Arm, 120), // Enables intake
      m_drivetrain.applyRequest(() -> m_RobotContainer.driveRobotCentric // Robot centric drive command that adjusts off of the lightlight
        .withVelocityX(3.4 * (1 - LimelightHelpers.getTA("limelight-sh")) )  
        .withVelocityY(-LimelightHelpers.getTX("limelight-sh") / 12) 
        .withRotationalRate(0))

    ).until(() -> LimelightHelpers.getTA("limelight-sh") >= .6);

  }

  private Command KillDrive(Arm Arm, Intake Intake) {

    return new ParallelCommandGroup( // Drive toward the center close note until the limelight loses detection

      m_drivetrain.applyRequest(() -> m_RobotContainer.driveRobotCentric // Robot centric drive command that adjusts off of the lightlight
        .withVelocityX(0)  
        .withVelocityY(0) 
        .withRotationalRate(0))

    ).withTimeout(.05);

  }

  private Command AdjustAngle(Arm Arm, Intake Intake) {

    return new ParallelCommandGroup( // Drive toward the center close note until the limelight loses detection

      m_drivetrain.applyRequest(() -> m_RobotContainer.driveRobotCentric // Robot centric drive command that adjusts off of the lightlight
        .withVelocityX(0)  
        .withVelocityY(0) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-ri") / 14))

    ).withTimeout(1);

  }
 
  private Command InitialPose(String Trajectory, boolean IsRed) {

    return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(new Pose2d(
      16.565 - Choreo.getTrajectory(Trajectory).getInitialPose().getX(),
      Choreo.getTrajectory(Trajectory).getInitialPose().getY(),
     // Choreo.getTrajectory(Trajectory).getInitialPose().getRotation().minus(new Rotation2d(3.1415)))));
      new Rotation2d(-Math.PI).minus(Choreo.getTrajectory(Trajectory).getInitialPose().getRotation()))));
  }

}

