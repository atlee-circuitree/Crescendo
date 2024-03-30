// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

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

public class DriveToSpeaker extends SequentialCommandGroup {
  
  Drivetrain m_drivetrain;
  Arm m_Arm;
  Intake m_Intake;
  RobotContainer m_RobotContainer;

  public DriveToSpeaker(Drivetrain Drivetrain, RobotContainer RobotContainer, Arm Arm, Intake Intake) {
  
    m_drivetrain = Drivetrain;
    m_Arm = Arm;
    m_Intake = Intake;
    m_RobotContainer = RobotContainer;

    addCommands(


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

    ).withTimeout(Choreo.getTrajectory(Trajectory).getTotalTime() * 1.2);

  }

  private ParallelCommandGroup ChoreoPathingWithIntake(String Trajectory, boolean IsRed) {

    return new ParallelCommandGroup(
      
      ChoreoPathing(Trajectory, IsRed),
      new ManualIntake(new Intake(), new Arm(), 120)
      
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
        .withVelocityX(1.8)  
        .withVelocityY(-LimelightHelpers.getTX("limelight-sh") / 8) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-sh") / 40))

    ).until(() -> LimelightHelpers.getTA("limelight-sh") >= .8);

  }

  private Command KillDrive(Arm Arm, Intake Intake) {

    return new ParallelCommandGroup( // Drive toward the center close note until the limelight loses detection

      m_drivetrain.applyRequest(() -> m_RobotContainer.driveRobotCentric // Robot centric drive command that adjusts off of the lightlight
        .withVelocityX(0)  
        .withVelocityY(-LimelightHelpers.getTX("limelight-sh") / 20) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-sh") / 40))

    ).withTimeout(.05);

  }
 
 
  private Command InitialPose(String Trajectory, boolean IsRed) {

    return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(Choreo.getTrajectory(Trajectory).getInitialPose()));

  }

}
