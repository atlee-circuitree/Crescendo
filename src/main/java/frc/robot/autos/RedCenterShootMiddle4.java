// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.AlwaysRunningIntake;
import frc.robot.commands.AutoArm;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.ManualShoot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightHelpers;

public class RedCenterShootMiddle4 extends SequentialCommandGroup {
  
  Drivetrain m_drivetrain;
  RobotContainer m_RobotContainer;

  public RedCenterShootMiddle4(Drivetrain Drivetrain, RobotContainer RobotContainer, Arm Arm, Intake Intake) {
  
    m_drivetrain = Drivetrain;
    m_RobotContainer = RobotContainer;

    addCommands(

      // First Shot
      new ManualShoot(Arm, 120),
      InitialPose("CenterShoot", true),

      // Second Shot
      new ParallelCommandGroup(
      new ManualIntake(Intake, Arm, 120),
      ChoreoPathing("CenterShoot", true) 
      ).withTimeout(3.4),
      new ManualIntake(Intake, Arm, 120),
      new ManualShoot(Arm, 120),

      // Third Shot
      new ParallelCommandGroup(
      new ManualIntake(Intake, Arm, 120),
      ChoreoPathing("CenterShoot2", true)
      ),
      new ManualIntake(Intake, Arm, 120),
      new ManualShoot(Arm, 120).withTimeout(3.8),

      // Fourth Shot
      new ParallelCommandGroup(
      new ManualIntake(Intake, Arm, 120),
      ChoreoPathing("CenterShoot3", true)
      ).withTimeout(1.3),

      new ParallelCommandGroup(
      new ManualIntake(Intake, Arm, 120),
      m_drivetrain.applyRequest(() -> RobotContainer.driveRobotCentric
        .withVelocityX(1)  
        .withVelocityY(0) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-ri") / 14))
      ).withTimeout(1.1),
         
      new ParallelCommandGroup(
      new ManualIntake(Intake, Arm, 120),
      ChoreoPathing("CenterShoot4", true)
      ).withTimeout(2),

      new ManualIntake(Intake, Arm, 120),
      new ManualShoot(Arm, 120)
      
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

  private Command InitialPose(String Trajectory, boolean IsRed) {

    if (IsRed = true) {

      return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(Choreo.getTrajectory(Trajectory).getInitialPose()));
      
    } else {

      return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(Choreo.getTrajectory(Trajectory).flipped().getInitialPose()));

    }

  }

}
