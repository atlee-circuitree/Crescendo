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
      new ParallelRaceGroup(
      new ManualIntake(Intake, Arm, 120),
      ChoreoPathing("CenterShoot", true) 
      ),
      //new ManualIntake(Intake, Arm, 120).withTimeout(3),
      new ManualShoot(Arm, 120),

      // Third Shot
      new ParallelRaceGroup(
      new ManualIntake(Intake, Arm, 120),
      ChoreoPathing("CenterShoot2", true)
      ),
      //new ManualIntake(Intake, Arm, 120).withTimeout(3),
      new ManualShoot(Arm, 120).withTimeout(1.3),

      // Fourth Shot
      new ParallelRaceGroup(
      new ManualIntake(Intake, Arm, 120),
      ChoreoPathing("CenterShoot3", true)
      ),

      new ParallelRaceGroup(
      new ManualIntake(Intake, Arm, 120),
      m_drivetrain.applyRequest(() -> RobotContainer.driveRobotCentric
        .withVelocityX(1)  
        .withVelocityY(0) 
        .withRotationalRate(-LimelightHelpers.getTX("limelight-ri") / 14))
      ),
         
      new ParallelRaceGroup(
      new ManualIntake(Intake, Arm, 120),
      ChoreoPathing("CenterShoot4", true)
      ),

      //new ManualIntake(Intake, Arm, 120),
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

    return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(new Pose2d(
      16.565 - Choreo.getTrajectory(Trajectory).getInitialPose().getX(),
      Choreo.getTrajectory(Trajectory).getInitialPose().getY(),
     // Choreo.getTrajectory(Trajectory).getInitialPose().getRotation().minus(new Rotation2d(3.1415)))));
      new Rotation2d(-Math.PI).minus(Choreo.getTrajectory(Trajectory).getInitialPose().getRotation()))));

  }

}
