// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.choreo.lib.Choreo;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

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

public class LimelightBlueCenterShoot4 extends SequentialCommandGroup {
  
  Drivetrain m_drivetrain;
  RobotContainer m_RobotContainer;

  public LimelightBlueCenterShoot4(Drivetrain Drivetrain, RobotContainer RobotContainer, Arm Arm, Intake Intake) {
  
    m_drivetrain = Drivetrain;
    m_RobotContainer = RobotContainer;

    addCommands(

      // First Shot

      new ManualShoot(Arm, 120), // Shoot your preload
      InitialPose("CSLimelight1", false), // Seed your inital position

      // Second Shot

      ChoreoPathing("CSLimelight1", false), // Line up with the center close note

      new ParallelCommandGroup( // Drive toward the center close note until the limelight loses detection
        new ManualIntake(Intake, Arm, 120), // Enables intake
        m_drivetrain.applyRequest(() -> RobotContainer.driveRobotCentric // Robot centric drive command that adjusts off of the lightlight
          .withVelocityX(1)  
          .withVelocityY(-LimelightHelpers.getTX("limelight-ri") / 30) 
          .withRotationalRate(0))
      ).until(() -> -LimelightHelpers.getTX("limelight-ri") / 30 == 0), // Command ends when the note loses detection

      new ParallelCommandGroup( // Go to the speaker to shoot our note
      new ManualIntake(Intake, Arm, 120), 
      ChoreoPathing("CSLimelight2", false)
      ),

      new ManualShoot(Arm, 120), // Shoot the note

      // Thrid Shot

      ChoreoPathing("CSLimelight3", false), // Line up with the center close note

      new ParallelCommandGroup( // Drive toward the center close note until the limelight loses detection
        new ManualIntake(Intake, Arm, 120), // Enables intake
        m_drivetrain.applyRequest(() -> RobotContainer.driveRobotCentric // Robot centric drive command that adjusts off of the lightlight
          .withVelocityX(1)  
          .withVelocityY(-LimelightHelpers.getTX("limelight-ri") / 30) 
          .withRotationalRate(0))
      ).until(() -> -LimelightHelpers.getTX("limelight-ri") / 30 == 0), // Command ends when the note loses detection

      new ParallelCommandGroup( // Go to the speaker to shoot our note
      new ManualIntake(Intake, Arm, 120), 
      ChoreoPathing("CSLimelight4", false)
      ),

      new ManualShoot(Arm, 120), // Shoot the note

      // Fourth Shot

      ChoreoPathing("CSLimelight5", false), // Line up with the center close note

      new ParallelCommandGroup( // Drive toward the center close note until the limelight loses detection
        new ManualIntake(Intake, Arm, 120), // Enables intake
        m_drivetrain.applyRequest(() -> RobotContainer.driveRobotCentric // Robot centric drive command that adjusts off of the lightlight
          .withVelocityX(1)  
          .withVelocityY(-LimelightHelpers.getTX("limelight-ri") / 30) 
          .withRotationalRate(0))
      ).until(() -> -LimelightHelpers.getTX("limelight-ri") / 30 == 0), // Command ends when the note loses detection

      new ParallelCommandGroup( // Go to the speaker to shoot our note
      new ManualIntake(Intake, Arm, 120), 
      ChoreoPathing("CSLimelight6", false)
      ),

      new ManualShoot(Arm, 120) // Shoot the note
      
    );

  }

  private ParallelCommandGroup ChoreoPathingWithIntake(String Trajectory, boolean IsRed) {

    return new ParallelCommandGroup( // Go to the speaker to shoot our note

      new ManualIntake(new Intake(), new Arm(), 120), 
      ChoreoPathing(Trajectory, IsRed)

    );

  }

  private ParallelRaceGroup DriveToNote(String Trajectory, RobotContainer robotContainer) {

    return new ParallelRaceGroup( // Drive toward the center close note until the limelight loses detection
        new ManualIntake(new Intake(), new Arm(), 120), // Enables intake
        m_drivetrain.applyRequest(() -> robotContainer.driveRobotCentric // Robot centric drive command that adjusts off of the lightlight
          .withVelocityX(1)  
          .withVelocityY(-LimelightHelpers.getTX("limelight-ri") / 30) 
          .withRotationalRate(0))
    ).until(() -> -LimelightHelpers.getTX("limelight-ri") / 30 == 0);

  }
 
  private Command ChoreoPathing(String Trajectory, boolean IsRed) {

    return Choreo.choreoSwerveCommand(

      Choreo.getTrajectory(Trajectory), 
      () -> (m_drivetrain.getState().Pose), 
      Constants.AutoDrivePID, Constants.AutoDrivePID, Constants.AutoTurnPID, 
      (ChassisSpeeds speeds) -> m_drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
      () -> IsRed, 
      m_drivetrain

    ).withTimeout(Choreo.getTrajectory(Trajectory).getTotalTime() * 1.1);

  }
 
  private Command InitialPose(String Trajectory, boolean IsRed) {

    return m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(Choreo.getTrajectory(Trajectory).getInitialPose()));

  }

}
