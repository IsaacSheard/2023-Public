// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final LimeLightSubsystem m_limeLightSubsystem = new LimeLightSubsystem();
  // The driver's controller
 // XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_driverController = new Joystick(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getY(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getX(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getZ(), 0.06),
                true),
            m_robotDrive));
  }
  
  public void robotPeriodic() {
    System.out.println("In robotPeriodic");
    Pose3d position = m_limeLightSubsystem.fieldPosition();
    if (position != null) {
        Rotation2d rotation = position.getRotation().toRotation2d();
        SmartDashboard.putNumber("Pos X", position.getX());
        SmartDashboard.putNumber("Pos Y", position.getY());
        SmartDashboard.putNumber("Rotation", rotation.getDegrees());
    } else {
        SmartDashboard.putNumber("Pos X", 0);
        SmartDashboard.putNumber("Pos Y", 0);
        SmartDashboard.putNumber("Rotation", 0);
    }

    long tagId = m_limeLightSubsystem.aprilTagId();
    SmartDashboard.putNumber("AprilTag ID", tagId);
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private SendableChooser<Integer> _autoChooser;

    public Command getAutonomousCommand() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));
        _autoChooser = new SendableChooser<>();
        _autoChooser.addOption("FullAuto", 1);
        _autoChooser.addOption("FullAutoX", 2);
        _autoChooser.addOption("Alt-2 Ball Bottom", 3);
        _autoChooser.addOption("3BallAuto", 4);
        _autoChooser.addOption("3BallAlt", 5);
        SmartDashboard.putData("Autonomous Routine", _autoChooser);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        /*SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            m_robotDrive::getPose, // Pose2d supplier
            m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            m_robotDrive); // The drive subsystem. Used to properly set the requirements of path following commands

        return autoBuilder.fullAuto(pathGroup);*/
        
        return m_robotDrive.followTrajectoryCommand(pathGroup.get(0), true);
    }
}
