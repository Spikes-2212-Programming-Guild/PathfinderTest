/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2212.robot;

import java.io.File;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.spikes2212.dashboard.DashBoardController;
import com.spikes2212.genericsubsystems.drivetrains.TankDrivetrain;
import com.spikes2212.utils.InvertedConsumer;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static OI m_oi;
	public static double leftCalculate, rightCalculate; 
	public static WPI_TalonSRX right_talon = new WPI_TalonSRX(RobotMap.CAN.DRIVETRAIN_MOTOR_RIGHT1)
	, left_talon = new WPI_TalonSRX(RobotMap.CAN.DRIVETRAIN_MOTOR_LEFT1);
	public static WPI_VictorSPX right_victor = new WPI_VictorSPX(RobotMap.CAN.DRIVETRAIN_MOTOR_RIGHT2)
	, left_victor = new WPI_VictorSPX(RobotMap.CAN.DRIVETRAIN_MOTOR_LEFT2);
	public static final int TICKS_PER_REVOLUTION = 360, maxSpeed = 1, maxAcc = 2, maxJerk = 60;
	public static final double kp = 0.8, ki = 0.0, kd = 0.0, kv = 0.5, ka = 0.0, time = 0.02
	, WHEEL_DIAMETER = 6 * 0.0254, ROBOT_WIDTH = 0.67, gyro_kp = 0;
	public static Waypoint start = new Waypoint(0,0,0);
	public static Waypoint middle = new Waypoint(1.8,0,0);
	public static Waypoint end = new Waypoint(0,0,0);
	public static Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, time, maxSpeed, maxAcc, maxJerk);
	public static Trajectory traj = Pathfinder.generate(new Waypoint[] {start, middle, end}, config);
	public static TankModifier modifier = new TankModifier(traj);
	public static Trajectory left, right;
	public static EncoderFollower folLeft, folRight;
	public static TankDrivetrain drivetrain = new TankDrivetrain(left_talon::set, new InvertedConsumer(right_talon::set));
	public static Encoder leftEncoder = new Encoder(RobotMap.DIO.LEFT_ENCODER_0, RobotMap.DIO.LEFT_ENCODER_1);
	public static Encoder rightEncoder = new Encoder(RobotMap.DIO.RIGHT_ENCODER_0, RobotMap.DIO.RIGHT_ENCODER_1);
	public static DashBoardController dbc = new DashBoardController();
	public static ADXRS450_Gyro gyro;  
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		gyro = new ADXRS450_Gyro();
		left_victor.follow(left_talon);
		right_victor.follow(right_talon);
		m_oi = new OI();
		modifier.modify(ROBOT_WIDTH);
		leftEncoder.setDistancePerPulse(WHEEL_DIAMETER * Math.PI / TICKS_PER_REVOLUTION);
		rightEncoder.setDistancePerPulse(WHEEL_DIAMETER * Math.PI / TICKS_PER_REVOLUTION);
		left = modifier.getLeftTrajectory();
		right = modifier.getRightTrajectory();
		folLeft = new EncoderFollower(left);
		folRight = new EncoderFollower(right);
		folLeft.configurePIDVA(kp, ki, kd, kv, ka);
		folRight.configurePIDVA(kp, ki, kd, kv, ka);
		dbc.addNumber("leftEncoder", () -> ((double)leftEncoder.get()));
		dbc.addNumber("rightEncoder", () -> ((double)rightEncoder.get()));
		dbc.addNumber("left output", () -> (leftCalculate));
		dbc.addNumber("right output", () -> (rightCalculate));
		dbc.addNumber("gyro angle", gyro::getAngle);
		dbc.addNumber("gyro turn", () -> turn);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		File trajectoryFile = new File("/home/lvuser/Trajectory.csv");
		Pathfinder.writeToCSV(trajectoryFile, traj);
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		leftEncoder.reset();
		rightEncoder.reset();
		gyro.reset();
		folLeft.reset();
		folRight.reset();
		folLeft.configureEncoder(leftEncoder.get(), TICKS_PER_REVOLUTION, WHEEL_DIAMETER);
		folRight.configureEncoder(rightEncoder.get(), TICKS_PER_REVOLUTION, WHEEL_DIAMETER);
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		}

	/**
	 * This function is called periodically during autonomous.
	 */
	double turn = 0;
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		dbc.update();
		leftCalculate = folLeft.calculate(leftEncoder.get());
		rightCalculate = folRight.calculate(rightEncoder.get());
		double angleError = Pathfinder.boundHalfDegrees(gyro.getAngle() - Pathfinder.r2d(folLeft.getHeading()));
		angleError = angleError % 360;
		if (Math.abs(angleError) > 180) angleError = (angleError > 0) ? angleError - 360 : angleError + 360;
		turn = gyro_kp * angleError;
		drivetrain.tankDrive(leftCalculate + turn, rightCalculate - turn);
	}

	@Override
	public void teleopInit() {
		dbc.update();
			gyro.reset();
		}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
