/**
 * This Java FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon SRX.  The CANTalon class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon SRX.
 * 
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 * 
 * This application is an IterativeRobot project to demonstrate a minimal implementation not requiring the command 
 * framework, however these code excerpts could be moved into a command-based project.
 * 
 * The project also includes instrumentation.java which simply has debug printfs, and a MotionProfile.java which is generated
 * in @link https://docs.google.com/spreadsheets/d/1PgT10EeQiR92LNXEOEe3VGn737P7WDP4t0CQxQgC8k0/edit#gid=1813770630&vpid=A1
 * or find Motion Profile Generator.xlsx in the Project folder.
 * 
 * Logitech Gamepad mapping, use left y axis to drive Talon normally.  
 * Press and hold top-left-shoulder-button5 to put Talon into motion profile control mode.
 * This will start sending Motion Profile to Talon while Talon is neutral. 
 * 
 * While holding top-left-shoulder-button5, tap top-right-shoulder-button6.
 * This will signal Talon to fire MP.  When MP is done, Talon will "hold" the last setpoint position
 * and wait for another button6 press to fire again.
 * 
 * Release button5 to allow PercentOutput control with left y axis.
 */

package org.usfirst.frc.team217.robot;

import com.ctre.phoenix.motorcontrol.can.*;

import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	/** The Talon we want to motion profile. */
	TalonSRX leftTalonMaster = new TalonSRX(2);
	//TalonSRX leftTalonSlave1 = new TalonSRX(11);
	//TalonSRX leftTalonSlave2 = new TalonSRX(12);
	
	TalonSRX rightTalonMaster = new TalonSRX(1);
	//TalonSRX rightTalonSlave1 = new TalonSRX(9);
	//TalonSRX rightTalonSlave2 = new TalonSRX(10);
	
	public Path midSwitchLeft = new MidSwitchLeft();

	/** some example logic on how one can manage an MP */
	MotionProfileRunner _example = new MotionProfileRunner(leftTalonMaster, rightTalonMaster, midSwitchLeft.getLeftArray(), midSwitchLeft.getRightArray());

	/** joystick for testing */
	Joystick _joy = new Joystick(0);

	/**
	 * cache last buttons so we can detect press events. In a command-based
	 * project you can leverage the on-press event but for this simple example,
	 * lets just do quick compares to prev-btn-states
	 */
	boolean[] _btnsLast = {false, false, false, false, false, false, false, false, false, false};

	/** run once after booting/enter-disable */
	public void disabledInit() {

		leftTalonMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		leftTalonMaster.setSensorPhase(false); /* keep sensor and motor in phase */
		leftTalonMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

		leftTalonMaster.config_kF(0, 0.076, Constants.kTimeoutMs); // 0.255
		leftTalonMaster.config_kP(0, 0.5, Constants.kTimeoutMs); // 5
		leftTalonMaster.config_kI(0, 0.0, Constants.kTimeoutMs); // 0
		leftTalonMaster.config_kD(0, 20.0, Constants.kTimeoutMs); // 1

		/* Our profile uses 10ms timing */
		leftTalonMaster.configMotionProfileTrajectoryPeriod(0, Constants.kTimeoutMs); 
		/*
		 * status 10 provides the trajectory target for motion profile AND
		 * motion magic
		 */
		leftTalonMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		
		leftTalonMaster.setInverted(true);
		//leftTalonSlave1.setInverted(true);
		//leftTalonSlave2.setInverted(true);
		
		//leftTalonSlave1.set(ControlMode.Follower, 7);
		//leftTalonSlave2.set(ControlMode.Follower, 7);
		
		rightTalonMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
		rightTalonMaster.setSensorPhase(false); /* keep sensor and motor in phase */
		rightTalonMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

		rightTalonMaster.config_kF(0, 0.076, Constants.kTimeoutMs);
		rightTalonMaster.config_kP(0, 0.5, Constants.kTimeoutMs);
		rightTalonMaster.config_kI(0, 0.0, Constants.kTimeoutMs);
		rightTalonMaster.config_kD(0, 20.0, Constants.kTimeoutMs);

		/* Our profile uses 10ms timing */
		rightTalonMaster.configMotionProfileTrajectoryPeriod(0, Constants.kTimeoutMs); 
		/*
		 * status 10 provides the trajectory target for motion profile AND
		 * motion magic
		 */
		rightTalonMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		
		//rightTalonSlave1.set(ControlMode.Follower, 8);
		//rightTalonSlave2.set(ControlMode.Follower, 8);
	}

	/** function is called periodically during operator control */
	public void teleopPeriodic() {
		
		/* get buttons */
		boolean[] btns = new boolean[_btnsLast.length];
		for (int i = 1; i < _btnsLast.length; ++i)
			btns[i] = _joy.getRawButton(i);

		/* get the left joystick axis on Logitech Gampead */
		double leftYjoystick = -1 * _joy.getY(); /* multiple by -1 so joystick forward is positive */
		double rightYjoystick = -1 * _joy.getRawAxis(5); /* multiple by -1 so joystick forward is positive */
		double signedLeft = leftYjoystick;
		double signedRight = rightYjoystick;

		/*
		 * call this periodically, and catch the output. Only apply it if user
		 * wants to run MP. */
		_example.control();

		/* Check button 5 (top left shoulder on the logitech gamead). */
		if (btns[5] == false) {
			/* 
			 * If it's not being pressed, just do a simple drive. This could be
			 * a RobotDrive class or custom drivetrain logic. The point is we
			 * want the switch in and out of MP Control mode.
			 */

			/* button5 is off so straight drive */
			leftTalonMaster.set(ControlMode.PercentOutput, leftYjoystick);
			System.out.println(leftYjoystick);
			rightTalonMaster.set(ControlMode.PercentOutput, rightYjoystick);
			System.out.println(rightYjoystick);

			_example.reset();
		} else {
			/*
			 * Button5 is held down so switch to motion profile control mode =>
			 * This is done in MotionProfileControl. When we transition from
			 * no-press to press, pass a "true" once to MotionProfileControl.
			 */

			SetValueMotionProfile setOutput = _example.getSetValue();

			leftTalonMaster.set(ControlMode.MotionProfile, setOutput.value);
			rightTalonMaster.set(ControlMode.MotionProfile, setOutput.value);

			/*
			 * if btn is pressed and was not pressed last time, In other words
			 * we just detected the on-press event. This will signal the robot
			 * to start a MP
			 */
			if ((btns[6] == true) && (_btnsLast[6] == false)) {
				/* user just tapped button 6 */

				// --- We could start an MP if MP isn't already running ----//
				_example.startMotionProfile();
			}
		}

		/* save buttons states for on-press detection */
		for (int i = 1; i < 10; ++i)
			_btnsLast[i] = btns[i];

		
		SmartDashboard.putNumber("Left Speed", leftTalonMaster.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Right Speed", rightTalonMaster.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Left Pos", leftTalonMaster.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Right Pos", rightTalonMaster.getSelectedSensorPosition(0));
		
	}

	/** function is called periodically during disable */
	public void disabledPeriodic() {
		/*
		 * it's generally a good idea to put motor controllers back into a known
		 * state when robot is disabled. That way when you enable the robot
		 * doesn't just continue doing what it was doing before. BUT if that's
		 * what the application/testing requires than modify this accordingly
		 */
		leftTalonMaster.set(ControlMode.PercentOutput, 0);
		rightTalonMaster.set(ControlMode.PercentOutput, 0);
		
		SmartDashboard.putNumber("Left Speed", leftTalonMaster.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Right Speed", rightTalonMaster.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Left Pos", leftTalonMaster.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Right Pos", rightTalonMaster.getSelectedSensorPosition(0));
		
		/* clear our buffer and put everything into a known state */
		_example.reset();
		
		
	}
}
