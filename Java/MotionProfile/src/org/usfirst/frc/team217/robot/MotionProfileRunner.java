/**
 * Example logic for firing and managing motion profiles.
 * This example sends MPs, waits for them to finish
 * Although this code uses a CANTalon, nowhere in this module do we changeMode() or call set() to change the output.
 * This is done in Robot.java to demonstrate how to change control modes on the fly.
 * 
 * The only routines we call on Talon are....
 * 
 * changeMotionControlFramePeriod
 * 
 * getMotionProfileStatus		
 * clearMotionProfileHasUnderrun     to get status and potentially clear the error flag.
 * 
 * pushMotionProfileTrajectory
 * clearMotionProfileTrajectories
 * processMotionProfileBuffer,   to push/clear, and process the trajectory points.
 * 
 * getControlMode, to check if we are in Motion Profile Control mode.
 * 
 * Example of advanced features not demonstrated here...
 * [1] Calling pushMotionProfileTrajectory() continuously while the Talon executes the motion profile, thereby keeping it going indefinitely.
 * [2] Instead of setting the sensor position to zero at the start of each MP, the program could offset the MP's position based on current position. 
 */
package org.usfirst.frc.team217.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import com.ctre.phoenix.motion.*;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;

public class MotionProfileRunner {
	
	public static final double WHEEL_DIAMETER = 3.5; // inches
	public static final int UNITS_PER_REVOLUTION = 4096; // encoder ticks
	
	/**
	 * The status of the motion profile executer and buffer inside the Talon.
	 * Instead of creating a new one every time we call getMotionProfileStatus,
	 * keep one copy.
	 */
	private MotionProfileStatus leftStatus = new MotionProfileStatus();
	private MotionProfileStatus rightStatus = new MotionProfileStatus();
	
	/** Arrays to hold the trajectories for each side. */
	private double[][] leftArray;
	private double[][] rightArray;

	/** Additional cache for holding the active trajectory points */
	double leftPos = 0, leftVel = 0, leftHeading = 0, rightPos = 0, rightVel = 0, rightHeading = 0;

	/**
	 * Reference to the talon we plan on manipulating. We will not changeMode()
	 * or call set(), just get motion profile status and make decisions based on
	 * motion profile.
	 */
	private TalonSRX leftTalon;
	private TalonSRX rightTalon;
	
	/**
	 * State machine to make sure we let enough of the motion profile stream to
	 * talon before we fire it.
	 */
	private int state = 0;
	/**
	 * Any time you have a state machine that waits for external events, its a
	 * good idea to add a timeout. Set to -1 to disable. Set to nonzero to count
	 * down to '0' which will print an error message. Counting loops is not a
	 * very accurate method of tracking timeout, but this is just conservative
	 * timeout. Getting time-stamps would certainly work too, this is just
	 * simple (no need to worry about timer overflows).
	 */
	private int loopTimeout = -1;
	/**
	 * If start() gets called, this flag is set and in the control() we will
	 * service it.
	 */
	private boolean start = false;

	/**
	 * Since the CANTalon.set() routine is mode specific, deduce what we want
	 * the set value to be and let the calling module apply it whenever we
	 * decide to switch to MP mode.
	 */
	private SetValueMotionProfile setValue = SetValueMotionProfile.Disable;
	/**
	 * How many trajectory points do we wait for before firing the motion
	 * profile.
	 */
	private static final int kMinPointsInTalon = 50;
	/**
	 * Just a state timeout to make sure we don't get stuck anywhere. Each loop
	 * is about 20ms.
	 */
	private static final int kNumLoopsTimeout = 10;
	
	/**
	 * Lets create a periodic task to funnel our trajectory points into our talon.
	 * It doesn't need to be very accurate, just needs to keep pace with the motion
	 * profiler executer.  Now if you're trajectory points are slow, there is no need
	 * to do this, just call _talon.processMotionProfileBuffer() in your teleop loop.
	 * Generally speaking you want to call it at least twice as fast as the duration
	 * of your trajectory points.  So if they are firing every 20ms, you should call 
	 * every 10ms.
	 */
	class PeriodicRunnable implements java.lang.Runnable {
	    public void run() {
	    	leftTalon.processMotionProfileBuffer();
	    	rightTalon.processMotionProfileBuffer();
	    }
	}
	Notifier notifer = new Notifier(new PeriodicRunnable());
	
	/**
	 * C'tor
	 * 
	 * @param talon
	 *            reference to Talon object to fetch motion profile status from.
	 */
	public MotionProfileRunner(TalonSRX leftTalon, TalonSRX rightTalon, double[][] leftArray, double[][] rightArray) {
		this.leftArray = leftArray;
		this.rightArray = rightArray;
		this.leftTalon = leftTalon;
		this.rightTalon = rightTalon;
		/*
		 * since our MP is 10ms per point, set the control frame rate and the
		 * notifer to half that
		 */
		leftTalon.changeMotionControlFramePeriod(1);
		rightTalon.changeMotionControlFramePeriod(1);
		notifer.startPeriodic(0.001);
	}

	/**
	 * Called to clear Motion profile buffer and reset state info during
	 * disabled and when Talon is not in MP control mode.
	 */
	public void reset() {
		/*
		 * Let's clear the buffer just in case user decided to disable in the
		 * middle of an MP, and now we have the second half of a profile just
		 * sitting in memory.
		 */
		leftTalon.clearMotionProfileTrajectories();
		rightTalon.clearMotionProfileTrajectories();
		/* When we do re-enter motionProfile control mode, stay disabled. */
		setValue = SetValueMotionProfile.Disable;
		/* When we do start running our state machine start at the beginning. */
		state = 0;
		loopTimeout = -1;
		/*
		 * If application wanted to start an MP before, ignore and wait for next
		 * button press
		 */
		start = false;
	}

	/**
	 * Called every loop.
	 */
	public void control() {
		/* Get the motion profile status every loop */
		leftTalon.getMotionProfileStatus(leftStatus);
		rightTalon.getMotionProfileStatus(rightStatus);

		/*
		 * track time, this is rudimentary but that's okay, we just want to make
		 * sure things never get stuck.
		 */
		if (loopTimeout < 0) {
			/* do nothing, timeout is disabled */
		} else {
			/* our timeout is nonzero */
			if (loopTimeout == 0) {
				/*
				 * something is wrong. Talon is not present, unplugged, breaker
				 * tripped
				 */
				Instrumentation.OnNoProgress();
			} else {
				--loopTimeout;
			}
		}

		/* first check if we are in MP mode */
		if (leftTalon.getControlMode() != ControlMode.MotionProfile
				|| rightTalon.getControlMode() != ControlMode.MotionProfile) {
			/*
			 * we are not in MP mode. We are probably driving the robot around
			 * using gamepads or some other mode.
			 */
			state = 0;
			loopTimeout = -1;
		} else {
			/*
			 * we are in MP control mode. That means: starting Mps, checking Mp
			 * progress, and possibly interrupting MPs if thats what you want to
			 * do.
			 */
			switch (state) {
				case 0: /* wait for application to tell us to start an MP */
					if (start) {
						start = false;
	
						setValue = SetValueMotionProfile.Disable;
						startFilling();
						/*
						 * MP is being sent to CAN bus, wait a small amount of time
						 */
						state = 1;
						loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 1: /*
						 * wait for MP to stream to Talon, really just the first few
						 * points
						 */
					/* do we have a minimum numberof points in Talon */
					if (leftStatus.btmBufferCnt > kMinPointsInTalon
							&& rightStatus.btmBufferCnt > kMinPointsInTalon) {
						/* start (once) the motion profile */
						setValue = SetValueMotionProfile.Enable;
						/* MP will start once the control frame gets scheduled */
						state = 2;
						loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 2: /* check the status of the MP */
					/*
					 * if talon is reporting things are good, keep adding to our
					 * timeout. Really this is so that you can unplug your talon in
					 * the middle of an MP and react to it.
					 */
					if (leftStatus.isUnderrun == false && rightStatus.isUnderrun == false) {
						loopTimeout = kNumLoopsTimeout;
					}
					/*
					 * If we are executing an MP and the MP finished, start loading
					 * another. We will go into hold state so robot servo's
					 * position.
					 */
					if (leftStatus.activePointValid && leftStatus.isLast
							&& rightStatus.activePointValid && rightStatus.isLast) {
						/*
						 * because we set the last point's isLast to true, we will
						 * get here when the MP is done
						 */
						setValue = SetValueMotionProfile.Hold;
						state = 0;
						loopTimeout = -1;
					}
					break;
			}

			/* Get the motion profile status every loop */
			leftTalon.getMotionProfileStatus(leftStatus);
			leftHeading = leftTalon.getActiveTrajectoryHeading();
			leftPos = leftTalon.getActiveTrajectoryPosition();
			leftVel = leftTalon.getActiveTrajectoryVelocity();
			
			rightTalon.getMotionProfileStatus(rightStatus);
			rightHeading = rightTalon.getActiveTrajectoryHeading();
			rightPos = rightTalon.getActiveTrajectoryPosition();
			rightVel = rightTalon.getActiveTrajectoryVelocity();

			/* printfs and/or logging */
			// TODO: Rewrite Instrumentation for both talons? Right now only left.
			Instrumentation.process(leftStatus, leftPos, leftVel, leftHeading);
		}
	}
	/**
	 * Find enum value if supported.
	 * @param durationMs
	 * @return enum equivalent of durationMs
	 */
	private TrajectoryDuration GetTrajectoryDuration(int durationMs)
	{	 
		/* create return value */
		TrajectoryDuration retval = TrajectoryDuration.Trajectory_Duration_0ms;
		/* convert duration to supported type */
		retval = retval.valueOf(durationMs);
		/* check that it is valid */
		if (retval.value != durationMs) {
			DriverStation.reportError("Trajectory Duration not supported - use configMotionProfileTrajectoryPeriod instead", false);		
		}
		/* pass to caller */
		return retval;
	}
	/** Start filling the MPs to all of the involved Talons. */
	private void startFilling() {
		/* since this example only has one talon, just update that one */
		startFilling(leftArray, rightArray, leftArray.length);
	}

	private void startFilling(double[][] leftProfile, double[][] rightProfile, int totalCnt) {

		/* create an empty point */
		TrajectoryPoint leftPoint = new TrajectoryPoint();
		TrajectoryPoint rightPoint = new TrajectoryPoint();

		/* did we get an underrun condition since last time we checked ? */
		if (leftStatus.hasUnderrun) {
			/* better log it so we know about it */
			Instrumentation.OnUnderrun();
			/*
			 * clear the error. This flag does not auto clear, this way 
			 * we never miss logging it.
			 */
			leftTalon.clearMotionProfileHasUnderrun(0);
		}
		/*
		 * just in case we are interrupting another MP and there is still buffer
		 * points in memory, clear it.
		 */
		leftTalon.clearMotionProfileTrajectories();
		rightTalon.clearMotionProfileTrajectories();

		/* set the base trajectory period to zero, use the individual trajectory period below */
		leftTalon.configMotionProfileTrajectoryPeriod(Constants.kBaseTrajPeriodMs, Constants.kTimeoutMs);
		rightTalon.configMotionProfileTrajectoryPeriod(Constants.kBaseTrajPeriodMs, Constants.kTimeoutMs);
		
		/* This is fast since it's just into our TOP buffer */
		for (int i = 0; i < totalCnt; ++i) {
			double leftPositionRaw = leftProfile[i][0]; // ft
			double leftVelocityRaw = leftProfile[i][1]; // ft/sec
			/* for each point, fill our structure and pass it to API */
			
			leftPoint.position = ft2Units(leftPositionRaw);
			leftPoint.velocity = fps2UnitsPerRev(leftVelocityRaw);
			leftPoint.headingDeg = 0; /* future feature - not used in this example*/
			leftPoint.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			leftPoint.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			leftPoint.timeDur = GetTrajectoryDuration((int)(leftProfile[i][2] * 1000));
			leftPoint.zeroPos = false;
			if (i == 0)
				leftPoint.zeroPos = true; /* set this to true on the first point */

			leftPoint.isLastPoint = false;
			if ((i + 1) == totalCnt)
				leftPoint.isLastPoint = true; /* set this to true on the last point  */

			leftTalon.pushMotionProfileTrajectory(leftPoint);
			
			double rightPositionRaw = rightProfile[i][0]; // ft
			double rightVelocityRaw = rightProfile[i][1]; // ft/sec
			/* for each point, fill our structure and pass it to API */
			
			rightPoint.position = ft2Units(rightPositionRaw);
			rightPoint.velocity = fps2UnitsPerRev(rightVelocityRaw);
			rightPoint.headingDeg = 0; /* future feature - not used in this example*/
			rightPoint.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			rightPoint.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			rightPoint.timeDur = GetTrajectoryDuration((int)rightProfile[i][2]);
			rightPoint.zeroPos = false;
			if (i == 0)
				rightPoint.zeroPos = true; /* set this to true on the first point */

			rightPoint.isLastPoint = false;
			if ((i + 1) == totalCnt)
				rightPoint.isLastPoint = true; /* set this to true on the last point  */

			rightTalon.pushMotionProfileTrajectory(rightPoint);
		}
	}
	/**
	 * Called by application to signal Talon to start the buffered MP (when it's
	 * able to).
	 */
	void startMotionProfile() {
		start = true;
	}

	/**
	 * 
	 * @return the output value to pass to Talon's set() routine. 0 for disable
	 *         motion-profile output, 1 for enable motion-profile, 2 for hold
	 *         current motion profile trajectory point.
	 */
	SetValueMotionProfile getSetValue() {
		return setValue;
	}
	
	/**
	 * Converts feet to encoder units.
	 * Uses {@value #WHEEL_DIAMETER}" for wheel diameter and {@value #UNITS_PER_REVOLUTION} for encoder units per revolution.
	 * @param feet
	 * @return encoder units
	 */
	double ft2Units(double feet) {
		feet *= 12; // inches
		feet /= WHEEL_DIAMETER * Math.PI; // revolutions
		feet *= UNITS_PER_REVOLUTION; // Units
		return feet;
	}
	
	/**
	 * Converts feet per second to encoder units per 100 milliseconds.
	 * Uses {@value #WHEEL_DIAMETER}" for wheel diameter and {@value #UNITS_PER_REVOLUTION} for encoder units per revolution.
	 * @param fps feet per second
	 * @return encoder units per 100 milliseconds
	 */
	double fps2UnitsPerRev(double fps) {
		fps /= 10; // ft/100ms
		fps *= 12; // in/100ms
		fps /= WHEEL_DIAMETER * Math.PI; // revolutions/100ms
		fps *= UNITS_PER_REVOLUTION; // Units/100ms
		return fps;
	}
}
