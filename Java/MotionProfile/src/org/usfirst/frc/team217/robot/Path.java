package org.usfirst.frc.team217.robot;

import jaci.pathfinder.Trajectory;

public abstract class Path {
	public abstract Trajectory getLeftTrajectory();
	public abstract Trajectory getRightTrajectory();
	public abstract double[][] getLeftArray();
	public abstract double[][] getRightArray();
}
