package org.usfirst.frc.team3476.Utility.Control;

import edu.wpi.first.wpilibj.interfaces.Gyro;

public interface DifferentialGyro extends Gyro
{
	public double calcDiff();
	
	public void resetDiff();
	
	public double get();
}
