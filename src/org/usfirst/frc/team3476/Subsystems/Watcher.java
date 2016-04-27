package org.usfirst.frc.team3476.Subsystems;

import java.util.Arrays;

import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Utility.OrangeUtility;

import edu.wpi.first.wpilibj.CANTalon;

public class Watcher implements Subsystem
{
	Subsystem[] systems;
	Thread watcherThread;
	SubsystemTask watcherTask;
	boolean watch;
	
	public Watcher(Subsystem[] systemsin, int exectime)
	{
		systems = systemsin;
		
		watch = true;
		watcherTask = new SubsystemTask(this, exectime);
		watcherThread = new Thread(watcherTask, "watcherThread");
		watcherThread.start();
	}
	
	@Override
	public String[] getAutoCommands(){return null;}

	@Override
	public void doAuto(double[] params, String command){}

	@Override
	public boolean isAutoDone(){return true;}

	@Override
	public String[] getConstantRequest(){return null;}

	@Override
	public void returnConstantRequest(double[] constantsin){}

	@Override
	public void update()
	{
		if(watch)
		{
			if(false)
			{
				CANTalon turret = ((Turret)systems[1]).getTurretMotor();
				System.out.println("turretMotor: " + turret.get());
			}
			if(false)
			{
				Shooter shooter = (Shooter)systems[2];
				System.out.println("Shooter setpoint: " + shooter.getFlySet());
			}
			if(true)
			{
				Shooter shooter = (Shooter)systems[2];
				double[] get = shooter.getMotorGroup().get();
				double attempt = shooter.getShooterPID().get(),
						actual = 0;
				for(double d : get)
				{
					actual += d;
				}
				actual /= get.length;
				if(!OrangeUtility.doubleEqual(attempt, actual, 1E-2))
				{
					System.out.println("ERROR Flyatt: " + attempt + " Flyact: " + actual);
				}
			}
			if(true)
			{
				Intake intake = (Intake)systems[3];
				double actual = intake.getIntake(),
						attempt = intake.getLastIntakeVal();
				if(!OrangeUtility.doubleEqual(attempt, actual, 1E-2))
				{
					System.out.println("ERROR Intatt: " + attempt + " Intact: " + actual);
				}
			}
		}
	}

	@Override
	public void stopThreads()
	{
		watcherTask.hold();
	}

	@Override
	public void startThreads()
	{
		watcherTask.resume();
	}

	@Override
	public boolean threadsActive()
	{
		return watcherTask.isActive();
	}

	@Override
	public void terminateThreads()
	{
		watcherTask.terminate();
	}

	@Override
	public void end(){}
	
	public String toString()
	{
		return "Watcher";
	}
	
	public void watch(boolean wat)
	{
		watch = wat;
	}
}
