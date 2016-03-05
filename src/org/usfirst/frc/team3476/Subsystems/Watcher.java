package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;

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
			CANTalon turret = ((Turret)systems[1]).getTurretMotor();
			System.out.println("turretMotor: " + turret.get());
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
