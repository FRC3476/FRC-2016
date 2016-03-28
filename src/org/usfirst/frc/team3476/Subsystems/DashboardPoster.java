package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Communications.Dashcomm;
import org.usfirst.frc.team3476.Main.Subsystem;

import edu.wpi.first.wpilibj.Timer;

public class DashboardPoster implements Subsystem
{

	private SubsystemTask posterTask;
	private Thread posterThread;
	private Subsystem[] systems;

	public DashboardPoster(Subsystem[] systems)
	{
		this.systems = systems;
		
		posterTask = new SubsystemTask(this, 50);
		posterThread = new Thread(posterTask, "watcherThread");
		posterThread.start();
	}

	@Override
	public String[] getAutoCommands()
	{
		return null;
	}

	@Override
	public void doAuto(double[] params, String command){}

	@Override
	public boolean isAutoDone()
	{
		return false;
	}

	@Override
	public String[] getConstantRequest()
	{
		return null;
	}

	@Override
	public void returnConstantRequest(double[] constantsin){}

	@Override
	public void update()
	{
		Shooter shooter = (Shooter)systems[2];
		Dashcomm.put("data/flywheel/tachometer", shooter.getFlySpeed());
		Dashcomm.put("data/flywheel/setpoint", shooter.getFlySet());
		Dashcomm.put("match/time", Timer.getMatchTime());
	}

	@Override
	public void stopThreads()
	{
		posterTask.hold();
	}
	
	@Override
	public void startThreads()
	{
		posterTask.resume();
	}
	
	public void terminateThreads()
	{
		posterTask.terminate();
		try
		{
			posterThread.join();
			System.out.println("Ended " + this + " thread.");
		}
		catch(InterruptedException e)
		{
			System.out.println("Ended " + this + " thread.");
		}
	}
	
	public synchronized void end(){}
	
	@Override
	public boolean threadsActive()
	{
		return posterTask.isActive();
	}
	
	public String toString()
	{
		return "DashboardPoster";
	}

}
