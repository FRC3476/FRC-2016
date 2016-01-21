package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class Intake implements Subsystem
{
	private double SUCKMOTORSPEED, LOADMOTORSPEED, ddTime;
	final String[] autoCommands = {"intake", "dropdown"};
	final String[] constants = {"SUCKMOTORSPEED", "LOADMOTORSPEED", "FORWARDISDOWN"};
	private boolean done, FORWARDISDOWN, started;
	
	private SpeedController dropdown, escalator;
	private Relay ddmotor;
	public enum DDdir{UP, DOWN, STOP}
	DDdir curDir;
	Timer ddTimer;
	
	private SubsystemTask task;
	private Thread ddThread;
	
	public Intake(SpeedController dropdownin, SpeedController escalatorin, Relay ddmotorin)
	{
		dropdown = dropdownin;
		escalator = escalatorin;
		ddmotor = ddmotorin;
		curDir = DDdir.STOP;
		ddTimer = new Timer();
		ddTime = 0;
		started = true;
		done = true;
		
		task = new SubsystemTask(this);
		ddThread = new Thread(task, "ddThread");
		ddThread.start();
	}
	
	@Override
	public String[] getAutoCommands()
	{
		return autoCommands;
	}

	@Override
	public synchronized void doAuto(double[] params, String command)
	{
		done = false;
		started = false;
		if(command.equalsIgnoreCase("intake"))
		{
			System.out.println("intaking " + params[0] + " at " + params[1] + "percent");
			//Direction(sign(possibly 0)), percent speed, constant to invert if necessary and make timing correct
			dropdown.set(params[0]*params[1]*SUCKMOTORSPEED/100);
			escalator.set(params[0]*params[1]*LOADMOTORSPEED/100);
			done = true;
		}
		else if(command.equalsIgnoreCase("dropdown"))
		{
			switch((int)params[0])
			{
				case 1:
					curDir = DDdir.UP;
					break;
				case 0:
					curDir = DDdir.STOP;
					setIntakeMovement(curDir);
					done = true;
					started = true;
					break;
				case -1:
					curDir = DDdir.DOWN;
					break;
			}
			ddTime = params[1];
		}
	}

	@Override
	public synchronized boolean isAutoDone()
	{
		return done;
	}

	@Override
	public String[] getConstantRequest()
	{
		return constants;
	}

	@Override
	public synchronized void returnConstantRequest(double[] constantsin)
	{
		SUCKMOTORSPEED = constantsin[0];
		LOADMOTORSPEED = constantsin[1];
		FORWARDISDOWN = constantsin[2] == 1;
		startThreads();
	}

	@Override
	public synchronized void update()
	{
		if(!started && !done)
		{
			ddTimer.reset();
			ddTimer.start();
			setIntakeMovement(curDir);
			started = true;
		}
		else if(started && !done)
		{
			done = ddTimer.hasPeriodPassed(ddTime);
			if(done)
			{
				ddTimer.stop();
				setIntakeMovement(DDdir.STOP);
			}
		}
	}
	
	public void setIntakeMovement(DDdir dir)
	{
		Value forward = Relay.Value.kForward;
		Value reverse = Relay.Value.kReverse;
		
		switch(dir)
		{
			case UP:
				ddmotor.set(FORWARDISDOWN ? reverse : forward);
				break;
			case DOWN:
				ddmotor.set(FORWARDISDOWN ? forward : reverse);
				break;
			case STOP:
				ddmotor.set(Relay.Value.kOff);
				break;
		}
	}
	
	public String toString()
	{
		return "Intake";
	}
	
	@Override
	public void stopThreads()
	{
		task.hold();
	}
	
	@Override
	public void startThreads()
	{
		task.resume();
	}
	
	public void terminateThreads()
	{
		task.terminate();
		try
		{
			ddThread.join();
			System.out.println("Ended " + this + " thread.");
		}
		catch(InterruptedException e)
		{
			System.out.println("Ended " + this + " thread.");
		}
	}

	@Override
	public void end()
	{
		setIntakeMovement(DDdir.STOP);
		dropdown.set(0);
		escalator.set(0);
	}
}
