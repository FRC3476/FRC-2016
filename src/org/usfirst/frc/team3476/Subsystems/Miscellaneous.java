package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Communications.Dashcomm;
import org.usfirst.frc.team3476.Main.Subsystem;
import org.usfirst.frc.team3476.Subsystems.Shooter.LoaderState;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

public class Miscellaneous implements Subsystem
{
	enum ClimberState{LOADED, FIRED, WINCHING, DONE}
	private static final double RESETSPEED = -0.25;
	ClimberState climberState;
	
	private double pressure;
	private Thread miscellaneousThread;
	private SubsystemTask task;
	private AnalogInput pressureSensor;
	private Solenoid hookRelease;
	private SpeedController winch;

	private boolean startClimb;
	private double WINCHTIME, CATCHTIME, WINCHSPEED;
	private String[] constants = {"WINCHTIME", "CATCHTIME", "WINCHSPEED"};

	private Timer hookTimer;
	private Timer winchTimer;
	private boolean reset;

	private boolean climbergo;

	private boolean startWinch;

	private boolean stopWinch;
	private boolean lastwinchback;
	
	public Miscellaneous(AnalogInput pressureSensor, Solenoid hookRelease, SpeedController winch)
	{
		this.pressureSensor = pressureSensor;
		pressure = 0;
		
		this.hookRelease = hookRelease;
		this.winch = winch;
		climberState = ClimberState.LOADED;
		hookTimer = new Timer();
		hookTimer.stop();
		winchTimer = new Timer();
		winchTimer.stop();
		reset = false;
		startClimb = false;
		climbergo = false;
		
		task = new SubsystemTask(this, 25);//x ms minimum exec time
		miscellaneousThread = new Thread(task, "miscellaneousThread");
		miscellaneousThread.start();
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
		return constants;
	}

	@Override
	public void returnConstantRequest(double[] constantsin)
	{
		int i = 0;
		WINCHTIME = constantsin[i];
		i++;
		CATCHTIME = constantsin[i];
		i++;
		WINCHSPEED = constantsin[i];
	}

	@Override
	public void update()
	{
		boolean winchback = Dashcomm.get("data/misc/resetting", false);
		if(climbergo)
		{
			switch(climberState)
			{
				case LOADED:
					if(startClimb)
					{
						climberState = ClimberState.FIRED;
						startClimb = false;
						fireHook();
						//hookTimer.reset();
						//hookTimer.start();
					}
					else
					{
						hookRelease.set(false);
					}
					
					if(winchback)
					{
						climberState = ClimberState.FIRED;
					}
					//reset dat state
					startClimb = false;
					startWinch = false;
					reset = false;
					setWinch(false, winchback);
					break;
					
					
				case FIRED:
					if(startWinch)
					{
						climberState = ClimberState.WINCHING;
						hookTimer.stop();
						winchTimer.reset();
						winchTimer.start();
					}
					
					if(!winchback && lastwinchback)
					{
						climberState = ClimberState.LOADED;
					}
					
					if(!winchback && !lastwinchback)//never was on
					{
						hookRelease.set(true);//reset dat state
					}
					startClimb = false;
					reset = false;
					setWinch(false, winchback);
					stopWinch = false;
					startWinch = false;
					break;
					
					
				case WINCHING:
					setWinch(true, winchback);
					
					if(stopWinch)//stop winching
					{
						climberState = ClimberState.FIRED;
					}
					
					if(!winchback && lastwinchback)
					{
						climberState = ClimberState.LOADED;
					}
					
					if(!winchback && !lastwinchback)//never was on
					{
						hookRelease.set(true);//reset dat state
					}
					
					startClimb = false;
					startWinch = false;
					reset = false;
					stopWinch = false;
					break;
					
					//unused
				case DONE:
					if(reset)
					{
						climberState = ClimberState.LOADED;
					}
					else
					{
						hookRelease.set(true);//reset dat state
						startClimb = false;
						startWinch = false;
						setWinch(false, winchback);
					}
					break;
					//end unused
			}//end state switch
		}//end climbergo
		lastwinchback = winchback;
	}
	
	private synchronized void setWinch(boolean on, boolean winchback)
	{
		if(!on)
		{
			winch.set(0);
		}
		else if(!winchback)
		{
			winch.set(WINCHSPEED);
		}
		else
		{
			winch.set(RESETSPEED);
		}
	}

	private synchronized void fireHook()
	{
		hookRelease.set(true);
	}
	
	public synchronized void advanceClimber()
	{
		if(climberState == ClimberState.LOADED)
		{
			startClimb();
		}
		else if(climberState == ClimberState.FIRED)
		{
			startWinch();
		}
	}
	
	public synchronized void resetClimber()
	{
		reset = true;
	}
	
	public synchronized void startClimb()
	{
		startClimb = true;
	}
	
	public synchronized void startWinch()
	{
		startWinch = true;
	}
	
	public synchronized void stopWinch()
	{
		stopWinch = true;
	}
	
	public synchronized boolean isFired()
	{
		return climberState != ClimberState.LOADED;
	}
	
	public synchronized ClimberState getClimberState()
	{
		return climberState;
	}
	
	public synchronized void climberEnable(boolean enable)
	{
		climbergo = enable;
	}
	
	@Override
	public String toString()
	{
		return "Miscellaneous";
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
			miscellaneousThread.join();
			System.out.println("Ended " + this + " thread.");
		}
		catch(InterruptedException e)
		{
			System.out.println("Ended " + this + " thread.");
		}
	}
	
	public synchronized void end()
	{
		winch.set(0);
	}
	
	@Override
	public boolean threadsActive()
	{
		return task.isActive();
	}
	
	private void calcPressure()
	{
		pressure = (250*pressureSensor.getVoltage()/5 - 25);
	}

	public double getPressure()
	{
		calcPressure();
		return pressure;
	}
	
	public double getPVoltage()
	{
		return pressureSensor.getVoltage();
	}
}
