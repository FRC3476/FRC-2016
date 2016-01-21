package org.usfirst.frc.team3476.ScriptableAuto;

import java.io.IOException;
import java.util.ArrayList;

import org.usfirst.frc.team3476.Main.*;
import org.usfirst.frc.team3476.Subsystems.StartSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Main
{
	Parser par;
	Subsystem[] systems;
	boolean stop;
	
	public Main(String year, Subsystem[] systemsin)
	{
		par = new Parser(getScript(), getConstants(), year);
		systems = systemsin;
		sendCheckText();
		passConstants();
		stop = false;
	}
	
	//Testing constructor
	public Main(String year, Subsystem[] systemsin, String script, String constants)
	{
		par = new Parser(script, constants, year);
		systems = systemsin;
		passConstants();
	}
	
	public void start()
	{
		reset();
		ArrayList<CommandBlock> curCommands;
		Subsystem current;
		boolean done;
		
		while(par.hasNextLine())
		{
			if(stop)break;
			done = false;
			curCommands = par.nextLine();
			SmartDashboard.putString("command", "" + curCommands);
			while (!done)//Keep going until line is done (ArrayList is empty)
			{
				if(stop)break;
				done = true;
				for (CommandBlock block : curCommands)//Go thru each CommandBlock on this line
				{
					if(stop)break;
					if(block.hasNext())//If there is another command, do things
					{
						current = findSubsystem(block.getCommand());//Grab the subsystem that deals with this command
						if(!block.getCommand().isStarted())//If the command has not been started (new command), start it (duh)
						{							
							current.doAuto(block.getCommand().getParams(), block.getCommand().getName());
							block.getCommand().start();
						}
						else
						{
							if(current.isAutoDone())//If the subsystem is done, remove the command from the queue
							{
								block.finishCommand();
							}
						}
						done = false;
					}
					else{/*No more commands, leave it alone, we are iterating over this ArrayList - throws ConcurrentModificationException*/}
				}
			}
		}
	}
	
	private String getScript()
	{
		return SmartDashboard.getString("java auto text");
	}
	
	private String getConstants()
	{
		return SmartDashboard.getString("java constants");
	}
	
	public void passConstants()
	{
		for(Subsystem current : systems)
		{
			//Return requested constants to the subsystem
			if(current != null)
			{
				String[] request = current.getConstantRequest();
				if(request != null)
				{
					double[] response = new double[request.length];
					try
					{
						for (int i = 0; i < request.length; i++)
						{
							response[i] = par.getConstant(request[i]);
						}
						current.returnConstantRequest(response);
					}
					catch (IOException e)
					{
						for (int i = 0; i < response.length; i++)
							response[i] = 0.0;
						current.returnConstantRequest(response);
						System.out.println("IOEXCEPTION: " + e.getMessage());
					}
				}
				else
				{
					//System.out.println("Subsystem " + current + " gave a null constants request.");
				}
			}
		}
	}
	
	private Subsystem findSubsystem(Command command)
	{
		for(Subsystem toSearch : systems)
		{
			if(toSearch != null)
			{
				for(String searchString : toSearch.getAutoCommands())
				{
					if(searchString.equals(command.getName()))
					{
						return toSearch;
					}
				}
			}
		}
		System.out.println("Command \"" + command.getName() + "\" not found.");

		return new ErrorSystem();
	}
	
	public void sendCheckText()
	{
		SmartDashboard.putString("java check text", par.getScript());
	}
	
	public synchronized void stop(Thread autoThread)
	{
		stop = true;
		try
		{
			autoThread.join();
		}
		catch(InterruptedException e){}
	}
	
	public synchronized void stopSubsystems(Thread autoThread)
	{
		for(Subsystem sys: systems)
		{
			sys.stopThreads();
		}
	}
	
	private synchronized void reset()
	{
		stop = false;
		robotDriveClear();
	}
	
	public synchronized void robotDriveClear()
	{
		System.out.println("robotDriveClear");
		for(Subsystem sys : systems)
		{
			if(sys.toString().toLowerCase().indexOf("drive") != -1)
			{
				StartSubsystem.init(sys);
				return;
			}
		}
		throw new NullPointerException("No drive subsystem found to appease watchdog.");
	}
	
	public void update()
	{
		par.update(getScript(), getConstants());
	}
}
