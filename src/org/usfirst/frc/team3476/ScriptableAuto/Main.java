package org.usfirst.frc.team3476.ScriptableAuto;

import java.io.IOException;
import java.util.ArrayList;

import org.usfirst.frc.team3476.Main.*;
import org.usfirst.frc.team3476.Subsystems.StartSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The main autonomous class.
 * @author Anthony Demetrescu
 *
 */
public class Main
{
	Parser par;
	Subsystem[] systems;
	boolean stop;
	
	/**
	 * Constructs a auto Main object with the given year for constants, and the list of subsystems on the robot.
	 * @param year the constants identifier
	 * @param systemsin the array of robot systems on this robot
	 */
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
	
	/**
	 * Starts autonomous.
	 */
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
				for(CommandBlock block : curCommands)//Go thru each CommandBlock on this line
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
	
	/**
	 * Returns the driver selected autonomous script.
	 * @return the String representation of the script
	 */
	private String getScript()
	{
		return SmartDashboard.getString("java auto text");
	}
	
	/**
	 * Returns the constants files String.
	 * @return the String representation of the constants files
	 */
	private String getConstants()
	{
		return SmartDashboard.getString("java constants");
	}
	
	/**
	 * Passes constants to all subsystems.
	 */
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
	
	/**
	 * Finds the first subsystem that the input command can apply to.
	 * @param command the command to search for
	 * @return the subsystem that can execute the command
	 */
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
	
	/**
	 * Sends the echo of the script back to the dashboard so that communications can be verified.
	 */
	public void sendCheckText()
	{
		SmartDashboard.putString("java check text", par.getScript());
	}
	
	/**
	 * Safely stops a subsystem Thread through the use of a gateway variable.
	 * @param autoThread the Thread to stop
	 */
	public synchronized void stop(Thread autoThread)
	{
		stop = true;
		try
		{
			autoThread.join();
		}
		catch(InterruptedException e){}
	}
	
	/**
	 * Calls stopThreads() on every subsystem to halt execution.
	 */
	public synchronized void stopSubsystems()
	{
		for(Subsystem sys: systems)
		{
			sys.stopThreads();
		}
	}
	
	/**
	 * Method used to ensure a certain state when starting auto.
	 * Clears the robot drive.
	 */
	private synchronized void reset()
	{
		stop = false;
		robotDriveClear();
	}
	
	/**
	 * Finds the drive subsystem and sets the drive values to zero to appease watchdog when auto is not running.
	 */
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
	
	/**
	 * Updates the script and constants for the Parser so that it can parse the latest selected autonomous and get the latest constants.
	 */
	public void update()
	{
		par.update(getScript(), getConstants());
	}
}