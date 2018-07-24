package aima.gui.applications.updateguivacuum;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;

import aima.core.agent.Agent;
import aima.core.agent.EnvironmentObject;
import aima.core.agent.EnvironmentState;
import aima.core.agent.impl.AbstractAgent;
//import aima.core.environment.vacuum.ModelBasedReflexVacuumAgent;
//import aima.core.environment.vacuum.ReflexVacuumAgent;
//import aima.core.environment.vacuum.SimpleReflexVacuumAgent;
//import aima.core.environment.vacuum.TableDrivenVacuumAgent;
//import aima.core.environment.vacuum.VacuumEnvironment;
import aima.core.environment.updatevacuum.*;
import aima.core.environment.xyenv.*;
import aima.core.util.datastructure.XYLocation;
import aima.core.util.datastructure.XYLocation.Direction;
import aima.gui.framework.AgentAppController;
import aima.gui.framework.AgentAppFrame;
import aima.gui.framework.AgentAppModel;
import aima.gui.framework.AgentView;
import aima.gui.framework.SimpleAgentApp;
import sv_code.MyVacuumAgent;

/**
 * Simple graphical application for experiments with vacuum cleaner agents. It
 * can be used as a template for creating other graphical agent applications.
 * 
 * @author R. Lunde
 */
public class UpdateVacuumApp extends SimpleAgentApp {

	/** Returns a <code>VacuumModel</code>. */
	@Override
	public AgentAppModel createModel() {
		return new VacuumModel();
	}

	/** Returns a <code>VacuumFrame</code>. */
	@Override
	public AgentAppFrame createFrame() {
		return new VacuumFrame();
	}

	/** Returns a <code>VacuumController</code>. */
	@Override
	public AgentAppController createController() {
		return new VacuumController();
	}

	// ///////////////////////////////////////////////////////////////
	// inner classes

	/**
	 * Provides access to the used environment, the names of the locations
	 * (squares) maintained by the environment, and the current state of all
	 * locations.
	 */
	protected static class VacuumModel extends AgentAppModel {
		private UpdateVacuumEnvironment env;
		private AbstractAgent agent;

		public void setEnv(UpdateVacuumEnvironment env) {
			this.env = env;
		}

		public UpdateVacuumEnvironment getEnv() {
			return env;
		}

		public void setAgent(AbstractAgent agent) {
			this.agent = agent;
		}

		public AbstractAgent getAgent() {
			return agent;
		}
		// Checks whether the agent is currently at the specified location.
		public boolean hasAgent(XYLocation location) {
			return agent != null
					&& location.equals(env.getAgentLocation(agent));
		}
		
	}

	/**
	 * Adds some selectors to the base class and adjusts its size.
	 */
	protected static class VacuumFrame extends AgentAppFrame {
		public static String ENV_SEL = "EnvSelection";
		public static String HINDER_SEL = "HinderSelection";
		public static String DIRT_SEL = "DirtSelection";
		public static String SEED_SEL = "EnvoronmentRandomness";
		public static String AGENT_SEL = "AgentSelection";
		public static String GUI_DELAY_SEL = "GUIDelaySelection";

		public static VacuumView vacuum_view = null;
		
		public VacuumFrame() {
			vacuum_view = new VacuumView(); 
			//setAgentView(new VacuumView());
			setAgentView(vacuum_view);
			setSelectors(new String[] { 
					ENV_SEL,
					HINDER_SEL,
					DIRT_SEL,
					SEED_SEL,
					AGENT_SEL,
					GUI_DELAY_SEL
					}, new String[] {
					"Select Environment",
					"Select hinder probability",
					"Select dirt probability",
					"Select environment randomness",
					"Select GUI delay [ms]",
					"Select Agent" });
			setSelectorItems(ENV_SEL, new String[] {
					"5x5 Environment",
					"10x10 Environment",
					"15x15 Environment",
					"20x20 Environment",
					"10x5 Environment",
					"5x10 Environment"}, 0);
			setSelectorItems(HINDER_SEL, new String[] {
					"0.1",
					"0.2",
					"0.5",
					"0.0"}, 0);
			setSelectorItems(DIRT_SEL, new String[] {
					"0.1",
					"0.2",
					"0.5"}, 0);
			setSelectorItems(SEED_SEL, new String[] {
					"Yes",
					"Fixed 1",
					"Fixed 2"}, 0);
			setSelectorItems(AGENT_SEL, new String[] {
					"MyVacuumAgent", "ReactiveVacuumAgent", "RandomVacuumAgent" },
					0);
			setSelectorItems(GUI_DELAY_SEL, new String[] {
					"100ms", "500ms", "10ms","50ms","1000ms" },
					0);
			setTitle("Vacuum Agent Application");
			setSize(1024, 768);
			setUpdateDelay(500);
		}
	}

	/**
	 * Displays the informations provided by the <code>VacuumModel</code> on a
	 * panel using 2D-graphics.
	 */
	protected static class VacuumView extends AgentView {
		boolean ready_to_paint=false;
		
		/*
		Hashtable<VacuumEnvironment.Location, int[]> dirtLookup = new Hashtable<VacuumEnvironment.Location, int[]>();

		int[] getDirt(VacuumEnvironment.Location location) {
			int[] coords = dirtLookup.get(location);
			if (coords == null) {
				java.util.Random rand = new java.util.Random();
				int size = rand.nextInt(8) + 4;
				coords = new int[2 * size];
				for (int i = 0; i < size; i++) {
					coords[2 * i] = rand.nextInt(6) + 1;
					coords[2 * i + 1] = rand.nextInt(8) + 1;
				}
			}
			dirtLookup.put(location, coords);
			return coords;
		}
*/
		/**
		 * Creates a 2D-graphics showing the agent in its environment. Locations
		 * are represented as rectangles, dirt as grey ovals, and the agent as
		 * red circle.
		 */
		@Override
		public void paint(java.awt.Graphics g) {
			if (!ready_to_paint)
				return;
			VacuumModel vmodel = (VacuumModel) model;
			UpdateVacuumEnvironment env = vmodel.getEnv();
			UpdateEnvironmentState state = (UpdateEnvironmentState) env.getCurrentState(); 
			
			
			
			this.adjustTransformation(0, 0, state.width*11 , state.height*11);
			java.awt.Graphics2D g2 = (java.awt.Graphics2D) g;
			g2.setColor(Color.white);
			g2.fillRect(0, 0, getWidth(), getHeight());
			
			
			for (int i=0; i < state.width ; i++){
				for (int j=0; j<state.height;j++){
					XYLocation loc = new XYLocation(i,j);
					
					if (env.isBlocked(loc)){
						//System.out.print("X");
						g2.setColor(Color.black);
						//g2.drawRect(x(11 * i), y(11*j), scale(10), scale(10));
						g2.fillRect(x(11 * i), y(11*j), scale(10), scale(10));
					} else if (!env.isClean(loc)){
						g2.setColor(Color.black);
						g2.drawRect(x(11 * i), y(11*j), scale(10), scale(10));
						g2.setColor(Color.red);
						g2.fillRect(x(11 * i), y(11*j), scale(10), scale(10));
						//System.out.print("E");
					} else {
						g2.setColor(Color.black);
						g2.drawRect(x(11 * i), y(11*j), scale(10), scale(10));
					}
					
					//state.getObjectsAt(loc);
					if (vmodel.hasAgent(loc)) {
						Direction agent_dir = state.getDirectionFor(vmodel.agent);
						g2.setColor(Color.yellow);
						int start,size;
						start = 20;
						size = 320;
						if (agent_dir.equals(Direction.East)) {
							start = 20;
							//g2.fillArc(x(11 * i ), y(11 * j ), scale(10), scale(10), 20,320);
							//System.out.println("East");
						}else if (agent_dir.equals(Direction.North)){
							start = 110;
							//g2.fillArc(x(11 * i ), y(11 * j ), scale(10), scale(10), 110,320);
							//System.out.println("North");
						}else if (agent_dir.equals(Direction.West)){
							start = 200;
							//g2.fillArc(x(11 * i ), y(11 * j ), scale(10), scale(10), 200,320);
							//System.out.println("West");
						}else if (agent_dir.equals(Direction.South)){
							start = 290;
							//g2.fillArc(x(11 * i ), y(11 * j ), scale(10), scale(10), 290,320);
							//System.out.println("South");
						}
						g2.setColor(Color.black);
						g2.fillArc(x(11 * i ), y(11 * j ), scale(10), scale(10), start,size);
						g2.setColor(Color.yellow);
						g2.fillArc(x(11 * i )+2, y(11 * j )+2, scale(10)-4, scale(10)-4, start+5,size-10);
					}
				}
				
				
			}
			
		}
/*
		public void paint(java.awt.Graphics g) {
			VacuumModel vmodel = (VacuumModel) model;
			List<VacuumEnvironment.Location> locations = vmodel.getLocations();
			this.adjustTransformation(0, 0, 11 * locations.size() - 1, 10);
			java.awt.Graphics2D g2 = (java.awt.Graphics2D) g;
			g2.setColor(Color.white);
			g2.fillRect(0, 0, getWidth(), getHeight());
			for (int i = 0; i < locations.size(); i++) {
				VacuumEnvironment.Location location = locations.get(i);
				g2.setColor(Color.black);
				g2.drawRect(x(11 * i), y(0), scale(10), scale(10));
				if (vmodel.isDirty(location)) {
					int[] coords = getDirt(location);
					for (int j = 0; j < coords.length; j += 2) {
						g2.setColor(Color.lightGray);
						g2.fillOval(x(11 * i + coords[j]), y(coords[j + 1]),
								scale(3), scale(2));
					}
				}
				g2.setColor(Color.black);
				g2.drawString(location.toString(), x(11 * i) + 10, y(0) + 20);
				if (vmodel.hasAgent(location)) {
					g2.setColor(Color.red);
					g2.fillOval(x(11 * i + 2), y(2), scale(6), scale(6));
				}
			}
		}
		*/
	}

	/**
	 * Defines how to react on user button events.
	 */
	protected static class VacuumController extends AgentAppController {
		/** Does nothing. */
		@Override
		public void clearAgent() {
		}

		/**
		 * Creates a vacuum agent and a corresponding environment based on the
		 * selection state of the selectors and finally updates the model.
		 */
		@Override
		public void prepareAgent() {
			AgentAppFrame.SelectionState selState = frame.getSelection();
			UpdateVacuumEnvironment env = null;
			AbstractAgent agent = null;
			int width = 10;
			int height = 10;
			long random_seed=-1;
			
			switch (selState.getValue(VacuumFrame.ENV_SEL)) {
			case 0:
				//env = new LIUVacuumEnvironment(5,5);
				width = 5;
				height = 5;
				break;
			case 1:
				width = 10;
				height = 10;
				//env = new LIUVacuumEnvironment(10,10);
				break;
			case 2:
				width = 15;
				height = 15;
				//env = new LIUVacuumEnvironment(15,15);
				break;
			case 3:
				width = 20;
				height = 20;
				//env = new LIUVacuumEnvironment(20,20);
				break;
			case 4:
				width = 5;
				height = 10;
				//env = new LIUVacuumEnvironment(5,10);
				break;
			case 5:
				width = 10;
				height = 5;
				//env = new LIUVacuumEnvironment(10,5);
				break;
				
			}
			double hinder_prob = 0.1;
			double dirt_prob = 0.1;
			switch (selState.getValue(VacuumFrame.HINDER_SEL)) {
			case 0:
				hinder_prob = 0.1;
				break;
			case 1:
				hinder_prob = 0.2;
				break;
			case 2:
				hinder_prob = 0.5;
				break;
			case 3:
				hinder_prob = 0.0;
				break;
			}
			switch (selState.getValue(VacuumFrame.DIRT_SEL)) {
			case 0:
				dirt_prob = 0.1;
				break;
			case 1:
				dirt_prob = 0.2;
				break;
			case 2:
				dirt_prob = 0.5;
				break;
			}
			switch (selState.getValue(VacuumFrame.SEED_SEL)) {
			case 0:
				random_seed = -1;
				break;
			case 1:
				random_seed = 69;
				break;
			case 2:
				random_seed = 123;
				break;
			}
			switch (selState.getValue(VacuumFrame.AGENT_SEL)) {
			case 0:
				try {
					frame.logMessage("Selecting MyVacuumAgent...");
					MyVacuumAgent a = (MyVacuumAgent) Class.forName("sv_code.MyVacuumAgent").newInstance();
					a.widthAppEvironment = width;
					a.heightAppEvironment = height;
					a.assignEvironmentApp();
					agent = a;
				} catch (InstantiationException e) {
					frame.logMessage("Instantiation exception.");
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (IllegalAccessException e) {
					// TODO Auto-generated catch block
					frame.logMessage("Illegal access exception.");
					e.printStackTrace();
				} catch (ClassNotFoundException e) {
					// TODO Auto-generated catch block
					frame.logMessage("MyVacuumAgent class not found exception..");
					e.printStackTrace();
				}
				break;
			case 1:
				try {
					frame.logMessage("Selecting ReactiveVacuumAgent ...");
					agent = (AbstractAgent) Class.forName("sv_code.ReactiveVacuumAgent").newInstance();
				} catch (InstantiationException e) {
					frame.logMessage("Instantiation exception.");
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (IllegalAccessException e) {
					// TODO Auto-generated catch block
					frame.logMessage("Illegal access exception.");
					e.printStackTrace();
				} catch (ClassNotFoundException e) {
					// TODO Auto-generated catch block
					frame.logMessage("ReactiveVacuumAgent class not found exception..");
					e.printStackTrace();
				}
				break;
			case 2: 
				try {
					frame.logMessage("Selecting RandomVacuumAgent...");
					agent = (AbstractAgent) Class.forName("sv_code.RandomVacuumAgent").newInstance();
				} catch (InstantiationException e) {
					frame.logMessage("Instantiation exception.");
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (IllegalAccessException e) {
					// TODO Auto-generated catch block
					frame.logMessage("Illegal access exception.");
					e.printStackTrace();
				} catch (ClassNotFoundException e) {
					// TODO Auto-generated catch block
					frame.logMessage("RandomVacuumAgent class not found exception..");
					e.printStackTrace();
				}
				break;
			}
			int gui_delay = 100;
			 
			switch (selState.getValue(VacuumFrame.GUI_DELAY_SEL)) {
			case 0:
				gui_delay = 100;
				break;
			case 1:
				gui_delay = 500;
				break;
			case 2:
				gui_delay = 10;
				break;
			case 3:
				gui_delay = 50;
				break;
			case 4:
				gui_delay = 1000;
				break;
			}
			env = new UpdateVacuumEnvironment(width,height,hinder_prob,dirt_prob,random_seed);
			((VacuumModel) model).setEnv(env);
			((VacuumModel) model).setAgent(agent);
			if (env != null && agent != null) {
				//env.addAgent(agent, LIUVacuumEnvironment.Location.A);
				env.addAgent(agent, new XYLocation(1,1));
				env.addEnvironmentView(model);
				frame.modelChanged();
			}
			frame.setUpdateDelay(gui_delay);
			VacuumFrame.vacuum_view.ready_to_paint=true;
			VacuumFrame.vacuum_view.repaint();
			
		}

		/** Starts the agent and afterwards updates the status of the frame. */
		@Override
		public void runAgent() {
			VacuumModel vmodel = (VacuumModel) model;
			frame.logMessage("<simulation-log>");
			vmodel.getEnv().stepUntilDone();
			frame.logMessage("Performance: "
					+ vmodel.getEnv().getPerformanceMeasure(vmodel.getAgent()));
			frame.logMessage("</simulation-log>");
			frame.setStatus("Task completed.");
		}
		@Override
		public void stepAgent() {
			VacuumModel vmodel = (VacuumModel) model;
			vmodel.getEnv().step();
			frame.logMessage("Performance: "
					+ vmodel.getEnv().getPerformanceMeasure(vmodel.getAgent()));
		}
	}

	// ///////////////////////////////////////////////////////////////
	// main method

	/**
	 * Starts the application.
	 */
	public static void main(String args[]) {
		new UpdateVacuumApp().startApplication();
	}
}
