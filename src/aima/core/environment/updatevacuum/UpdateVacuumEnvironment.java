package aima.core.environment.updatevacuum;

import java.util.Random;
import java.util.Set;

import aima.core.agent.Action;
import aima.core.agent.Agent;
import aima.core.agent.EnvironmentObject;
import aima.core.agent.EnvironmentState;
import aima.core.agent.Percept;
import aima.core.agent.impl.AbstractEnvironment;
import aima.core.agent.impl.DynamicAction;
import aima.core.agent.impl.DynamicPercept;
import aima.core.environment.xyenv.*;
import aima.core.util.datastructure.XYLocation;
import aima.core.util.datastructure.XYLocation.Direction;

public class UpdateVacuumEnvironment extends AbstractEnvironment {
	public static final Action ACTION_TURN_LEFT = new DynamicAction("TurnLeft");
	public static final Action ACTION_TURN_RIGHT = new DynamicAction("TurnRight");
	public static final Action ACTION_MOVE_FORWARD = new DynamicAction("MoveForward");
	public static final Action ACTION_SUCK = new DynamicAction("Suck");
	
	public static final String ATTRIBUTE_BUMP = "bump";
	public static final String ATTRIBUTE_DIRT = "dirt";
	public static final String ATTRIBUTE_HOME = "home";
	
	private long random_seed = 6969;
	private UpdateEnvironmentState envState = null; 
	private boolean bump = false;
	
	private boolean isDone = false;
	
	public UpdateVacuumEnvironment(int width, int height) {
		assert (width > 0);
		assert (height > 0);

		envState = new UpdateEnvironmentState(width+2, height+2);
		makePerimeter();
	
		
		addRandomDirt(5);
	}
	public UpdateVacuumEnvironment(int width, int height,double hinder, double dirt, long r_seed) {
		assert (width > 0);
		assert (height > 0);
		
		random_seed = r_seed;

		envState = new UpdateEnvironmentState(width+2, height+2);
		makePerimeter();
		
		int size = width*height;
		
		addHinders((int)(size*hinder));
		
		addRandomDirt((int)((size*(1-hinder))*dirt));
		
	}

	public boolean isClean(XYLocation loc) {
		for (EnvironmentObject eo : envState.getObjectsAt(loc)) {
			if (eo instanceof Dirt) {
				return false;
			}
		}
		return true;
	}
	public void addRandomDirt(int dirt_cnt) {
		Random r = new Random();
		//System.out.println("random_seed1 " + random_seed);
		if (random_seed!=-1)
			r.setSeed(random_seed);
		
		int i=0;
		while(i<dirt_cnt){
			XYLocation loc = new XYLocation(
					r.nextInt(envState.width-1)+1,
					r.nextInt(envState.height-1)+1
					);
			if (!isBlocked(loc) && isClean(loc)) {
				moveObjectToAbsoluteLocation(new Dirt(), loc);
				i++;
				//System.out.println("Added dirt to loc " + loc);
			}
			
		}
	}
	public void addHinders(int hinder_cnt) {
		Random r = new Random();
		//System.out.println("random_seed2 " + random_seed);
		if (random_seed!=-1)
			r.setSeed(random_seed);
		int i=0;
		while(i<hinder_cnt){
			int x = r.nextInt(envState.width-1)+1;
			int y = r.nextInt(envState.height-1)+1;
			if (x==1 && y==1)
				continue;
			XYLocation loc = new XYLocation(x,y);
			if (!isBlocked(loc) && isClean(loc)) {
				moveObjectToAbsoluteLocation(new Wall(), loc);
				i++;
				//System.out.println("Added dirt to loc " + loc);
			}
			
		}
	}
	
	public void addAgent(Agent a, XYLocation location) {
		// Ensure the agent state information is tracked before
		// adding to super, as super will notify the registered
		// EnvironmentViews that is was added.
		addAgent(a);
		moveObjectToAbsoluteLocation(a, location);
		// Dat agent vao vi tri va quay ve phia EAST:
		envState.setDirectionFor(a, Direction.East);
		updatePerformanceMeasure(a, 0);
		//System.out.println("Added agent to loc " + location);
		//System.out.println("Agent dir " + envState.getDirectionFor(a) );
	}

	public XYLocation getAgentLocation(Agent a) {
		return getCurrentLocationFor(a);
	}

	@Override
	public EnvironmentState getCurrentState() {
		return envState;
	}

	@Override
	public EnvironmentState executeAction(Agent a, Action action) {
		bump = false;
		if (ACTION_TURN_RIGHT == action) {
			Direction old_dir = envState.getDirectionFor(a);
			
			if (old_dir.equals(Direction.North)) envState.setDirectionFor(a, Direction.East);
			else if (old_dir.equals(Direction.East)) envState.setDirectionFor(a, Direction.South);
			else if (old_dir.equals(Direction.South)) envState.setDirectionFor(a, Direction.West);
			else if (old_dir.equals(Direction.West)) envState.setDirectionFor(a, Direction.North);
			updatePerformanceMeasure(a, -1);
			
		} else if (ACTION_TURN_LEFT == action) {
			Direction old_dir = envState.getDirectionFor(a);
			
			if (old_dir.equals(Direction.North)) envState.setDirectionFor(a, Direction.West);
			else if (old_dir.equals(Direction.West)) envState.setDirectionFor(a, Direction.South);
			else if (old_dir.equals(Direction.South)) envState.setDirectionFor(a, Direction.East);
			else if (old_dir.equals(Direction.East)) envState.setDirectionFor(a, Direction.North);
			updatePerformanceMeasure(a, -1);
			
		} else if (ACTION_MOVE_FORWARD == action) {
			Direction dir = envState.getDirectionFor(a);
			XYLocation loc = envState.getCurrentLocationFor(a);
			
			//System.out.println("FORWARD loc=" + loc);
			if (dir.equals(Direction.North)) loc = loc.north();
			else if (dir.equals(Direction.West)) loc = loc.west();
			else if (dir.equals(Direction.South)) loc = loc.south();
			else if (dir.equals(Direction.East)) loc = loc.east();
			
			if (isBlocked(loc)){
				// Bump
				//System.out.println("BUMP");
				bump = true;
				
			} else {
				//System.out.println("MOVED to" + loc);
				envState.moveObjectToAbsoluteLocation(a, loc);
			}
			
			updatePerformanceMeasure(a, -1);
			
		} else if (ACTION_SUCK == action) {
			XYLocation loc = envState.getCurrentLocationFor(a);
			//System.out.println("SUCK" + loc);
			if (isClean(loc)){
				updatePerformanceMeasure(a, -1);
			} else {
				for (EnvironmentObject eo : envState.getObjectsAt(loc)) {
					if (eo instanceof Dirt) {
						//System.out.println("SUCK2" + loc);
						envState.removeObjectFromAbsoluteLocation(eo,loc);
						break;
					}
				}
				updatePerformanceMeasure(a, 100);
			}
			
			
			/*
			if (LocationState.Dirty == envState.getLocationState(envState
					.getAgentLocation(a))) {
				envState.setLocationState(envState.getAgentLocation(a),
						LocationState.Clean);
				updatePerformanceMeasure(a, 10);
			}*/
		} else if (action.isNoOp()) {
			// In the Vacuum Environment we consider things done if
			// the agent generates a NoOp.
			
			XYLocation loc = envState.getCurrentLocationFor(a);
			if (loc.equals(new XYLocation(1,1)))
				updatePerformanceMeasure(a, 1000);
			else
				updatePerformanceMeasure(a, -1);
			isDone = true;
		}
		
		return envState;
	}

	@Override
	public Percept getPerceptSeenBy(Agent anAgent) {
		//return new DynamicPercept();
		//String []percept = {new String("bump"), new String("dirt"),new String("home")};

		XYLocation loc = envState.getCurrentLocationFor(anAgent);
		Boolean []values = {Boolean.valueOf(bump),Boolean.valueOf(!isClean(loc)),Boolean.valueOf(loc.equals(new XYLocation(1,1)))};
		//Boolean []values = {valueOf(true),valueOf(true),valueOf(false)};
		String []percept = {ATTRIBUTE_BUMP,ATTRIBUTE_DIRT,ATTRIBUTE_HOME};
		
		DynamicPercept p = new DynamicPercept(percept, values);
		//System.out.println("b= " + p.getAttribute(ATTRIBUTE_BUMP)+" d= " + p.getAttribute(ATTRIBUTE_DIRT)+" h= " + p.getAttribute(ATTRIBUTE_HOME));
		return p;
	}

	public void addObjectToLocation(EnvironmentObject eo, XYLocation loc) {
		moveObjectToAbsoluteLocation(eo, loc);
	}

	public void moveObjectToAbsoluteLocation(EnvironmentObject eo,
			XYLocation loc) {
		// Ensure the object is not already at a location
		envState.moveObjectToAbsoluteLocation(eo, loc);

		// Ensure is added to the environment
		addEnvironmentObject(eo);
	}

	public void moveObject(EnvironmentObject eo, XYLocation.Direction direction) {
		XYLocation presentLocation = envState.getCurrentLocationFor(eo);

		if (null != presentLocation) {
			XYLocation locationToMoveTo = presentLocation.locationAt(direction);
			if (!(isBlocked(locationToMoveTo))) {
				moveObjectToAbsoluteLocation(eo, locationToMoveTo);
			}
		}
	}

	public XYLocation getCurrentLocationFor(EnvironmentObject eo) {
		return envState.getCurrentLocationFor(eo);
	}

	public Set<EnvironmentObject> getObjectsAt(XYLocation loc) {
		return envState.getObjectsAt(loc);
	}

	public Set<EnvironmentObject> getObjectsNear(Agent agent, int radius) {
		return envState.getObjectsNear(agent, radius);
	}

	public boolean isBlocked(XYLocation loc) {
		for (EnvironmentObject eo : envState.getObjectsAt(loc)) {
			if (eo instanceof Wall) {
				return true;
			}
		}
		return false;
	}	
	public void makePerimeter() {
		for (int i = 0; i < envState.width; i++) {
			XYLocation loc = new XYLocation(i, 0);
			XYLocation loc2 = new XYLocation(i, envState.height - 1);
			envState.moveObjectToAbsoluteLocation(new Wall(), loc);
			envState.moveObjectToAbsoluteLocation(new Wall(), loc2);
		}

		for (int i = 0; i < envState.height; i++) {
			XYLocation loc = new XYLocation(0, i);
			XYLocation loc2 = new XYLocation(envState.width - 1, i);
			envState.moveObjectToAbsoluteLocation(new Wall(), loc);
			envState.moveObjectToAbsoluteLocation(new Wall(), loc2);
		}
	}
	
	@Override
	public boolean isDone() {
		return super.isDone() || isDone;
	}
}
