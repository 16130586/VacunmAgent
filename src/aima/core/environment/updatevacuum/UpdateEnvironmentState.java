package aima.core.environment.updatevacuum;

import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import aima.core.agent.Agent;
import aima.core.agent.EnvironmentObject;
import aima.core.agent.EnvironmentState;
import aima.core.util.datastructure.XYLocation;
import aima.core.util.datastructure.XYLocation.Direction;


public class UpdateEnvironmentState implements EnvironmentState {
	public int width;
	public int height;

	
	private Map<XYLocation, Set<EnvironmentObject>> objsAtLocation = new LinkedHashMap<XYLocation, Set<EnvironmentObject>>();
	private Map<EnvironmentObject, Direction> objsDirection = new LinkedHashMap<EnvironmentObject, Direction>();
	
	public UpdateEnvironmentState(int width, int height) {
		this.width = width;
		this.height = height;
		for (int h = 1; h <= height; h++) {
			for (int w = 1; w <= width; w++) {
				objsAtLocation.put(new XYLocation(h, w),
						new LinkedHashSet<EnvironmentObject>());
			}
		}
	}

	public void moveObjectToAbsoluteLocation(EnvironmentObject eo,
			XYLocation loc) {
		// Ensure is not already at another location
		for (Set<EnvironmentObject> eos : objsAtLocation.values()) {
			if (eos.remove(eo)) {
				break; // Should only every be at 1 location
			}
		}
		// Add it to the location specified
		getObjectsAt(loc).add(eo);
	}

	public void removeObjectFromAbsoluteLocation(EnvironmentObject eo,
			XYLocation loc) {
		Set<EnvironmentObject> eos = objsAtLocation.get(loc);
		eos.remove(eo);
	}
	
	
	public Set<EnvironmentObject> getObjectsAt(XYLocation loc) {
		Set<EnvironmentObject> objectsAt = objsAtLocation.get(loc);
		if (null == objectsAt) {
			// Always ensure an empty Set is returned
			objectsAt = new LinkedHashSet<EnvironmentObject>();
			objsAtLocation.put(loc, objectsAt);
		}
		return objectsAt;
	}

	public XYLocation getCurrentLocationFor(EnvironmentObject eo) {
		for (XYLocation loc : objsAtLocation.keySet()) {
			if (objsAtLocation.get(loc).contains(eo)) {
				return loc;
			}
		}
		return null;
	}

	public Direction getDirectionFor(EnvironmentObject eo) {
		return objsDirection.get(eo);
	}
	
	public void setDirectionFor(EnvironmentObject eo, Direction dir) {
		objsDirection.put(eo, dir);
	}
	
	public Set<EnvironmentObject> getObjectsNear(Agent agent, int radius) {
		Set<EnvironmentObject> objsNear = new LinkedHashSet<EnvironmentObject>();

		XYLocation agentLocation = getCurrentLocationFor(agent);
		for (XYLocation loc : objsAtLocation.keySet()) {
			if (withinRadius(radius, agentLocation, loc)) {
				objsNear.addAll(objsAtLocation.get(loc));
			}
		}
		// Ensure the 'agent' is not included in the Set of
		// objects near
		objsNear.remove(agent);

		return objsNear;
	}

	@Override
	public String toString() {
		return "LIUEnvironmentState:" + objsAtLocation.toString();
	}

	//
	// PRIVATE METHODS
	//
	private boolean withinRadius(int radius, XYLocation agentLocation,
			XYLocation objectLocation) {
		int xdifference = agentLocation.getXCoOrdinate()
				- objectLocation.getXCoOrdinate();
		int ydifference = agentLocation.getYCoOrdinate()
				- objectLocation.getYCoOrdinate();
		return Math.sqrt((xdifference * xdifference)
				+ (ydifference * ydifference)) <= radius;
	}
}