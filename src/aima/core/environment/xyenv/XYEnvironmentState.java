package aima.core.environment.xyenv;

import aima.core.agent.Agent;
import aima.core.agent.EnvironmentObject;
import aima.core.agent.EnvironmentState;
import aima.core.util.datastructure.XYLocation;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

public class XYEnvironmentState implements EnvironmentState {
	public int width;
	public int height;
	private Map<XYLocation, Set<EnvironmentObject>> objsAtLocation = new LinkedHashMap();

	public XYEnvironmentState(int width, int height) {
		this.width = width;
		this.height = height;
		for (int h = 1; h <= height; h++) {
			for (int w = 1; w <= width; w++) {
				this.objsAtLocation.put(new XYLocation(h, w),
						new LinkedHashSet());
			}
		}
	}

	public void moveObjectToAbsoluteLocation(EnvironmentObject eo,
			XYLocation loc) {
		for (Set<EnvironmentObject> eos : this.objsAtLocation.values()) {
			if (eos.remove(eo)) {
				break;
			}
		}
		getObjectsAt(loc).add(eo);
	}

	public Set<EnvironmentObject> getObjectsAt(XYLocation loc) {
		Set<EnvironmentObject> objectsAt = (Set) this.objsAtLocation.get(loc);
		if (null == objectsAt) {
			objectsAt = new LinkedHashSet();
			this.objsAtLocation.put(loc, objectsAt);
		}
		return objectsAt;
	}

	public XYLocation getCurrentLocationFor(EnvironmentObject eo) {
		for (XYLocation loc : this.objsAtLocation.keySet()) {
			if (((Set) this.objsAtLocation.get(loc)).contains(eo)) {
				return loc;
			}
		}
		return null;
	}

	public Set<EnvironmentObject> getObjectsNear(Agent agent, int radius) {
		Set<EnvironmentObject> objsNear = new LinkedHashSet();

		XYLocation agentLocation = getCurrentLocationFor(agent);
		for (XYLocation loc : this.objsAtLocation.keySet()) {
			if (withinRadius(radius, agentLocation, loc)) {
				objsNear.addAll((Collection) this.objsAtLocation.get(loc));
			}
		}
		objsNear.remove(agent);

		return objsNear;
	}

	public String toString() {
		return "XYEnvironmentState:" + this.objsAtLocation.toString();
	}

	private boolean withinRadius(int radius, XYLocation agentLocation,
			XYLocation objectLocation) {
		int xdifference = agentLocation.getXCoOrdinate()
				- objectLocation.getXCoOrdinate();

		int ydifference = agentLocation.getYCoOrdinate()
				- objectLocation.getYCoOrdinate();

		return Math.sqrt(xdifference * xdifference + ydifference * ydifference) <= radius;
	}
}
