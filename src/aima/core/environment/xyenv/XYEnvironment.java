package aima.core.environment.xyenv;

import aima.core.agent.Action;
import aima.core.agent.Agent;
import aima.core.agent.EnvironmentObject;
import aima.core.agent.EnvironmentState;
import aima.core.agent.Percept;
import aima.core.agent.impl.AbstractEnvironment;
import aima.core.agent.impl.DynamicPercept;
import aima.core.util.datastructure.XYLocation;
import aima.core.util.datastructure.XYLocation.Direction;
import java.util.Set;

public class XYEnvironment extends AbstractEnvironment {
	private XYEnvironmentState envState = null;

	public XYEnvironment(int width, int height) {
		assert (width > 0);
		assert (height > 0);

		this.envState = new XYEnvironmentState(width, height);
	}

	public EnvironmentState getCurrentState() {
		return this.envState;
	}

	public EnvironmentState executeAction(Agent a, Action action) {
		return this.envState;
	}

	public Percept getPerceptSeenBy(Agent anAgent) {
		return new DynamicPercept();
	}

	public void addObjectToLocation(EnvironmentObject eo, XYLocation loc) {
		moveObjectToAbsoluteLocation(eo, loc);
	}

	public void moveObjectToAbsoluteLocation(EnvironmentObject eo,
			XYLocation loc) {
		this.envState.moveObjectToAbsoluteLocation(eo, loc);

		addEnvironmentObject(eo);
	}

	public void moveObject(EnvironmentObject eo, XYLocation.Direction direction) {
		XYLocation presentLocation = this.envState.getCurrentLocationFor(eo);
		if (null != presentLocation) {
			XYLocation locationToMoveTo = presentLocation.locationAt(direction);
			if (!isBlocked(locationToMoveTo)) {
				moveObjectToAbsoluteLocation(eo, locationToMoveTo);
			}
		}
	}

	public XYLocation getCurrentLocationFor(EnvironmentObject eo) {
		return this.envState.getCurrentLocationFor(eo);
	}

	public Set<EnvironmentObject> getObjectsAt(XYLocation loc) {
		return this.envState.getObjectsAt(loc);
	}

	public Set<EnvironmentObject> getObjectsNear(Agent agent, int radius) {
		return this.envState.getObjectsNear(agent, radius);
	}

	public boolean isBlocked(XYLocation loc) {
		for (EnvironmentObject eo : this.envState.getObjectsAt(loc)) {
			if ((eo instanceof Wall)) {
				return true;
			}
		}
		return false;
	}

	// Create wall:
	public void makePerimeter() {
		for (int i = 0; i < this.envState.width; i++) {
			XYLocation loc = new XYLocation(i, 0);
			XYLocation loc2 = new XYLocation(i, this.envState.height - 1);
			this.envState.moveObjectToAbsoluteLocation(new Wall(), loc);
			this.envState.moveObjectToAbsoluteLocation(new Wall(), loc2);
		}
		for (int i = 0; i < this.envState.height; i++) {
			XYLocation loc = new XYLocation(0, i);
			XYLocation loc2 = new XYLocation(this.envState.width - 1, i);
			this.envState.moveObjectToAbsoluteLocation(new Wall(), loc);
			this.envState.moveObjectToAbsoluteLocation(new Wall(), loc2);
		}
	}
}
