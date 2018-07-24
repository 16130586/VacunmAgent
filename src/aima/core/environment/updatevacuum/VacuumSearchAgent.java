package aima.core.environment.updatevacuum;

import aima.core.agent.Action;
import aima.core.agent.impl.AbstractAgent;
import java.util.List;

public class VacuumSearchAgent extends AbstractAgent {
	private boolean live = true;

	public void setActionSeq(List<Action> list) {
		cleanActionSeq();
		for (Action a : list) {
			((UpdateVacuumProgram) this.program).actions_seq.add(a);
		}
		((UpdateVacuumProgram) this.program).iter = ((UpdateVacuumProgram) this.program).actions_seq
				.iterator();
	}

	public void cleanActionSeq() {
		((UpdateVacuumProgram) this.program).actions_seq.clear();
		((UpdateVacuumProgram) this.program).iter = null;
	}

	public VacuumSearchAgent() {
		super(new UpdateVacuumProgram());
	}

	public boolean isAlive() {
		return this.live;
	}

	public void setAlive(boolean alive) {
		this.live = alive;
	}
}
