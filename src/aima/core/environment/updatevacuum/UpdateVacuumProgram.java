package aima.core.environment.updatevacuum;

import aima.core.agent.Action;
import aima.core.agent.AgentProgram;
import aima.core.agent.Percept;
import aima.core.agent.impl.NoOpAction;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

class UpdateVacuumProgram implements AgentProgram {
	public List<Action> actions_seq = new ArrayList();
	public int iterationCounter = 100;
	public Iterator<Action> iter = null;

	public Action execute(Percept percept) {
		this.iterationCounter -= 1;
		if ((this.actions_seq.isEmpty()) || (this.actions_seq == null)) {
			return NoOpAction.NO_OP;
		}
		if (this.iterationCounter == 0) {
			return NoOpAction.NO_OP;
		}
		if (this.iter.hasNext()) {
			return (Action) this.iter.next();
		}
		return NoOpAction.NO_OP;
	}
}
