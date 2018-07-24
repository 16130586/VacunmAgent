package aima.core.environment.updatevacuum;

import aima.core.agent.Action;
import aima.core.agent.impl.AbstractAgent;
import java.util.ArrayList;
import java.util.List;

public class UpdateVacuumGUIAgent
  extends AbstractAgent
{
  public void setActionSeq(ArrayList<Action> actionsSeq)
  {
    cleanActionSeq();
    for (Action a : actionsSeq) {
      ((UpdateVacuumProgram)this.program).actions_seq.add(a);
    }
    ((UpdateVacuumProgram)this.program).iter = ((UpdateVacuumProgram)this.program).actions_seq.iterator();
  }
  
  public void cleanActionSeq()
  {
    ((UpdateVacuumProgram)this.program).actions_seq.clear();
    ((UpdateVacuumProgram)this.program).iter = null;
  }
  
  public UpdateVacuumGUIAgent()
  {
    super(new UpdateVacuumProgram());
  }
}
