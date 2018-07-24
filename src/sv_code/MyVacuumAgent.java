package sv_code;

import aima.core.environment.updatevacuum.*;
import aima.core.util.datastructure.XYLocation;
import aima.core.agent.Action;
import aima.core.agent.AgentProgram;
import aima.core.agent.Percept;
import aima.core.agent.impl.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Queue;
import java.util.Random;
import java.util.concurrent.CopyOnWriteArrayList;

class MyAgentState {
	public int width = 30, height = 50;
	public int[][] world = new int[width][height];
	public int initialized = 0;
	final int UNKNOWN = 0;
	final int WALL = 1;
	final int CLEAR = 2;
	final int DIRT = 3;
	final int HOME = 4;
	final int ACTION_NONE = 0;
	final int ACTION_MOVE_FORWARD = 1;
	final int ACTION_TURN_RIGHT = 2;
	final int ACTION_TURN_LEFT = 3;
	final int ACTION_SUCK = 4;

	public int agent_x_position = 0;
	public int agent_y_position = 0;
	public int agent_last_action = ACTION_NONE;

	public static final int NORTH = 0;
	public static final int EAST = 1;
	public static final int SOUTH = 2;
	public static final int WEST = 3;
	public int agent_direction = EAST;
	public List<Action> listActions = new LinkedList<>();
	public XYLocation homeLocation;

	MyAgentState() {
		for (int i = 0; i < world.length; i++)
			for (int j = 0; j < world[i].length; j++)
				world[i][j] = UNKNOWN;
		agent_last_action = ACTION_NONE;
		homeLocation = new XYLocation(0, 0);
	}

	// Based on the last action and the received percept updates the x & y agent
	// position
	public void updatePosition(DynamicPercept p) {
		Boolean bump = (Boolean) p.getAttribute("bump");

		if (agent_last_action == ACTION_MOVE_FORWARD && !bump) {
			switch (agent_direction) {
			case MyAgentState.NORTH:
				agent_y_position--;
				break;
			case MyAgentState.EAST:
				agent_x_position++;
				break;
			case MyAgentState.SOUTH:
				agent_y_position++;
				break;
			case MyAgentState.WEST:
				agent_x_position--;
				break;
			}
		}

	}

	public void updateWorld(int x_position, int y_position, int info) {
//		System.out.println();
//		System.out.println();
//		System.out.println("truoc khi tham hoa xay ra " + x_position + " " + y_position + info);
		world[x_position][y_position] = info;
	}

	public void printWorldDebug() {
		for (int i = 0; i < world.length; i++) {
			for (int j = 0; j < world[i].length; j++) {
				if (world[j][i] == UNKNOWN)
					System.out.print(" ? ");
				if (world[j][i] == WALL)
					System.out.print(" 1 ");
				if (world[j][i] == CLEAR)
					System.out.print(" 0 ");
				if (world[j][i] == DIRT)
					System.out.print(" D ");
				if (world[j][i] == HOME)
					System.out.print(" H ");
			}
			System.out.println("");
		}
	}

	public boolean isGoal(XYLocation data) {
		return data.equals(homeLocation);
	}

	/**
	 * Tim duong ve nha: - Dua tren cac diem ma vacuum da di qua de tim duong ve nha
	 */

}

class MyAgentProgram implements AgentProgram {
	public int appWidth, appHeight;
	public MyAgentState state = new MyAgentState();
	/** So buoc di truoc khi ve nha: */
	int countDownForwardMove = state.width * state.height * 2;
	/** List duong di ve: */
	List<XYLocation> homePath = new ArrayList<>();

	public Map<XYLocation, Pair<Integer, Boolean>> lookup = Collections.synchronizedMap(new LinkedHashMap<>());

	/**
	 * Luat di chuyen la: - Di het so buoc countDownForwardMove thi ve nha - Neu gap
	 * bui thi hut - Neu gap tuong thi lui lai 1 buoc (di vao tuong roi moi lui lai)
	 * va cho quay trai hoac phai ngau nhien - Di cac diem chua di theo thu tu uu
	 * tien: Di thang > Re trai > Re phai: ( Vi du: dang quay mat ve 1 phia thi xet
	 * xem phia truoc da di chua, neu chua (UNKNOW) thi di toi, neu di roi (CLEAR
	 * hoac WALL) thi xet ben trai, neu ben trai chua di thi re trai, di roi thi xet
	 * ben phai, neu ben phai chua di thi re phai, di roi thi mac dinh cho di thang
	 * huong ban dau )
	 */
	class MoveState {
		XYLocation data;
		MoveState parent;

		public MoveState(XYLocation data, MoveState parent) {
			super();
			this.data = data;
			this.parent = parent;
		}

		public List<MoveState> successMovings(List<XYLocation> haveToChecking) {
			List<MoveState> ret = new LinkedList<>();
			for (XYLocation square : haveToChecking) {
				if (state.world[square.getXCoOrdinate()][square.getYCoOrdinate()] != state.WALL)
					ret.add(new MoveState(square, this));
			}
			return ret;
		}

		@Override
		public boolean equals(Object obj) {
			if (!(obj instanceof MoveState))
				return false;

			MoveState o = (MoveState) obj;
			return this.data.equals(o.data);
		}
	}

	public List<XYLocation> findHome() {
		List<XYLocation> goal = new LinkedList<>();

		Queue<MoveState> expandStates = new LinkedList<>();
		MoveState startingState = new MoveState(new XYLocation(state.agent_x_position, state.agent_y_position), null);
		expandStates.add(startingState);

		List<MoveState> traveledStates = new LinkedList<>();

		while (!expandStates.isEmpty()) {
			MoveState movingState = expandStates.remove();
			if (this.state.isGoal(movingState.data)) {
//				System.out.println("Goal");
				getPath(movingState, goal);
				break;
			}

			traveledStates.add(movingState);
			List<MoveState> expandNextSuccessorStates = movingState.successMovings(getSquareR(movingState.data, 1));
			for (MoveState succesor : expandNextSuccessorStates) {
				if (!traveledStates.contains(succesor))
					expandStates.add(succesor);
			}

		}
		return goal;
	}

	private void getPath(MoveState check, List<XYLocation> ret) {
		if (check.parent != null) {
			ret.add(check.data);
			getPath(check.parent, ret);
		}

	}

	@Override
	public Action execute(Percept percept) {
		DynamicPercept p = (DynamicPercept) percept;
		Boolean bump = (Boolean) p.getAttribute("bump");
		Boolean dirt = (Boolean) p.getAttribute("dirt");
		Boolean home = (Boolean) p.getAttribute("home");

		if (!state.listActions.isEmpty()) {
			if (state.listActions.size() == 1) {
				if (state.listActions.get(0) == UpdateVacuumEnvironment.ACTION_MOVE_FORWARD) {
					switch (state.agent_direction) {
					case MyAgentState.NORTH:
						state.agent_y_position--;
						break;
					case MyAgentState.SOUTH:
						state.agent_y_position++;
						break;
					case MyAgentState.EAST:
						state.agent_x_position++;
						break;
					case MyAgentState.WEST:
						state.agent_x_position--;
						break;
					}
//					System.out.println("combine and set to " + state.agent_x_position + " " + state.agent_y_position);
					savingToLookupData(new XYLocation(state.agent_x_position, state.agent_y_position));
					return state.listActions.remove(0);
				}
			} else
				return state.listActions.remove(0);
		}

		if (countDownForwardMove <= 0)
			if (homePath.isEmpty() && !home) {
//				System.out.println("findding home!");
				homePath = findHome();
				if (homePath.isEmpty())
					return NoOpAction.NO_OP;
				else {
//					System.out.println("path " + homePath);
					return goHome();
				}
			} else if (!homePath.isEmpty())
				return goHome();
			else if (home && homePath.isEmpty())
				return NoOpAction.NO_OP;

		if (bump) {
			rememberEnvironment("wall");
			savingToLookupData(new XYLocation(state.agent_x_position, state.agent_y_position));
			matchingLocation();
		}

		if (dirt) {
			savingToLookupData(new XYLocation(state.agent_x_position, state.agent_y_position));
			rememberEnvironment("dirty");
		} else {
			savingToLookupData(new XYLocation(state.agent_x_position, state.agent_y_position));
			rememberEnvironment("clearn");
		}

		// starting learning
		countDownForwardMove--;

		if (dirt) {
			state.agent_last_action = state.ACTION_SUCK;
			return UpdateVacuumEnvironment.ACTION_SUCK;
		} else {
			// lay ra 4 diem lan can co the di
			// neu ca 4 diem deu khong di dc -> ket -> noops
			// neu ca 4 diem deu la ?? -> random 1 duong -> luu lai
			// neu 1 trong 4 duong la ?? -> di duong do -> luu lai
			// neu co lon hon 2 ?? -> random 1 trong 2 -> luu lai
			// neu ca 4 duong deu cleanr - > chon duong co so lan it di nhat

			List<XYLocation> oneSquareR = Collections
					.synchronizedList(getSquareR(new XYLocation(state.agent_x_position, state.agent_y_position), 1));
//			System.out.println("con lai " + (countDownForwardMove));
			List<XYLocation> walls = Collections.synchronizedList(new LinkedList<>());
			List<XYLocation> wellKnowns = Collections.synchronizedList(new LinkedList<>());
			List<XYLocation> unKnows = Collections.synchronizedList(new LinkedList<>());
//			for (XYLocation s : oneSquareR) {
//				System.out.println(s.getXCoOrdinate() + " " + s.getYCoOrdinate());
//			}
			int indexOfUnknowSquare = -1;
			int index = 0;
			for (XYLocation location : oneSquareR) {
				if (state.world[location.getXCoOrdinate()][location.getYCoOrdinate()] == state.WALL) {
//					System.out.println("have wall");
					walls.add(location);
				} else if (state.world[location.getXCoOrdinate()][location.getYCoOrdinate()] == state.UNKNOWN) {
//					System.out.println("have unknown" + location.getXCoOrdinate() + " " + location.getYCoOrdinate());
					indexOfUnknowSquare = index;
					unKnows.add(location);

				} else {
//					System.out.println("well knows " + location.getXCoOrdinate() + " " + location.getYCoOrdinate()
//							+ "  " + state.world[location.getXCoOrdinate()][location.getYCoOrdinate()]);
					wellKnowns.add(location);
				}
				index++;
			}
			// ket trong 4 buc tuong
			if (walls.size() == 4) {
//				System.out.println("ket trong 4 buc tuong");
				return NoOpAction.NO_OP;
			}

			// co >= 1 o chua di
			if (indexOfUnknowSquare != -1) {
				XYLocation unknown = unKnows.get(new Random().nextInt(unKnows.size()));
//				System.out.println("huong hien tai " + state.agent_direction);
//				System.out.println("vi tri hien tai" + state.agent_x_position + " " + state.agent_y_position);
//				System.out.println("co 1 o chua di" + unknown.getXCoOrdinate() + " " + unknown.getYCoOrdinate());
				// luu lai data cua o chua di do
				savingToLookupData(unknown);
				return moveToSquare(unknown);
			}
			// tat ca cac o xung quanh deu di roi
			else {
				XYLocation bestChoice = getBestChoice(wellKnowns);
				savingToLookupData(bestChoice);
				return moveToSquare(bestChoice);
			}

		}

	}

	private XYLocation getBestChoice(List<XYLocation> wellKnowns) {

		List<Pair<Integer, Boolean>> entries = new CopyOnWriteArrayList<>();

		for (XYLocation location : wellKnowns) {
			entries.add(lookup.get(location));
		}
		int minIndex = 0;
//		System.out.println("entries size " + entries.size() + " wellknowsn size " + wellKnowns.size());
//		System.out.println("null ?" + (entries.get(minIndex).left == null));
		for (int index = 0; index < entries.size(); index++) {
//			System.out.println("left " + (entries.get(index).left == null));
			if (entries.get(minIndex).left > entries.get(index).left)
				minIndex = index;
		}
//		System.out.println("entries size " + entries.size());
		return wellKnowns.get(minIndex);
	}

	private Action moveToSquare(XYLocation newLocation) {
		// chuan hoa huong hien tai ve huong east == 1
		// sau do xet vi tri cua huong moi ->so lan quay trai
		// buoc cuoi cung la move forward
		// int haveToTurnLeft = 0;
		// if (state.agent_direction == MyAgentState.EAST)
		// haveToTurnLeft = 0;
		// if (state.agent_direction == MyAgentState.NORTH)
		// haveToTurnLeft = 3;
		// if (state.agent_direction == MyAgentState.WEST)
		// haveToTurnLeft = 2;
		// if (state.agent_direction == MyAgentState.SOUTH)
		// haveToTurnLeft = 1;
		// if (haveToTurnLeft != 0)
		// state.agent_direction = MyAgentState.EAST;
		// while (haveToTurnLeft != 0) {
		// state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
		// haveToTurnLeft--;
		// }
		// // xu ly
		int newX = newLocation.getXCoOrdinate();
		int newY = newLocation.getYCoOrdinate();
		int oldX = state.agent_x_position;
		int oldY = state.agent_y_position;

		if (state.agent_direction == MyAgentState.EAST) {
			// tien toi hoi EAST
			if ((newX - oldX == 1) && newY == oldY) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.EAST;
			}
			// tien toi NORTH
			if (newX == oldX && (oldY - newY == 1)) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.NORTH;
			}
			// tien toi South
			if (newX == oldX && (newY - oldY == 1)) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_RIGHT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.SOUTH;
			}
			// tien toi WEST
			if ((oldX - newX == 1) && newY == oldY) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.WEST;
			}

		}

		else if (state.agent_direction == MyAgentState.NORTH) {
			// tien toi hoi EAST
			if ((newX - oldX == 1) && newY == oldY) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_RIGHT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.EAST;
			}
			// tien toi NORTH
			if (newX == oldX && (oldY - newY == 1)) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.NORTH;
			}
			// tien toi South
			if (newX == oldX && (newY - oldY == 1)) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.SOUTH;
			}
			// tien toi WEST
			if ((oldX - newX == 1) && newY == oldY) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.WEST;
			}
		} else if (state.agent_direction == MyAgentState.WEST) {
			// tien toi hoi EAST
			if ((newX - oldX == 1) && newY == oldY) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.EAST;
			}
			// tien toi NORTH
			if (newX == oldX && (oldY - newY == 1)) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_RIGHT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.NORTH;
			}
			// tien toi South
			if (newX == oldX && (newY - oldY == 1)) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.SOUTH;
			}
			// tien toi WEST
			if ((oldX - newX == 1) && newY == oldY) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.WEST;
			}
		} else if (state.agent_direction == MyAgentState.SOUTH) {
			// tien toi hoi EAST
			if ((newX - oldX == 1) && newY == oldY) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.EAST;
			}
			// tien toi NORTH
			if (newX == oldX && (oldY - newY == 1)) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_LEFT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.NORTH;
			}
			// tien toi South
			if (newX == oldX && (newY - oldY == 1)) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.SOUTH;
			}
			// tien toi WEST
			if ((oldX - newX == 1) && newY == oldY) {
				state.listActions.add(UpdateVacuumEnvironment.ACTION_TURN_RIGHT);
				state.listActions.add(UpdateVacuumEnvironment.ACTION_MOVE_FORWARD);
				state.agent_direction = MyAgentState.WEST;
			}
		}

		Action s = state.listActions.remove(0);
		if (s == UpdateVacuumEnvironment.ACTION_MOVE_FORWARD)
			state.agent_last_action = state.ACTION_MOVE_FORWARD;
		else if (s == UpdateVacuumEnvironment.ACTION_SUCK)
			state.agent_last_action = state.ACTION_SUCK;
		else if (s == UpdateVacuumEnvironment.ACTION_TURN_LEFT) {
			state.agent_last_action = state.ACTION_TURN_LEFT;
		} else if (s == UpdateVacuumEnvironment.ACTION_TURN_RIGHT)
			state.agent_last_action = state.ACTION_TURN_RIGHT;
		else
			state.agent_last_action = state.ACTION_NONE;

		if (state.listActions.size() == 0) {
			state.agent_x_position = newX;
			state.agent_y_position = newY;
//			System.out.println("setIn nextSquare " + newX + " " + newY);
		}

		return s;
	}

	private synchronized void savingToLookupData(XYLocation xyLocation) {
		// location , (so lan di qua , co phai tuong k)
		Pair<Integer, Boolean> entry = lookup.get(xyLocation);
		// chua di qua == hoan toan chua co du lieu
		if (entry == null) {
			entry = new Pair<Integer, Boolean>();
			// va thuc te bang wall (updating vao trong du lieu roi) -> updating vao lookup
			if (state.world[xyLocation.getXCoOrdinate()][xyLocation.getYCoOrdinate()] == state.WALL) {
				entry.left = 1;
				entry.right = true;
				lookup.put(xyLocation, entry);
			}
			// khong phai wall
			else {
				entry.left = 1;
				entry.right = false;
				lookup.put(xyLocation, entry);
			}
		}
		// khac null -> da di qua roi -> co the la wall , hoac sach se (co so lan di
		// qua)
		else {
			// entry.right == wall // wall chi di 1 lan thoi

			if (entry.right)
				return;
			entry.left++;
			lookup.put(xyLocation, entry);
		}
	}

	private synchronized List<XYLocation> getSquareR(XYLocation root, int i) {
		int width = state.width - 1;
		int height = state.height - 1;
		List<XYLocation> success = new LinkedList<>();
		List<XYLocation> ret = new LinkedList<>();
		int x = root.getXCoOrdinate();
		int y = root.getYCoOrdinate();

		if (x == 0 && y == 0) {
			success.add(new XYLocation(0, 1));
			success.add(new XYLocation(1, 0));
		} else if (x == width && y == 0) {
			success.add(new XYLocation(x - 1, 0));
			success.add(new XYLocation(x, 1));
		} else if (x == width && y == height) {
			success.add(new XYLocation(width, height - 1));
			success.add(new XYLocation(width - 1, height));
		} else if (x == 0 && y == height) {
			success.add(new XYLocation(0, height - 1));
			success.add(new XYLocation(1, height));
		} else if (y == height && (x != 0 && x != width)) {
			success.add(new XYLocation(x - 1, y));
			success.add(new XYLocation(x + 1, y));
			success.add(new XYLocation(x, y - 1));
		} else if (y == 0 && (x != 0 && x != width)) {
			success.add(new XYLocation(x - 1, 0));
			success.add(new XYLocation(x + 1, 0));
			success.add(new XYLocation(x, 1));
		} else if (x == 0 && (y != 0 && y != height)) {
			success.add(new XYLocation(0, y + 1));
			success.add(new XYLocation(0, y - 1));
			success.add(new XYLocation(1, y));
		} else if (x == width && (y != 0 && y != height)) {
			success.add(new XYLocation(x, y - 1));
			success.add(new XYLocation(x, y + 1));
			success.add(new XYLocation(x - 1, y));
		} else {
			success.add(new XYLocation(x + 1, y));
			success.add(new XYLocation(x - 1, y));
			success.add(new XYLocation(x, y - 1));
			success.add(new XYLocation(x, y + 1));
		}
		if (i == 1) {
			for (XYLocation o : success)
				if (o.getXCoOrdinate() < appWidth && o.getYCoOrdinate() < appHeight)
					ret.add(o);
			return ret;
		}
		for (XYLocation square : success) {
			getSquareR(root, square, ret);
		}
		return ret;
	}

	private void getSquareR(XYLocation root, XYLocation next, List<XYLocation> container) {
		if (next.equals(root))
			return;
		List<XYLocation> nextExpand = getSquareR(next, 1);
		for (XYLocation s : nextExpand)
			if (!container.contains(s))
				container.add(s);
	}

	private void matchingLocation() {
		if (state.agent_last_action == state.ACTION_MOVE_FORWARD)
			switch (state.agent_direction) {
			case MyAgentState.EAST:
				state.agent_x_position--;
				break;
			case MyAgentState.WEST:
				state.agent_x_position++;
				break;
			case MyAgentState.NORTH:
				state.agent_y_position++;
				break;
			case MyAgentState.SOUTH:
				state.agent_y_position--;
				break;
			}

	}

	private void rememberEnvironment(String string) {
		switch (string) {
		case "wall":
			state.updateWorld(state.agent_x_position, state.agent_y_position, state.WALL);
			break;
		case "dirty":
			state.updateWorld(state.agent_x_position, state.agent_y_position, state.DIRT);
			break;
		case "clearn":
			state.updateWorld(state.agent_x_position, state.agent_y_position, state.CLEAR);
			break;
		default:
			state.updateWorld(state.agent_x_position, state.agent_y_position, state.UNKNOWN);
			break;
		}
	}

	private Action goHome() {
		return moveToSquare(homePath.remove(homePath.size() - 1));
	}

}

public class MyVacuumAgent extends AbstractAgent {
	public int widthAppEvironment;
	public int heightAppEvironment;

	public MyVacuumAgent() {
		super(new MyAgentProgram());
	}

	public void assignEvironmentApp() {
		((MyAgentProgram) this.program).appHeight = heightAppEvironment;
		((MyAgentProgram) this.program).appWidth = widthAppEvironment;
	}
}

class Pair<K, V> {
	K left;
	V right;

	public Pair() {
	}

	public Pair(K left, V right) {
		super();
		this.left = left;
		this.right = right;
	}

}
