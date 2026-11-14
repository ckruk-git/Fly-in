import parser


class Drone:
    def __init__(self, drone_id: str, path: list[str]) -> None:
        self.id: str = drone_id
        self.path: list[str] = path
        self.path_index: int = 0
        self.delivered: bool = False
        self.in_transit: bool = False
        self.target: parser.Hub | None = None
        self.transit_remaining: int = 0
        self.transit_connection: str | None = None
        self.transit_conn: parser.Connection | None = None
        self.prev_position: str = self.position

    @property
    def position(self) -> str:
        return self.path[self.path_index]


class Simulation:
    def __init__(
        self,
        drones: list[Drone],
        graph: dict[str, list[str]],
        hubs: dict[str, parser.Hub],
        connections: dict[frozenset[str], parser.Connection],
        end_name: str
    ) -> None:
        self.drones: list[Drone] = drones
        self.graph: dict[str, list[str]] = graph
        self.hubs: dict[str, parser.Hub] = hubs
        self.turn: int = 0
        self.end_name: str = end_name
        self.connections: dict[frozenset[str], parser.Connection] = connections
        self.animation: int = 0

        for hub in hubs.values():
            hub.cur_occupancy = 0
        for drone in drones:
            hubs[drone.position].cur_occupancy += 1

    def execution(self, execution_list: list[Drone]) -> None:
        for drone in execution_list:
            next_name = drone.path[drone.path_index + 1]
            next_hub = self.hubs[next_name]
            drone.target = next_hub
            drone.path_index += 1
        self.turn += 1
        return

    def step(self) -> list[str]:
        drones_queue = sorted(
            self.drones,
            key=lambda drone: drone.path_index,
            reverse=True
        )
        execution_list = []
        moves = []
        for conn in self.connections.values():
            conn.cur_occupancy = 0
        for drone in drones_queue:
            if drone.path[drone.path_index] == self.end_name:
                drone.delivered = True
            if drone.delivered:
                continue
            if drone.in_transit:
                if drone.target is None or drone.transit_conn is None:
                    print("ERROR: missing transit target", drone.id)
                    continue
                if drone.transit_remaining > 0:
                    if drone.target.cur_occupancy >= drone.target.max_drones:
                        moves.append(
                            f"{drone.id}-{drone.transit_connection}"
                        )
                        continue
                    drone.transit_remaining -= 1
                    moves.append(
                        f"{drone.id}-{drone.transit_connection}"
                    )
                    continue
                drone.target.cur_occupancy += 1
                drone.transit_conn.transit_occupancy -= 1
                drone.prev_position = drone.position
                drone.target = None
                drone.transit_connection = None
                drone.transit_conn = None
                drone.in_transit = False
                moves.append(f"{drone.id}-{drone.position}")
            if not drone.in_transit:
                drone.prev_position = drone.position
            if drone.path_index + 1 >= len(drone.path):
                continue
            cur_name = drone.position
            next_name = drone.path[drone.path_index + 1]
            cur_hub = self.hubs[cur_name]
            next_hub = self.hubs[next_name]
            conn = self.connections[frozenset({cur_name, next_name})]
            drone.target = next_hub
            if next_hub.zone == "blocked":
                continue
            if next_name != self.end_name:
                if next_hub.cur_occupancy >= next_hub.max_drones:
                    continue
            if (
                next_hub.zone != "restricted" and
                conn.cur_occupancy + conn.transit_occupancy
                >= conn.max_link_capacity
            ):
                continue
            if next_hub.zone == "restricted":
                if conn.cur_occupancy + conn.transit_occupancy > conn.max_link_capacity:
                    continue
                drone.in_transit = True
                conn.transit_occupancy += 1
                drone.path_index += 1
                cur_hub.cur_occupancy -= 1
                drone.transit_connection = f"{cur_name}-{next_name}"
                drone.transit_remaining = 1
                drone.transit_conn = conn
                moves.append(f"{drone.id}-{drone.transit_connection}")
            else:
                execution_list.append(drone)
                next_hub.cur_occupancy += 1
                cur_hub.cur_occupancy -= 1
                conn.cur_occupancy += 1
                moves.append(f"{drone.id}-{next_name}")
        self.execution(execution_list)
        return moves
