class RobotState:
    def __init__(self, ID):
        self.id = ID
        self.available = True
        self.clientID = None
        self.shelves = None
        self.items = None
        self.current_pose = None
        self.goal = None
        self.pickup = None
        self.start_time = None
        self.pickup_time = None
        self.drop_time = None

    def update_availability(self, availability):
        self.available = availability

    def set_goal(self, destination, goal_pose):
        self.goal = {'x': goal_pose['x'], 'y':goal_pose['y'], 'z':goal_pose['z'], 'w':goal_pose['w'],
                     'shelf': destination, 'items': self.items[destination]}

    def assign_order(self, clientID, shelves, item_quantities):
        self.clientID = clientID
        self.shelves = shelves
        self.shelves.append('drop_counter')
        item_quantities.append(sum(item_quantities))
        self.items = dict(zip(shelves, item_quantities))
