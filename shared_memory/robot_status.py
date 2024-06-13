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

    def set_goal(self, shelf, goal_pose, drop=False):
        if drop:
            self.goal = {'x': goal_pose['x'], 'y': goal_pose['y'], 'w': goal_pose['w'], 'shelf': 'drop_counter',
                         'items': self.items[shelf]}
        else:
            self.goal = {'x': goal_pose['x'], 'y':goal_pose['y'], 'w':goal_pose['w'], 'shelf': shelf,
                         'items': self.items[shelf]}
            self.pickup = shelf

    def assign_order(self, clientID, shelves, item_quantities):
        self.clientID = clientID
        self.shelves = shelves
        self.items = dict(zip(shelves, item_quantities))
