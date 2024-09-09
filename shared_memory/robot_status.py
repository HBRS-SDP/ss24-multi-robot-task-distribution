from datetime import datetime


class RobotState:
    def __init__(self, ID):
        self.id = ID
        self.clientID = None
        self.shelf = None
        self.items = None
        self.current_pose = None
        self.start_time = None
        self.end_time = None
        self.eta = None
        self.saving = False

    def assign_task(self, client_id, shelf, items):
        self.clientID = client_id
        self.shelf = shelf
        self.items = items
        self.start_time = datetime.now()

    def reset(self):
        self.clientID = None
        self.shelf = None
        self.items = None
        self.current_pose = None
        self.start_time = None
        self.end_time = None
        self.eta = None