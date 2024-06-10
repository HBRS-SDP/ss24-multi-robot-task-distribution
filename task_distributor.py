import csv
import zmq
import time

class WarehouseManager:
    def __init__(self):
        self.robots = []

    def load_orders(self, csv_file):
        orders = []
        with open(csv_file, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                orders.append(row)
        return orders

    def handle_task_request(self, robot_id):
        if not self.orders:
            return None
        
        order = self.orders.pop(0)
        client_id = order['Client']
        shelves = [order[f'Shelf_{i}'] for i in range(1, 5)]
        item_quantities = [int(order[f'Quantity_{i}']) for i in range(1, 5)]
        
        return {
            'client_id': client_id,
            'shelves': shelves,
            'item_quantities': item_quantities
        }

    def start(self):
        self.orders = self.load_orders('data/sample_orders.csv')

        # Initialize the CSV file for logging tasks
        with open('data/robot_assignments.csv', 'w', newline='') as csvfile:
            fieldnames = ['timestamp', 'robot_id', 'client_id', 'shelves', 'item_quantities']
            self.writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            self.writer.writeheader()

        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind("tcp://*:5555")
        
        print("Task distributor ready to serve tasks")

        while True:
            message = socket.recv_json()
            robot_id = message['robot_id']
            task = self.handle_task_request(robot_id)
            
            if task:
                # Log the task assignment to the CSV file
                with open('data/robot_assignments.csv', 'a', newline='') as csvfile:
                    writer = csv.DictWriter(csvfile, fieldnames=self.writer.fieldnames)
                    writer.writerow({
                        'timestamp': time.time(),
                        'robot_id': robot_id,
                        'client_id': task['client_id'],
                        'shelves': task['shelves'],
                        'item_quantities': task['item_quantities']
                    })
                socket.send_json(task)
            else:
                socket.send_json({"error": "No tasks available"})

if __name__ == "__main__":
    manager = WarehouseManager()
    manager.start()

