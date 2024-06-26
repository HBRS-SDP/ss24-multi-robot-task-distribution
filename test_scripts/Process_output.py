import csv

# Define shelf locations and their coordinates
shelf_locations = {'A': (0, 0), 'B': (0, 1), 'C': (1, 0), 'D': (1, 1)}

# Read the sample orders from the CSV file
orders = []
with open('sample_orders.csv', 'r') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        orders.append(row)

# Initialize robots and current robot index
robots = ['Robot1', 'Robot2', 'Robot3', 'Robot4']
current_robot_index = 0

# Process each order and assign robots
process_log = []
for order in orders:
    client_id = order['Client']
    order_items = [(order[f'Shelf_{i}'], int(order[f'Quantity_{i}'])) for i in range(1, 5) if order[f'Shelf_{i}'] and order[f'Quantity_{i}']]
    
    # Assign a robot for the client in a round-robin fashion
    robot_id = robots[current_robot_index % len(robots)]
    current_robot_index += 1
    
    # Add each shelf item to the process log
    for shelf, quantity in order_items:
        x, y = shelf_locations[shelf]
        process_log.append({'Client ID': client_id, 'Robot ID': robot_id, 'Shelf': shelf, 'Quantity': quantity, 'X': x, 'Y': y})

# Write the process_log  to an output CSV file
with open('process_log.csv', 'w', newline='') as csvfile:
    fieldnames = ['Client ID', 'Robot ID', 'Shelf', 'Quantity', 'X', 'Y']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    
    writer.writeheader()
    for log_entry in process_log:
        writer.writerow(log_entry)
        print(log_entry)

