import csv
import random

# Generate sample client IDs
client_ids = ['client{}'.format(i) for i in range(1, 101)]

# Generate sample shelves
shelves = ['A', 'B', 'C', 'D']

# Shelf capacity dictionary
shelf_capacity = {'A': 25, 'B': 25, 'C': 25, 'D': 25}

# Generate random sample input data
sample_data = []
for client_id in client_ids:
    # Randomly select a number of shelves for the order (1 to 4)
    num_shelves = random.randint(1, 4)
    
    # Randomly select shelves and quantities
    shelves_sample = random.sample(shelves, num_shelves)
    quantities_sample = [random.randint(1, shelf_capacity[shelf]) for shelf in shelves_sample]
    
    # Combine shelves and quantities into pairs
    order_items = [(shelf, quantity) for shelf, quantity in zip(shelves_sample, quantities_sample)]
    
    # Append the order to the sample data
    order = {'Client': client_id}
    for i, (shelf, quantity) in enumerate(order_items, start=1):
        order[f'Shelf_{i}'] = shelf
        order[f'Quantity_{i}'] = quantity
    sample_data.append(order)

# Write the sample data to a CSV file
with open('sample_orders.csv', 'w', newline='') as csvfile:
    fieldnames = ['Client'] + [f'Shelf_{i}' for i in range(1, 5)] + [f'Quantity_{i}' for i in range(1, 5)]
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    
    writer.writeheader()
    for order in sample_data:
        writer.writerow(order)
        print(order)

