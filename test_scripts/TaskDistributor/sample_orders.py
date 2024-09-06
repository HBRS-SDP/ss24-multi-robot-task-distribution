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
    order = {'Client': client_id}
    
    # Ensure that each client has items in all 4 shelves
    for i, shelf in enumerate(shelves, start=1):
        quantity = random.randint(1, shelf_capacity[shelf])
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

