import zmq
import csv
import sys

def client_task(csv_file):
    # create a ZeroMQ context
    context = zmq.Context()
    
    # create a REQ socket (request-reply pattern) for client communication
    socket = context.socket(zmq.REQ)
    
    # connect to the broker  
    socket.connect("tcp://localhost:5555")
    
    print("Client is running...")
    
    # Open the csv file containing orders
    with open(csv_file, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        
        # Iterate through each order in the CSV file
        for row in reader:
            client_id = row['Client']
            # Prepare the task message to send to the broker
            task = {
            'client_id': client_id,
            'shelves': [row[f'Shelf_{i}'] for i in range(1, 5)],
            'item_quantities': [int(row[f'Quantity_{i}']) for i in range(1, 5)]}


            # send the task message to the broker
            socket.send_string(str(task))
            
            # Wait for the broker to send back a response
            reply = socket.recv_string()
            # Print the reply from the broker
            print(f"{client_id}: {reply}")

if __name__ == "__main__":
    # load the CSV file 
    csv_file = sys.argv[1]
    # Run the client task function with the provided CSV file
    client_task(csv_file)

