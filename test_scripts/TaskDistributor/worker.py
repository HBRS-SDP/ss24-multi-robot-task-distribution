import zmq
import sys
import time

def worker_task(ident):
    # Create a ZeroMQ context
    context = zmq.Context()
    
    # Create a REQ socket (request-reply pattern) for worker communication
    socket = context.socket(zmq.REQ)
    
    # Set a unique identity for the worker and connect to the broker
    socket.identity = u"Worker-{}".format(ident).encode("ascii")
    socket.connect("tcp://localhost:5556")
    
    # Notify the broker that the worker is ready for work
    socket.send(b"READY")
    
    print(f"Worker {ident} is running...")
    
    while True:
        # Wait for a task from the broker
        address, empty, request = socket.recv_multipart()
        print(f"Worker {ident} received task: {request.decode('ascii')}")
        
  
          
        #  Add logic here for traveling to location and picking up an order
        time.sleep(2)  # Remove the delay after adding the logic
        
        result = f"Processed by Worker-{ident}: {request.decode('ascii')}"
        
        # Send the result message back 
        socket.send_multipart([address, b"", result.encode('ascii')])
        
        # Notify the broker that the worker is ready for new tasks
        #socket.send(b"READY")

if __name__ == "__main__":

    ident = sys.argv[1]
    worker_task(ident)

