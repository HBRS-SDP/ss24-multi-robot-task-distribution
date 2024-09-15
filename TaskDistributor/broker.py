import zmq

def main():
    context = zmq.Context()
    
    frontend = context.socket(zmq.ROUTER)
    frontend.bind("tcp://*:5555")  # Bind frontend to port 5555 for client connections
    
    backend = context.socket(zmq.ROUTER)
    backend.bind("tcp://*:5556")  # Bind backend to port 5556 for worker connections

    poller = zmq.Poller()
    poller.register(frontend, zmq.POLLIN)  # Monitor frontend for incoming messages
    poller.register(backend, zmq.POLLIN)   # Monitor backend for incoming messages

    workers = []

    print("Broker is running...")

    while True:
        socks = dict(poller.poll())  # Poll sockets for events

        if backend in socks:
            message = backend.recv_multipart()
            worker, empty, client = message[:3]
            workers.append(worker)  # Add the worker to the list of available workers
            if client != b"READY":
                empty, reply = message[3:]
                frontend.send_multipart([client, b"", reply])
        
        if frontend in socks and workers:
            client, empty, request = frontend.recv_multipart()
            worker = workers.pop(0)  # Get the next available worker
            backend.send_multipart([worker, b"", client, b"", request])

if __name__ == "__main__":
    main()

