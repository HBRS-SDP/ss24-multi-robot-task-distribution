# Task Distribution Overview

In this multi-robot system, task distribution is handled by the broker using ZMQ. The broker ensures that tasks from the client are evenly distributed among workers (robots), based on their availability. Here's the core concept of the task distribution algorithm:

## Task Distribution Workflow

### Initialization:
- The broker starts and binds two sockets:
  - **Frontend Socket (port 5555)**: To handle client requests (tasks).
  - **Backend Socket (port 5556)**: To manage communication with the workers.
- A poller is initialized to monitor both sockets for incoming messages.
- The broker maintains two lists:
  - `workers[]`: Tracks available workers (robots) that are ready to take on tasks.
  - `tasks[]`: Queues incoming tasks from the client.

### Worker Availability :
- Each worker, upon startup, sends a `READY` message to the broker. This signals the broker that the worker is available for tasks.
- The broker adds the worker to the `workers[]` list.

**Key Event**: Worker sends `READY` → Broker adds worker to the available workers list.

### Task Arrival (Client Sends a Task):
- The client submits a task to the broker via the **frontend socket**.
- The task is stored in the `tasks[]` queue until a worker becomes available.

**Key Event**: Client sends a task → Broker adds task to the task queue.

### Task Assignment:
- The broker continually monitors both the `tasks[]` queue and the `workers[]` list.
- When both a task and a worker are available, the broker assigns the task to the next available worker.
- **Task Assignment Strategy**:
  - The broker selects the first available worker from the `workers[]` list (FIFO).
  - The broker removes the worker from the available list and assigns the task.

**Key Event**: Task is available + Worker is available → Broker sends task to worker.

### Task Completion:
- Once a worker completes a task, it sends the result back to the broker.
- The broker forwards this result to the client and adds the worker back to the `workers[]` list, marking it as ready for a new task.

**Key Event**: Worker completes task → Broker receives result and marks worker as available.

### Continuous Polling:
- The broker continuously polls both the **frontend** (for tasks) and **backend** (for worker availability and task results).
- The cycle repeats, ensuring that tasks are distributed evenly among workers, and no worker is overloaded.
