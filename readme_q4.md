### Question 4

In this question, we develop a state machine using the following method:

- A separate class called StateManger manages the state for the entire drone mission.
- The StateManager maintains 1 of 7 possible states that is:
    - IDLE: The system begins in this state generating no control inputs.
    - TAKEOFF: The system begins to takeoff and goes to a desired hover location at 0.5m height.
    - HOVER1: The system maintains its location for a short duration of 3 seconds.
    - TRACK: The system tracks a given trajectory.
    - HOVER2: The system maintains its location for a short duration of 3 seconds.
    - LAND: The system begins its descent from its current location and reaches the ground.
    - COMPLETE: The mission has been executed successfully.
- The StateManager maintains an internal Finite State Machine architecture that allows the system to go to the next state only if the trajectory has been completely tracked for a given state and the drone has converged to the given location of the trajectory.
- The StateManager moves the state of the system by incrementing the current state to the next state once the current state has completed execution.