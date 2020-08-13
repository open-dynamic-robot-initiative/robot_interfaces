About Time Series
=================

On Time Series and Time Relation of Actions and Observations
------------------------------------------------------------

All data transfer between the front end (= user code) and the back end
(= robot hardware) goes through so called time series. When calling
`append_desired_action(action)`, the action is not applied immediately
but is *appended* to the time series of desired actions. The back end
runs in a loop at around 1 kHz and in each iteration it takes the next
action from that time series and applies it on the robot. The time index
`t` returned by the function indicates the time step at which the given
action will be applied.

All the get methods (e.g. `get_observation()`) require a "time index" as
argument. If the specified time step has already passed, they
immediately return the value from the corresponding step. If it lies in
the future, the method will block and wait until the specified time step
is reached and then return. Note that only the last 1000 elements are
kept in the buffer. Trying to access an older time index results in an
exception.

### Time Relation of Actions and Observations

The time index `t` returned by `append_desired_action()` identifies the
time step at which the given action is sent to the robot, that is at
this moment the action *starts* to being applied. The observation that
is returned by `get_observation(t)` also belongs to this very moment.
This means the observation of step `t` belongs to the moment when the
action of step `t` just starts to being applied, that is this
observation is not yet affected by that action!

### Send Action to Start Backend

In the beginning of the program execution, the back end is idle and
waiting for the first action. Only after the first action is received,
the loop is started that applies actions and writes observations to the
time series.

This means **you first have to send an action before you can read the
first observation!**

There are applications where an observation is needed before sending the
first real action (e.g. when the action depends on the current
position). A safe solution in this case is to start with a zero-torque
action:

Python:

```{.py}
# an action without arguments defaults to zero torque
zero_torque_action = robot_interfaces.trifinger.Action()
t = frontend.append_desired_action(zero_torque_action)
first_observation = frontend.get_observation(t)
```

C++:

```{.cpp}
Action zero_torque_action = Action::Zero();
auto t = frontend.append_desired_action(zero_torque_action);
auto first_observation = frontend.get_observation(t);
```

### When Next Action Is Not Provided In Time {#next-action-not-in-time}

If the back end reaches a time step `t` but the user did not yet provide
an action for this time step (e.g. because the user code is running
slower than 1 kHz), the back end automatically sets the desired action
for step `t` to the same as the one of `t - 1`.

This is indicated to the user through the `action_repetitions` field in
the status message which contains the number of times the current action
has been repeated.


