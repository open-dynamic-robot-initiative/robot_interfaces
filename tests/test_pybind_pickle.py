"""Unit tests verifying that pickling of bound types works as expected."""
import pickle
import numpy as np

import robot_interfaces


def test_pickle_finger_action():
    action = robot_interfaces.finger.Action(
        torque=[1, 2, 3],
        position=[4, 5, 6],
        position_kp=[0.1, 0.2, 0.3],
        position_kd=[11, 22, 33],
    )

    pickled = pickle.dumps(action)
    unpickled = pickle.loads(pickled)

    np.testing.assert_array_equal(action.torque, unpickled.torque)
    np.testing.assert_array_equal(action.position, unpickled.position)
    np.testing.assert_array_equal(action.position_kp, unpickled.position_kp)
    np.testing.assert_array_equal(action.position_kd, unpickled.position_kd)


def test_pickle_trifinger_action():
    action = robot_interfaces.trifinger.Action(
        torque=[1, 2, 3, 4, 5, 6, 7, 8, 9],
        position=[11, 22, 33, 44, 55, 66, 77, 88, 99],
        position_kp=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9],
        position_kd=[1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9],
    )

    pickled = pickle.dumps(action)
    unpickled = pickle.loads(pickled)

    np.testing.assert_array_equal(action.torque, unpickled.torque)
    np.testing.assert_array_equal(action.position, unpickled.position)
    np.testing.assert_array_equal(action.position_kp, unpickled.position_kp)
    np.testing.assert_array_equal(action.position_kd, unpickled.position_kd)


def test_pickle_finger_observation():
    obs = robot_interfaces.finger.Observation()
    obs.torque = [1, 2, 3]
    obs.velocity = [11, 22, 33]
    obs.position = [111, 222, 333]
    obs.tip_force = [1111]

    pickled = pickle.dumps(obs)
    unpickled = pickle.loads(pickled)

    np.testing.assert_array_equal(obs.torque, unpickled.torque)
    np.testing.assert_array_equal(obs.velocity, unpickled.velocity)
    np.testing.assert_array_equal(obs.position, unpickled.position)
    np.testing.assert_array_equal(obs.tip_force, unpickled.tip_force)


def test_pickle_trifinger_observation():
    obs = robot_interfaces.trifinger.Observation()
    obs.torque = [1, 2, 3, 4, 5, 6, 7, 8, 9]
    obs.velocity = [11, 22, 33, 44, 55, 66, 77, 88, 99]
    obs.position = [111, 222, 333, 444, 555, 666, 777, 888, 999]
    obs.tip_force = [1111, 2222, 3333]

    pickled = pickle.dumps(obs)
    unpickled = pickle.loads(pickled)

    np.testing.assert_array_equal(obs.torque, unpickled.torque)
    np.testing.assert_array_equal(obs.velocity, unpickled.velocity)
    np.testing.assert_array_equal(obs.position, unpickled.position)
    np.testing.assert_array_equal(obs.tip_force, unpickled.tip_force)


def test_pickle_one_joint_observation():
    # one_joint observations do not have a "tip_force"
    obs = robot_interfaces.one_joint.Observation()
    obs.torque = [1]
    obs.velocity = [11]
    obs.position = [111]

    pickled = pickle.dumps(obs)
    unpickled = pickle.loads(pickled)

    np.testing.assert_array_equal(obs.torque, unpickled.torque)
    np.testing.assert_array_equal(obs.velocity, unpickled.velocity)
    np.testing.assert_array_equal(obs.position, unpickled.position)


def test_pickle_status():
    status = robot_interfaces.Status()
    status.action_repetitions = 42
    status.set_error(status.ErrorStatus.DRIVER_ERROR, "some message")

    pickled = pickle.dumps(status)
    unpickled = pickle.loads(pickled)

    assert unpickled.action_repetitions == 42
    assert unpickled.error_status == status.ErrorStatus.DRIVER_ERROR
    assert unpickled.get_error_message() == "some message"
