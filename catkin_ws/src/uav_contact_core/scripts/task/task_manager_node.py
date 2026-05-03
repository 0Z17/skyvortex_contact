#!/usr/bin/env python3

import rospy
from uav_contact_msgs.msg import TaskPhase, SafetyState


class TaskManagerNode:
    def __init__(self):
        rospy.init_node("task_manager", anonymous=False)

        self.rate_hz = rospy.get_param("/task_manager/rate", 20.0)
        self.auto_start = rospy.get_param("/task_manager/auto_start", False)
        self.stabilize_duration = rospy.get_param("/task_manager/stabilize_duration", 5.0)
        self.approach_duration = rospy.get_param("/task_manager/approach_duration", 30.0)
        self.initial_contact_duration = rospy.get_param("/task_manager/initial_contact_duration", 5.0)
        self.retreat_duration = rospy.get_param("/task_manager/retreat_duration", 20.0)
        self.gate_on_offboard_ready = bool(rospy.get_param("/task_manager/gate_on_offboard_ready", True))
        self.offboard_ready_hold_sec = float(rospy.get_param("/task_manager/offboard_ready_hold_sec", 0.5))

        self.phase = TaskPhase.IDLE
        self.phase_start_time = rospy.Time.now()
        self.emergency_requested = False
        self.safety_safe = False
        self.offboard_ready_since = None

        self.phase_pub = rospy.Publisher("/uav_contact/task/phase", TaskPhase, queue_size=10)
        self.safety_sub = rospy.Subscriber("/uav_contact/safety/state", SafetyState, self._on_safety_state)

        rospy.loginfo("Task manager node started, phase=IDLE")

    def _on_safety_state(self, msg):
        self.safety_safe = bool(msg.safe)
        if self.gate_on_offboard_ready and self.safety_safe:
            if self.offboard_ready_since is None:
                self.offboard_ready_since = rospy.Time.now()
        else:
            self.offboard_ready_since = None

        if msg.require_emergency_retreat and self.phase not in (
            TaskPhase.EMERGENCY_RETREAT, TaskPhase.ERROR, TaskPhase.FINISHED
        ):
            rospy.logwarn("Emergency retreat requested by safety monitor: %s", msg.reason)
            self.emergency_requested = True
            self.phase = TaskPhase.EMERGENCY_RETREAT
            self.phase_start_time = rospy.Time.now()

    def _phase_duration(self):
        return {
            TaskPhase.STABILIZE: self.stabilize_duration,
            TaskPhase.APPROACH: self.approach_duration,
            TaskPhase.INITIAL_CONTACT: self.initial_contact_duration,
            TaskPhase.SLIDING_CONTACT: float("inf"),
            TaskPhase.RETREAT: self.retreat_duration,
            TaskPhase.EMERGENCY_RETREAT: 10.0,
        }

    def _transition_map(self):
        return {
            TaskPhase.IDLE: TaskPhase.STABILIZE,
            TaskPhase.STABILIZE: TaskPhase.APPROACH,
            TaskPhase.APPROACH: TaskPhase.INITIAL_CONTACT,
            TaskPhase.INITIAL_CONTACT: TaskPhase.SLIDING_CONTACT,
            TaskPhase.SLIDING_CONTACT: TaskPhase.RETREAT,
            TaskPhase.RETREAT: TaskPhase.FINISHED,
        }

    def _phase_enables(self):
        return {
            TaskPhase.IDLE: (False, False, False, False),
            TaskPhase.STABILIZE: (False, False, False, True),
            TaskPhase.APPROACH: (True, False, True, True),
            TaskPhase.INITIAL_CONTACT: (True, True, True, True),
            TaskPhase.SLIDING_CONTACT: (True, True, True, True),
            TaskPhase.RETREAT: (True, False, True, True),
            TaskPhase.EMERGENCY_RETREAT: (False, False, False, True),
            TaskPhase.FINISHED: (False, False, False, False),
            TaskPhase.ERROR: (False, False, False, False),
        }

    def _offboard_ready(self):
        if not self.gate_on_offboard_ready:
            return True
        if self.offboard_ready_since is None:
            return False
        return (rospy.Time.now() - self.offboard_ready_since).to_sec() >= self.offboard_ready_hold_sec

    def _update_phase(self):
        if self.phase in (TaskPhase.FINISHED, TaskPhase.ERROR):
            return

        if self.phase == TaskPhase.EMERGENCY_RETREAT:
            duration = self._phase_duration().get(TaskPhase.EMERGENCY_RETREAT, 10.0)
            elapsed = (rospy.Time.now() - self.phase_start_time).to_sec()
            if elapsed >= duration:
                self.phase = TaskPhase.FINISHED
                rospy.loginfo("Emergency retreat completed, moving to FINISHED")
            return

        if self.phase == TaskPhase.IDLE and self.auto_start:
            if self._offboard_ready():
                self.phase = TaskPhase.STABILIZE
                self.phase_start_time = rospy.Time.now()
                rospy.loginfo("Auto-starting: STABILIZE")
            return

        if self.phase == TaskPhase.SLIDING_CONTACT:
            return

        duration = self._phase_duration().get(self.phase)
        elapsed = (rospy.Time.now() - self.phase_start_time).to_sec()
        if duration and elapsed >= duration:
            next_phase = self._transition_map().get(self.phase)
            if next_phase is not None:
                if next_phase == TaskPhase.APPROACH and not self._offboard_ready():
                    return
                self.phase = next_phase
                self.phase_start_time = rospy.Time.now()
                rospy.loginfo("Phase transition: %s", self._phase_to_name(next_phase))

    def _phase_to_name(self, phase):
        names = {
            TaskPhase.IDLE: "IDLE",
            TaskPhase.STABILIZE: "STABILIZE",
            TaskPhase.APPROACH: "APPROACH",
            TaskPhase.INITIAL_CONTACT: "INITIAL_CONTACT",
            TaskPhase.SLIDING_CONTACT: "SLIDING_CONTACT",
            TaskPhase.RETREAT: "RETREAT",
            TaskPhase.EMERGENCY_RETREAT: "EMERGENCY_RETREAT",
            TaskPhase.FINISHED: "FINISHED",
            TaskPhase.ERROR: "ERROR",
        }
        return names.get(phase, "UNKNOWN")

    def _publish_phase(self):
        enables = self._phase_enables().get(self.phase, (False, False, False, False))
        msg = TaskPhase()
        msg.header.stamp = rospy.Time.now()
        msg.phase = self.phase
        msg.elapsed_time = (rospy.Time.now() - self.phase_start_time).to_sec()
        msg.enable_trajectory = enables[0]
        msg.enable_contact_control = enables[1]
        msg.enable_servo = enables[2]
        msg.enable_uav_control = enables[3]
        msg.description = self._phase_to_name(self.phase)
        self.phase_pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._update_phase()
            self._publish_phase()
            rate.sleep()


def main():
    node = TaskManagerNode()
    node.spin()


if __name__ == "__main__":
    main()
