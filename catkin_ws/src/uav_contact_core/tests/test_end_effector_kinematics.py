from pathlib import Path
import importlib.util


def _load_end_effector_module():
    module_path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "kinematics"
        / "end_effector_kinematics_node.py"
    )
    spec = importlib.util.spec_from_file_location("end_effector_kinematics_node", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_state_contains_pose_twist_and_contact_fields():
    module = _load_end_effector_module()
    state = module.build_state()

    assert "position" in state
    assert "orientation" in state
    assert "linear_velocity" in state
    assert "angular_velocity" in state
    assert "normal_velocity" in state
    assert "contact_error" in state


def test_build_and_publish_emits_state_once_with_fake_publisher():
    module = _load_end_effector_module()

    class FakePublisher:
        def __init__(self):
            self.published = []

        def publish(self, message):
            self.published.append(message)

    fake_publisher = FakePublisher()
    node = module.EndEffectorKinematicsNode(publisher=fake_publisher)

    state = node.build_and_publish()

    assert len(fake_publisher.published) == 1
    assert fake_publisher.published[0] == state
