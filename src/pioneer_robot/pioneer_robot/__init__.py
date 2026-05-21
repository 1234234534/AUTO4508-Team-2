from rosbag2_interfaces.srv import Snapshot

self._snapshot_client = self.create_client(
    Snapshot,
    '/rosbag2_recorder/snapshot' # VERIFY ACTUAL TOPIC WHEN RUNNING
)

self._estop_snapshot_armed = True