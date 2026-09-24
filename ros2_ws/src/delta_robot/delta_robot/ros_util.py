"""Small rclpy helpers shared by the nodes (clean start-up / shutdown on Humble and Jazzy)."""

from __future__ import annotations

from typing import Any, Callable, Optional

import rclpy

try:  # Humble+ ; guarded for older/odd installs
    from rclpy.executors import ExternalShutdownException
except ImportError:  # pragma: no cover
    class ExternalShutdownException(Exception):  # type: ignore[no-redef]
        pass


def run_node(factory: Callable[[], Any], args: Optional[list] = None) -> None:
    """rclpy.init -> spin(factory()) -> node.shutdown() (if any) -> destroy -> rclpy.shutdown."""
    rclpy.init(args=args)
    node: Any = None
    try:
        node = factory()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            stop = getattr(node, "shutdown", None)
            if callable(stop):
                try:
                    stop()
                except Exception as e:  # noqa: BLE001 - never block shutdown
                    print("shutdown error: %s" % e)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
