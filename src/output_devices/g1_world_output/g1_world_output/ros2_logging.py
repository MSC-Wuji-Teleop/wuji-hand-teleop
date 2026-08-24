"""Re-export from pico_input (canonical source for ROS2 logging bridge)."""
try:
    from pico_input.ros2_logging import *  # noqa: F401,F403
except ImportError:
    # Fallback when pico_input is not on PYTHONPATH (standalone tests).
    import logging

    class ROS2LoggerAdapter:
        def __init__(self, ros_logger):
            self._logger = ros_logger

        def debug(self, msg, *args, **kwargs):
            self._logger.debug(msg % args if args else msg)

        def info(self, msg, *args, **kwargs):
            self._logger.info(msg % args if args else msg)

        def warning(self, msg, *args, **kwargs):
            self._logger.warning(msg % args if args else msg)

        def error(self, msg, *args, **kwargs):
            self._logger.error(msg % args if args else msg)

    def setup_ros2_logging_bridge(ros_logger):
        logging.getLogger().handlers.clear()
        return ROS2LoggerAdapter(ros_logger)
