#!/usr/bin/env python3
"""Structured JSON logging for ROS 2 Python nodes."""

import json
import logging
import sys
import time
import uuid
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Dict, Optional

import structlog


class LogLevel(Enum):
    """Log level enumeration."""

    DEBUG = "DEBUG"
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"
    FATAL = "FATAL"


class Component(Enum):
    """Component enumeration for structured logging."""

    STARTUP = "startup"
    TIMER = "timer"
    SUBSCRIBER = "subscriber"
    HEALTH = "health"
    SHUTDOWN = "shutdown"


class EventType(Enum):
    """Event type enumeration for structured logging."""

    PUBLISH = "publish"
    RECEIVE = "receive"
    HEALTH_CHECK = "health_check"
    INITIALIZATION = "initialization"
    PARAMETER_UPDATE = "parameter_update"
    ERROR_OCCURRED = "error"


class StructuredLogger:
    """Structured JSON logger for ROS 2 nodes."""

    def __init__(self, node_name: str):
        """Initialize the structured logger.

        Args:
            node_name: Name of the ROS 2 node
        """
        self.node_name = node_name
        self.correlation_id = str(uuid.uuid4())[:8]

        # Configure structlog
        structlog.configure(
            processors=[
                structlog.stdlib.filter_by_level,
                structlog.stdlib.add_logger_name,
                structlog.stdlib.add_log_level,
                structlog.stdlib.PositionalArgumentsFormatter(),
                structlog.processors.TimeStamper(fmt="iso"),
                structlog.processors.StackInfoRenderer(),
                structlog.processors.format_exc_info,
                structlog.processors.UnicodeDecoder(),
                structlog.processors.JSONRenderer(),
            ],
            context_class=dict,
            logger_factory=structlog.stdlib.LoggerFactory(),
            wrapper_class=structlog.stdlib.BoundLogger,
            cache_logger_on_first_use=True,
        )

        # Setup standard logging
        logging.basicConfig(
            format="%(message)s",
            stream=sys.stdout,
            level=logging.DEBUG,
        )

        self.logger = structlog.get_logger(node_name)

    def log(
        self,
        level: LogLevel,
        component: Component,
        event_type: EventType,
        message: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Log a structured message.

        Args:
            level: Log level
            component: Component generating the log
            event_type: Type of event being logged
            message: Human-readable message
            context: Additional context data
        """
        if context is None:
            context = {}

        log_data = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "level": level.value,
            "node_name": self.node_name,
            "component": component.value,
            "event_type": event_type.value,
            "message": message,
            "context": context,
            "correlation_id": self.correlation_id,
        }

        # Log at appropriate level
        if level == LogLevel.DEBUG:
            self.logger.debug("", **log_data)
        elif level == LogLevel.INFO:
            self.logger.info("", **log_data)
        elif level == LogLevel.WARN:
            self.logger.warning("", **log_data)
        elif level == LogLevel.ERROR:
            self.logger.error("", **log_data)
        elif level == LogLevel.FATAL:
            self.logger.critical("", **log_data)

    def debug(
        self,
        component: Component,
        event_type: EventType,
        message: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Log a debug message."""
        self.log(LogLevel.DEBUG, component, event_type, message, context)

    def info(
        self,
        component: Component,
        event_type: EventType,
        message: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Log an info message."""
        self.log(LogLevel.INFO, component, event_type, message, context)

    def warn(
        self,
        component: Component,
        event_type: EventType,
        message: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Log a warning message."""
        self.log(LogLevel.WARN, component, event_type, message, context)

    def error(
        self,
        component: Component,
        event_type: EventType,
        message: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Log an error message."""
        self.log(LogLevel.ERROR, component, event_type, message, context)

    def fatal(
        self,
        component: Component,
        event_type: EventType,
        message: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Log a fatal message."""
        self.log(LogLevel.FATAL, component, event_type, message, context)
