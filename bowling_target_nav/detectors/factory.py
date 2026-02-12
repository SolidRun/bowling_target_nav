"""
Detector Factory
================

Factory pattern implementation for creating detectors based on configuration.
Provides a clean way to instantiate the correct detector type.
"""

import logging
from typing import Dict, Optional, Type, Any

from .base import DetectorBase
from .yolo_onnx_detector import YoloOnnxDetector
from .drp_binary_detector import DrpBinaryDetector
from .mock_detector import MockDetector

logger = logging.getLogger(__name__)


class DetectorFactory:
    """
    Factory for creating detector instances.

    Uses Factory pattern to create the appropriate detector based on
    configuration or explicit type specification.

    Usage:
        # Create from config
        from bowling_target_nav.core import get_config
        config = get_config()
        detector = DetectorFactory.create_from_config(config)

        # Create by type
        detector = DetectorFactory.create("yolo_onnx", model_path="model.onnx")

        # Register custom detector
        DetectorFactory.register("my_detector", MyDetectorClass)
    """

    # Registry of detector types
    _registry: Dict[str, Type[DetectorBase]] = {
        'yolo_onnx': YoloOnnxDetector,
        'drp_binary': DrpBinaryDetector,
        'mock': MockDetector,
    }

    # Aliases for convenience
    _aliases: Dict[str, str] = {
        'yolo': 'yolo_onnx',
        'onnx': 'yolo_onnx',
        'drp': 'drp_binary',
        'binary': 'drp_binary',
        'v2n': 'drp_binary',
        'test': 'mock',
        'fake': 'mock',
    }

    @classmethod
    def register(cls, name: str, detector_class: Type[DetectorBase]) -> None:
        """
        Register a new detector type.

        Args:
            name: Unique name for the detector type
            detector_class: Detector class (must inherit from DetectorBase)
        """
        if not issubclass(detector_class, DetectorBase):
            raise TypeError(f"{detector_class} must inherit from DetectorBase")

        cls._registry[name] = detector_class
        logger.info(f"Registered detector type: {name}")

    @classmethod
    def unregister(cls, name: str) -> None:
        """
        Unregister a detector type.

        Args:
            name: Name of detector type to remove
        """
        if name in cls._registry:
            del cls._registry[name]

    @classmethod
    def get_available_types(cls) -> list:
        """Get list of available detector types."""
        return list(cls._registry.keys())

    @classmethod
    def _resolve_type(cls, detector_type: str) -> str:
        """Resolve type name including aliases."""
        detector_type = detector_type.lower().strip()

        # Check aliases
        if detector_type in cls._aliases:
            detector_type = cls._aliases[detector_type]

        return detector_type

    @classmethod
    def create(cls, detector_type: str, **kwargs) -> DetectorBase:
        """
        Create a detector instance.

        Args:
            detector_type: Type of detector ("yolo_onnx", "drp_binary", "mock")
            **kwargs: Additional arguments passed to detector constructor

        Returns:
            Initialized detector instance

        Raises:
            ValueError: If detector type is not registered
        """
        resolved_type = cls._resolve_type(detector_type)

        if resolved_type not in cls._registry:
            available = ', '.join(cls._registry.keys())
            raise ValueError(
                f"Unknown detector type: '{detector_type}'. "
                f"Available types: {available}"
            )

        detector_class = cls._registry[resolved_type]
        logger.info(f"Creating detector: {detector_class.__name__}")

        return detector_class(**kwargs)

    @classmethod
    def create_from_config(cls, config: Any) -> DetectorBase:
        """
        Create a detector from configuration object.

        Args:
            config: Configuration object with detection settings

        Returns:
            Initialized detector instance
        """
        det_config = config.detection
        detector_type = det_config.detector_type

        resolved_type = cls._resolve_type(detector_type)

        # Build kwargs based on detector type
        kwargs = {
            'confidence_threshold': det_config.confidence_threshold,
            'target_class': det_config.target_class,
        }

        if resolved_type == 'yolo_onnx':
            kwargs.update({
                'model_path': det_config.yolo_model_path,
                'input_size': det_config.yolo_input_size,
                'class_names': det_config.yolo_class_names,
            })

        elif resolved_type == 'drp_binary':
            kwargs.update({
                'binary_path': det_config.drp_binary_path,
                'input_method': det_config.drp_input_method,
                'input_path': det_config.drp_input_path,
                'output_method': det_config.drp_output_method,
                'output_path': det_config.drp_output_path,
                'timeout': det_config.drp_timeout,
                'class_names': det_config.yolo_class_names,  # Reuse class names
            })

        elif resolved_type == 'mock':
            # Mock detector uses defaults
            pass

        return cls.create(resolved_type, **kwargs)

    @classmethod
    def create_and_initialize(
        cls,
        detector_type: str,
        **kwargs
    ) -> Optional[DetectorBase]:
        """
        Create and initialize a detector.

        Args:
            detector_type: Type of detector
            **kwargs: Additional arguments for detector

        Returns:
            Initialized detector, or None if initialization failed
        """
        detector = cls.create(detector_type, **kwargs)

        if detector.initialize():
            return detector

        logger.error(f"Failed to initialize {detector_type} detector")
        return None


# Convenience function
def create_detector(
    detector_type: str = "yolo_onnx",
    config: Any = None,
    auto_initialize: bool = True,
    **kwargs
) -> Optional[DetectorBase]:
    """
    Convenience function to create a detector.

    Args:
        detector_type: Type of detector (ignored if config provided)
        config: Configuration object (optional)
        auto_initialize: Whether to initialize detector
        **kwargs: Additional arguments for detector

    Returns:
        Detector instance (initialized if auto_initialize=True)

    Examples:
        # Simple creation
        detector = create_detector("yolo_onnx", model_path="model.onnx")

        # From config
        detector = create_detector(config=my_config)

        # Mock for testing
        detector = create_detector("mock", detection_mode="always")
    """
    if config is not None:
        detector = DetectorFactory.create_from_config(config)
    else:
        detector = DetectorFactory.create(detector_type, **kwargs)

    if auto_initialize:
        if not detector.initialize():
            logger.error("Detector initialization failed")
            return None

    return detector
