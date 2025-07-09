# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
from abc import ABC, abstractmethod
from asyncio import Semaphore
from typing import Tuple

from .common import Config

logger = logging.getLogger(__name__)
config = Config.get()


class ConfigUpdatable(ABC):
    """Interface for components that need to be updated when config changes"""

    # Subclasses must declare this field
    semaphore_: Semaphore

    @classmethod
    def update_from_config(cls):
        """Update this component with the latest config values"""
        # Get the current semaphore value
        current_value = cls.semaphore_._value
        # Get the new value from config using subclass-specific keys
        section, key = cls.get_setting_keys()
        new_value = config.getint(section, key)

        if current_value != new_value:
            # Create a new semaphore with the updated value
            cls.semaphore_ = Semaphore(value=new_value)
            logger.info(
                f"Updated {cls.__name__} semaphore from {current_value} to {new_value}"
            )

    @classmethod
    @abstractmethod
    def get_setting_keys(cls) -> Tuple[str, str]:
        """Return the config section and key for this component's semaphore setting"""
        pass


def update_all_components():
    """Update all components semaphores with the latest config values"""
    # Import all classes that need semaphores config updates
    from .downloader import Downloader
    from .encryption import VrsEncryptor
    from .hash_calculator import HashCalculator
    from .health_check import HealthCheckRunner
    from .uploader import Uploader

    components = [
        Uploader,
        Downloader,
        VrsEncryptor,
        HealthCheckRunner,
        HashCalculator,
    ]

    for component_class in components:
        component_class.update_from_config()
