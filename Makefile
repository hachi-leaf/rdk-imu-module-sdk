# Copyright (c) 2026 Leaf. D-Robotics.
# SPDX-License-Identifier: MIT
.PHONY: all core python ros2 install install-core install-python clean

CORE_DIR   = core
PYTHON_DIR = python
ROS2_DIR   = ros2

all: core python ros2

core:
	$(MAKE) -C $(CORE_DIR)

python: core
	$(MAKE) -C $(PYTHON_DIR)

ros2: core
ifdef ROS_DISTRO
	cd $(ROS2_DIR) && colcon build
else
	$(info [WARNING] ROS 2 environment not sourced, skipping ROS 2 build.)
	$(info [WARNING] To build ROS 2 package, run: source /opt/ros/$$ROS_DISTRO/setup.bash && make ros2)
endif

install-core:
	$(MAKE) -C $(CORE_DIR) install

install-python:
	$(MAKE) -C $(PYTHON_DIR) install

install: install-core install-python
	@echo "All components installed. For ROS 2, source $(ROS2_DIR)/install/setup.bash"

clean:
	$(MAKE) -C $(CORE_DIR) clean
	$(MAKE) -C $(PYTHON_DIR) clean
	rm -rf $(ROS2_DIR)/build $(ROS2_DIR)/install $(ROS2_DIR)/log

uninstall:
	$(MAKE) -C $(CORE_DIR) uninstall
	$(MAKE) -C $(PYTHON_DIR) uninstall