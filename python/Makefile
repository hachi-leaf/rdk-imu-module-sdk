# Copyright (c) 2026 Leaf. D-Robotics.
# SPDX-License-Identifier: MIT
.PHONY: wheel clean install uninstall

wheel:
	python setup.py bdist_wheel

install: wheel
	pip install --force-reinstall $$(ls -t dist/*.whl | head -1)

uninstall:
	pip uninstall -y rdkimu

clean:
	rm -rf build dist *.egg-info