.PHONY: wheel clean install uninstall

wheel:
	python setup.py bdist_wheel

install: wheel
	pip install $$(ls -t dist/*.whl | head -1)

uninstall:
	pip uninstall -y rdkimu

clean:
	rm -rf build dist *.egg-info