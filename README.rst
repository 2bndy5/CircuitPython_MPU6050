Introduction
============

.. image:: https://readthedocs.org/projects/circuitpython-mpu6050/badge/?version=latest
    :target: https://circuitpython-mpu6050.readthedocs.io/en/latest/?badge=latest
    :alt: Documentation Status

.. image:: https://travis-ci.org/2bndy5/CircuitPython_MPU6050.svg?branch=master
    :target: https://travis-ci.org/2bndy5/CircuitPython_MPU6050
    :alt: Build Status

A port of `the python library for the MPU6050 <https://github.com/Tijndagamer/mpu6050.git>`_ 6 Degrees of Freedom sensor to CircuitPython using the Adafruit_BusDevice library.

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

How to Install
=====================
This library isn't getting deployed to pypi.org as `adadruit has a created a library <https://pypi.org/project/adafruit-circuitpython-mpu6050/>`_ unbeknownst to me while I was developing this library. Use the following commands to install this library:

.. code-block:: shell

    git clone https://github.com/2bndy5/CircuitPython_MPU6050.git
    cd CircuitPython_MPU6050
    python3 setup.py install

To install globally, prefix the last command with ``sudo``.

Usage Example
=============

Ensure you've connected the MPU6050 correctly by running the simple test located in the `examples folder of this library <https://github.com/2bndy5/CircuitPython_MPU6050/tree/master/examples>`_. See also the `examples/` section.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/2bndy5/CircuitPython_MPU6050/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.
