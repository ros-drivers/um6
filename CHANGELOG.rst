^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package um6
^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Specify non-max inter-byte timeout to avoid 32-bit problems in serial.
* Switch to %zd for logging size_t values.
* Be more optimistic about finding packets back-to-back, rather than expecting inter-packet junk bytes.
* fix euler angle rpy so that it is published in ENU

0.0.1 (2013-08-30)
------------------
* Initial release of um6 driver. Mostly feature parity with Python driver.
  - Switches the mag vector into ENU, unlike the python driver.
  - Permits more of the configuration vectors to be set.
  - Maintains reset service and option to issue resets on startup.
