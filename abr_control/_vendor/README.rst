***********************
Vendorized dependencies
***********************

This directory contains ABR_Control dependencies
that have been vendorized.
A vendorized dependency is shipped with ABR_Control
to allow for easy offline install.

To add a new vendorized dependency,
add it to ``abr_control/_vendor/requirements.txt`` and run

.. code:: bash

   pip install --target abr_control/_vendor -r abr_control/_vendor/requirements.txt

from the ABR_Control root directory.

To update a vendorized dependency,
change the version number associated with that package
in ``abr_control/_vendor/requirements.txt``
and rerun the above command
from the ABR_Control root directory.
