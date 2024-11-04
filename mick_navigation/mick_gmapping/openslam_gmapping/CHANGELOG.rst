^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package openslam_gmapping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2016-04-23)
------------------
* better Windows compilation
  This is taken from `#9 <https://github.com/ros-perception/openslam_gmapping/issues/9>`_ which can now be closed.
* fix a few more graphics stuff for Qt5
* get GUI back in shape for those interested
* use srand instead of srand48
  srand48 is non-standard and we are using a seed that is an
  unsigned int so we might as well use srand
* Contributors: Vincent Rabaud

0.1.1 (2015-06-25)
------------------
* fix cppcheck warnings
* License from BSD to CC
* Contributors: Isaac IY Saito, Vincent Rabaud

0.1.0 (2013-06-28 17:33:53 -0700)
---------------------------------
- Forked from https://openslam.informatik.uni-freiburg.de/data/svn/gmapping/trunk/
- Catkinized and prepared for release into the ROS ecosystem
