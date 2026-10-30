from setuptools import find_packages, setup
from glob import glob
from pathlib import Path
setup(name="ugv_relocalization", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/ugv_relocalization"]),
                  ("share/ugv_relocalization", ["package.xml", "README.md", "DECISIONS.md", "ROBOT_TEST_PLAN.md", "requirements.txt"])] +
                 [("share/ugv_relocalization/" + d, [f for f in glob(d + "/*") if Path(f).is_file()]) for d in ["launch", "config", "maps", "scripts"]],
      install_requires=["setuptools"], extras_require={"test": ["pytest"]}, zip_safe=False,
      maintainer="Om Pug", maintainer_email="ompug@users.noreply.github.com",
      description="Passive Scout Mini relocalization and graph pose integration", license="Apache-2.0",
      entry_points={"console_scripts": ["relocalize = ugv_relocalization.node:main",
          "pose_bridge = ugv_relocalization.bridge:main", "calibrate = ugv_relocalization.calibrate:main",
          "coverage = ugv_relocalization.coverage:main", "bag_crosscheck = ugv_relocalization.bag_crosscheck:main"]})
