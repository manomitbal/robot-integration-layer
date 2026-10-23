import os
# Minimal stand-in for ament_index_python so xacro can resolve $(find pkg) without a ROS install.
PKGS = {
    "ur_description": os.path.abspath("vendor/Universal_Robots_ROS2_Description"),
    "fanuc_crx_description": os.path.abspath("vendor/fanuc_description/fanuc_crx_description"),
    "spot_description": os.path.abspath("vendor/spot_description/spot_description"),
}
def get_package_share_directory(p):
    return PKGS[p]
