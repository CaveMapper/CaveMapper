import bpy
import pkg_resources
import subprocess
import sys

def lib_ver_checker(package_name):
    pkg_resources._initialize()
    installed_packages = pkg_resources.WorkingSet()
    package_version = None
    
    for dist in installed_packages:
        if dist.key == package_name:
            package_version = dist.version
            break

    return package_version

def install_packeage(package_name,target_version):
    current_version = lib_ver_checker(package_name)
    print(current_version)
    print(target_version)
    
    if current_version != target_version:
        if current_version != None:
            print(f"{package_name} is installed with different version({current_version}).Uninstalling...")
            subprocess.check_call([sys.executable, "-m", "pip", "uninstall", "-y", package_name])
            print(f"Uninstalling finished")
        print(f"Installing {package_name}=={target_version}...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", f"{package_name}=={target_version}"])
        print(f"Installing finished")
    else:
        print(f"{package_name}=={target_version} is already installed")
               
 
bpy.ops.wm.console_toggle()
install_packeage('open3d','0.17.0')