import subprocess
import os
import sys


"""
  This script acts as the Package Manager (PM).
  It handles downloading external dependencies linked to git repos into this project.
"""


def check_for_dependency_definition_file():
  dependencyListExists = os.path.exists("./package.txt")
  
  if(not dependencyListExists):
    print('package.txt is not found in the root directory.')
    sys.exit()

  return True


def get_dependency_list():
  # read in dependencies and strip the newlines    
  with open('package.txt') as package_file:
    dependencies = [line.rstrip() for line in package_file]
    return dependencies


def determine_dependency_directory(dependency_git_url):
  git_url = dependency_git_url
  if(dependency_git_url.endswith('.git')):
    git_url = os.path.splitext(dependency_git_url)[0]

  directory_name = os.path.split(git_url)[1]
  return directory_name


def install_dependency(dependency_git_url):
  if ( (not dependency_git_url.endswith('.git'))
    and ("github.com" not in dependency_git_url.lower()) ):
    print("Dependency: %s could not be installed. This is not a valid git repo" % (dependency_git_url))
    return
  
  directory_name = determine_dependency_directory(dependency_git_url)
  if(os.path.isdir("./external_packages/%s" % (directory_name)) ):
    print("Package: %s is already installed. Skipping..." % (directory_name))
  else:
    install_command = "cd ./external_packages && git clone %s %s" % (dependency_git_url, directory_name)
    subprocess.call(install_command, shell=True)
    print("Successfully Installed: %s" % (dependency_git_url))


def main():
  check_for_dependency_definition_file()
  dependencies = get_dependency_list()

  # ensure there is an external_packages directory to install dependencies into
  subprocess.call(["mkdir", "-p", "./external_packages"])

  for dependency in dependencies:
    install_dependency(dependency)


main()