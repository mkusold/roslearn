# ROS Learn

## Goal

## Conventions

1. All units are in accordance with [REP-103](https://www.ros.org/reps/rep-0103.html) unless explicitly specified.

## Setup

### Prerequisites & Configuration

1. Download and setup [docker](https://www.docker.com/get-started) Note: docker-compose is usually packaged with most Docker installations
2. Download [gazebo 9](http://gazebosim.org/tutorials?cat=install) on your local machine. For Macs, just run `brew install gazebo9`.
3. Install a [VNC Viewer](https://www.realvnc.com/en/connect/download/viewer/macos/). If you use Chrome, you can use the Chrome App [VNC Viewer for Google Chrome](https://chrome.google.com/webstore/detail/vnc%C2%AE-viewer-for-google-ch/iabmpiboiopbgfabjmgeedhcmjenhbla?hl=en). The VNC Viewer helps see the GUI for Gazebo.

OSX:

1. Download [XQuartz](https://www.xquartz.org/)
2. Activate the option ‘Allow connections from network clients’ in XQuartz settings
3. run `bash macSetup.sh` and copy your IP address into the `.env` file

### ENV file

1. The `.env` file stores the build and run configuration for the project's docker container.
2. In particular, note the `ENV` varible in `.env` file. `ENV` could be one of `dev`, `test` or `prod`.

## Utilizing Docker

### Building the Container

Change directories into the root directory and run `docker-compose build`.

### Starting the Container

Run `docker-compose up`.

## Development

* For development and testing:
  * In the `.env` file, make sure to change the ENV variable to `dev` - Docker Compose pulls all its environment variables from this `.env` file.
  * In a separate terminal window, run `docker attach roslearn`
  * Go to this project's root directory (where the Makefile exists)
  * To run all packages specified in the d2_bringup launch file, run `make run`.
  * To see all make command options, simply run `make`.
* For production, change the .env `ENV` variable to `prod` and run the docker image which should auto-start

When new packages are created, add a call to its main launch file to the rosweb_bringup package's launch file.

## Testing

You can execute all tests by running `make test`.

### Unit Testing

To execute just unit tests you can run `make coverage`. Coverage reports are generated and can be viewed by opening the cover/index.html file in a browser.

To manually run one python unit test at a time you can run it like so: `python -m unittest test_config`. (note: don't include the .py)

### Node Testing

You can run an individual ROS node tes like so: `rostest sample_package.test`

### Simulation Testing

1. Within the Docker container, run `make gazebo`
2. On your host machine run your VNC Viewer and connect to `localhost:5900` to see the gazebo instance running in the container

If you run into issues with gazebo, try running `--verbose`. If an old crashed gazebo server is still running, you can clear it out by running `killall gzserver`

## Project Quality Checklist

* [ ] README Documentation
* [ ] Changelog
* [ ] Code Documentation
* [ ] Docker Capable
* [ ] Makefile scripts
* [ ] Appropriate Logging
* [ ] Configuration Files
* [ ] Multiple Environment Support
* [ ] Unit Testing
* [ ] Node Unit Testing
* [ ] Integration Testing

### Tmux Hints

* tmux comes installed if you use the Docker container
* You can use tmux to split the screen `tmux && tmux split`
* You can move between screens with `ctrl-b then [up/down arrow key]`
* You can scroll up and down with `ctrl-b then [`.
* You can quit a window with `exit`

### ROS Hints

If you're having problems running the group in development, run `catkin_make` and `source ~/catkin_ws/devel/setup.bash`
