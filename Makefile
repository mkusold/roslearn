# Run 'make help' in the terminal to see a list of script options

.PHONY: help
help: ## Show help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

# GENERAL DEVELOPMENT TOOLING
.PHONY: install
install: ## installs external dependencies
	@(python pm.py && cd ${CATKIN_WS} && catkin_make)

.PHONY: clean
clean: ## Removes compiled python code to ensure it is freshly regenerated on the next run
	@(cd ${CATKIN_WS} && find . -name '*.pyc' -delete && find . -name '__pycache__' -delete)

.PHONY: register
register: ## Makes all necessary code executable so that they can be found and run by the launch file
	@(cd ${CATKIN_WS}/src/${PKG_NAME} && find . -name '*.py' -exec chmod +x {} \;)

.PHONY: update-packages
update-packages: register ## adds and updates new ROS Packages
	@(cd ${CATKIN_WS} && catkin_make)

.PHONY: graph
graph: ## Shows ROS node graph
	@(DISPLAY=${HOST_IP}:0 bash -c 'rqt_graph')

.PHONY: vnc
vnc: ## sets up VNC Viewer connections
	@(Xvfb :1 -screen 0 1600x1200x16 & \
	x11vnc -ncache 10 -q -o ~/vncLogfile & > /dev/null 2>&1 )

# TESTING
.PHONY: coverage
coverage: ## Creates coverage report for unit tests
	@echo "Running unit tests for package 1"
	# @(cd ${CATKIN_WS}/src/${PKG_NAME}/package_1 \
	# 	&& nosetests --cover-erase -c ${CATKIN_WS}/src/${PKG_NAME}/.noserc \
	# 	&& (echo "=== See interactive Report at cover/index.html ==="))

.PHONY: test
test: coverage ## Runs Tests including ROS Node tests and unit tests (via the coverage command)
	@(cd ${CATKIN_WS} && catkin_make run_tests)

# RUNNING
.PHONY: run
run: ## Starts all ROS packages as dictated by the bringup package
	@(cd ${CATKIN_WS} && roslaunch ${PKG_NAME}_bringup ${PKG_NAME}_bringup.launch)

