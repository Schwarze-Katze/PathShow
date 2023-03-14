source devel/setup.bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd src/plan_path/launch/
roslaunch demo.launch output_path:=$SCRIPT_DIR/src/output

