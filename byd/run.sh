#!/usr/bin/env bash

set -euo pipefail

USB_CAN_WS="/media/f/nvme_storage/can_ws"
RTK_CAN_WS="/media/f/nvme_storage/can_ws"
ROS2_WS="/media/f/nvme_storage/ros2_ws"
LOG_DATE="$(date +%m%d)"
LOG_DIR="/media/f/nvme_storage/autoware/log/${LOG_DATE}"

next_run_index() {
	local dir="$1"
	local max_index=0
	local entry
	local name
	local index

	shopt -s nullglob
	for entry in "$dir"/*; do
		name="$(basename "$entry")"

		if [[ "$name" =~ ^planning_control_bag_([0-9]+)$ ]]; then
			index=$((10#${BASH_REMATCH[1]}))
		elif [[ "$name" =~ ^planning_control_localization_([0-9]+)\.bag$ ]]; then
			index=$((10#${BASH_REMATCH[1]}))
		elif [[ "$name" =~ ^autoware_([0-9]+)\.log$ ]]; then
			index=$((10#${BASH_REMATCH[1]}))
		elif [[ "$name" =~ ^([0-9]{2})\.log$ ]]; then
			index=$((10#${BASH_REMATCH[1]}))
		else
			continue
		fi

		if (( index > max_index )); then
			max_index=$index
		fi
	done

	printf "%02d" $((max_index + 1))
}

pick_terminal() {
	if command -v gnome-terminal >/dev/null 2>&1; then
		echo "gnome-terminal"
		return
	fi

	echo ""
}

open_terminal_tabs() {
	local terminal_bin="$1"
	local autoware_log_file="$2"
	local bag_output_dir="$3"

	if [[ "$terminal_bin" != "gnome-terminal" ]]; then
		echo "Tab mode requires gnome-terminal."
		echo "Please install gnome-terminal and run again."
		exit 1
	fi

	gnome-terminal --tab --title="1_usb_can" -- \
		bash -ic "cd \"$USB_CAN_WS\" && source install/setup.bash && ros2 run usb_can_driver can_usb_node; exec bash"
	gnome-terminal --tab --title="2_rtk_can" -- \
		bash -ic "cd \"$RTK_CAN_WS\" && source install/setup.bash && ros2 run rtk_can_driver can_rtk_node; exec bash"
	gnome-terminal --tab --title="3_sensing3" -- \
		bash -ic "sensing3; exec bash"
	gnome-terminal --tab --title="4_perception" -- \
		bash -ic "cd \"$ROS2_WS\" && source install/setup.bash && ros2 launch didrive_launch didrive_perception.launch.xml; exec bash"
	gnome-terminal --tab --title="5_autoware3" -- \
		bash -ic "mkdir -p \"$LOG_DIR\" && autoware3 &> \"$autoware_log_file\"; exec bash"
	gnome-terminal --tab --title="6_bag_record" -- \
		bash -ic "mkdir -p \"$LOG_DIR\" && ros2 bag record -e '^/planning/.|^/control/.|^/localization/kinematic_state$' -o \"$bag_output_dir\"; exec bash"
}

main() {
	local terminal_bin
	local run_index
	local autoware_log_file
	local bag_output_dir
	terminal_bin="$(pick_terminal)"

	if [[ -z "$terminal_bin" ]]; then
		echo "No terminal app found. Please install gnome-terminal."
		exit 1
	fi

	mkdir -p "$LOG_DIR"
	run_index="$(next_run_index "$LOG_DIR")"
	autoware_log_file="$LOG_DIR/autoware_${run_index}.log"
	bag_output_dir="$LOG_DIR/planning_control_bag_${run_index}"

	echo "Log directory: $LOG_DIR"
	echo "Run index: $run_index"
	echo "Autoware log: $autoware_log_file"
	echo "Bag output: $bag_output_dir"

	open_terminal_tabs "$terminal_bin" "$autoware_log_file" "$bag_output_dir"

	echo "Launched 1 terminal window with 6 tabs."
}

main "$@"
