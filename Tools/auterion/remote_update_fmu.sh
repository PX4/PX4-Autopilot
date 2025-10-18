#!/bin/bash
# Flash PX4 to a device running AuterionOS in the local network
if [ "$1" == "-h" ] || [ "$1" == "--help" ] || [ $# -lt 2 ]; then
	echo "Usage: $0 -f <firmware.px4|.elf> [-c <configuration_dir>] -d <IP/Device> [-u <user>] [-p <ssh_port>] [--revert]"
	exit 1
fi

ssh_port=22
ssh_user=root
ssh_opts="-o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no"

while getopts ":f:c:d:p:u:r" opt; do
	case ${opt} in
		f )
			if [ -n "$OPTARG" ]; then
				firmware_file="$OPTARG"
			else
				echo "ERROR: -f requires a non-empty option argument."
				exit 1
			fi
			;;
		c )
			if [ -f "$OPTARG/rc.autostart" ]; then
				config_dir="$OPTARG"
			else
				echo "ERROR: -c configuration directory is empty or does not contain a valid rc.autostart"
				exit 1
			fi
			;;
		d )
			if [ "$OPTARG" ]; then
				device="$OPTARG"
			else
				echo "ERROR: -d requires a non-empty option argument."
				exit 1
			fi
			;;
		p )
			if [[ "$OPTARG" =~ ^[0-9]+$ ]]; then
				ssh_port="$OPTARG"
			else
				echo "ERROR: -p ssh_port must be a number."
				exit 1
			fi
			;;
		u )
			if [ "$OPTARG" ]; then
				ssh_user="$OPTARG"
			else
				echo "ERROR: -u requires a non-empty option argument."
				exit 1
			fi
			;;
		r )
			revert=true
			;;
	esac
done

if [ -z "$device" ]; then
	echo "Error: missing device"
	exit 1
fi

target_dir=/shared_container_dir/fmu
target_file_name="update-dev.tar"

if [ "$revert" == true ]; then
	# revert to the release version which was originally deployed
	cmd="cp $target_dir/update.tar $target_dir/$target_file_name"
	ssh $ssh_opts -t -p $ssh_port $ssh_user@$device "$cmd"
else
	# create custom update-dev.tar
	tmp_dir="$(mktemp -d)"
	config_path=""
	firmware_path=""

	if [ -d "$config_dir" ]; then
		cp -r "$config_dir" "$tmp_dir/config"
		config_path=config
	fi

	if [ -f "$firmware_file" ]; then
		extension="${firmware_file##*.}"
		cp "$firmware_file" "$tmp_dir/firmware.$extension"
		if [ "$extension" == "elf" ]; then
			# ensure the file is stripped to reduce file size
			arm-none-eabi-strip "$tmp_dir/firmware.$extension"
		fi
		firmware_path="firmware.$extension"
	fi

	pushd "$tmp_dir" &>/dev/null

	if [ -z $firmware_path ] && [ -z $config_path ]; then
		exit 1
	fi

	tar_name="tar"

	if [ -x "$(command -v gtar)" ]; then
		# check if gnu-tar is installed on macOS and use that instead
		tar_name="gtar"
	fi

	$tar_name -C "$tmp_dir" --sort=name --owner=root:0 --group=root:0 --mtime='2019-01-01 00:00:00' -cvf $target_file_name $firmware_path $config_path

	# send it to the target to start flashing
	scp $ssh_opts -P $ssh_port "$target_file_name" $ssh_user@"$device":$target_dir
	popd &>/dev/null
	rm -rf "$tmp_dir"
fi

# grab status output for flashing progress
cmd="tail --follow=name $target_dir/update_status 2>/dev/null || true"
ssh $ssh_opts -t -p $ssh_port $ssh_user@$device "$cmd"
