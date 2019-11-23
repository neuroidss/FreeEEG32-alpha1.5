#!/bin/bash
BuildType=Release
BuildOption=--release
base_dir=$(dirname "$(readlink -f "$0")")
work_dir=`pwd`
build_dir_base="${work_dir}/build"
install_dir_base="${work_dir}/dist"
dependencies_dir="${work_dir}/dependencies"
user_data_subdir="openvibe-2.2.0"

while [[ $# -gt 0 ]]; do
	key="$1"
	case $key in
		-h | --help)
			echo "-d | --debug : build as debug"
			echo "-r | --release : build as release"
			echo "--build-dir <dirname> : directory for build files"
			echo "--install-dir <dirname> : binaries deployment directory"
			exit
			;;
		-d | --debug)
			BuildType=Debug
			BuildOption=--debug
			;;
		-r | --release)
			BuildType=Release
			BuildOption=--release
			;;
		--build-dir)
			build_dir_base="$2"
			shift
			;;
		--install-dir)
			install_dir_base="$2"
			shift
			;;
		--userdata-subdir)
			user_data_subdir="$2"
			shift
			;;
		--dependencies-dir)
			dependencies_dir="$2"
			shift
			;;
		*)
			echo "ERROR: Unknown parameter $i"
			exit 1
			;;
	esac
	shift
done

echo Building sdk
cd ${base_dir}/sdk/scripts
./unix-build --build-type ${BuildType} --build-dir ${build_dir_base}/sdk-${BuildType} --install-dir ${install_dir_base}/sdk-${BuildType} --dependencies-dir ${dependencies_dir} --userdata-subdir ${user_data_subdir} --build-unit --build-validation --test-data-dir ${dependencies_dir}/test-input
if [[ ! $? -eq 0 ]]; then
	echo "Error while building sdk"
	exit 1
fi

echo Building designer
cd ${base_dir}/designer/scripts
./unix-build --build-type=${BuildType} --build-dir=${build_dir_base}/designer-${BuildType} --install-dir=${install_dir_base}/designer-${BuildType} --sdk=${install_dir_base}/sdk-${BuildType} --userdata-subdir=${user_data_subdir}
if [[ ! $? -eq 0 ]]; then
	echo "Error while building designer"
	exit 1
fi

echo Building extras
cd ${base_dir}/extras/scripts
./linux-build ${BuildOption} --build-dir ${build_dir_base}/extras-${BuildType} --install-dir ${install_dir_base}/extras-${BuildType} --sdk ${install_dir_base}/sdk-${BuildType} --designer ${install_dir_base}/designer-${BuildType} --dependencies-dir ${dependencies_dir} --userdata-subdir ${user_data_subdir}
if [[ ! $? -eq 0 ]]; then
	echo "Error while building extras"
	exit 1
fi
