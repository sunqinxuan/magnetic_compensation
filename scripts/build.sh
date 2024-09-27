build_curr_dir=$(pwd)
echo "build dir: $build_curr_dir"
mkdir $build_curr_dir/build
cd $build_curr_dir/build || exit
cmake ..
make -j12 install
cd $build_curr_dir
unset build_curr_dir