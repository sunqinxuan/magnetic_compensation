curr_dir=$(pwd)
echo "make sdk dir: $curr_dir"
if [ ! -d $curr_dir/sdk ]; then
    echo "NOT HAVE SDK DIR!"
    exit 1
fi
rm *.tar.gz
rm -r $curr_dir/sdk/build
rm -r $curr_dir/sdk/install
rm -r $curr_dir/sdk/include
rm -r $curr_dir/sdk/scripts
rm -r $curr_dir/sdk/lib
rm -r $curr_dir/sdk/etc
mkdir sdk/scripts
mkdir sdk/include
ln -vsf $curr_dir/etc $curr_dir/sdk/etc
ln -vsf $curr_dir/3rd_party/eigen-3.3.9 $curr_dir/sdk/include/eigen
ln -vsf $curr_dir/include/api $curr_dir/sdk/include/api
ln -vsf $curr_dir/scripts/set_env.sh $curr_dir/sdk/scripts/set_env.sh
ln -vsf $curr_dir/scripts/build.sh $curr_dir/sdk/scripts/build.sh

echo "LN INSTALL LIB TO SDK LIB!"

if [ ! -d $curr_dir/install ]; then
    echo "NOT HAVE INSTALL DIR, MAKE INSTALL!"
    cd $curr_dir
    . ./scripts/build.sh
fi

cd $curr_dir
if [ ! -d $curr_dir/install ]; then
    echo "STILL NOT HAVE INSTALL DIR!"
    exit 2
fi

ln -vsf $curr_dir/install/lib $curr_dir/sdk/lib

cd $curr_dir
tar -czvhf $curr_dir/mic_sdk.tar.gz $curr_dir/sdk/

echo "TEST SDK COMPILING!"

cd $curr_dir/sdk
. ./scripts/build.sh

cd $curr_dir
unset curr_dir